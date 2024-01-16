#![no_main]
#![no_std]
#![feature(type_alias_impl_trait)]

use panic_probe as _;
use defmt_rtt as _; // global logger

use stm32h7xx_hal::{
    gpio::{
        gpiob::PB14,
        gpioc::{PC13, PC3},
        gpiod::{PD0, PD1},
        gpioe::PE1,
        gpioi::PI14,
        Edge, ExtiPin, Input, Output, PushPull, Speed
    },
    can::Can,
    device::FDCAN1,
    rtc::{Rtc, RtcClock},
    rcc::{self, rec::{self, FdcanClkSel}, CoreClocks},
    prelude::*,
    serial,
    independent_watchdog::IndependentWatchdog,
    nb::block,
    stm32
};

use fdcan::{
    config::{DataBitTiming, NominalBitTiming},
    FdCanControl, Fifo0, Fifo1, NormalOperationMode, Rx, Tx,
    ExternalLoopbackMode,
    frame::{FrameFormat, TxFrameHeader, RxFrameInfo},
    id::{ExtendedId, StandardId, Id},
    interrupt::{Interrupt, Interrupts, InterruptLine}
};

use core::num::{NonZeroU16, NonZeroU8};
use core::fmt::Write;

use rtic_monotonics::systick::*;
use rtic_monotonics::Monotonic;

mod can_tasks;
use crate::can_tasks::*;

#[rtic::app(device = stm32h7xx_hal::pac, dispatchers = [UART4, SPI1])]
mod app {
    type FdCanMode = ExternalLoopbackMode;

    use stm32h7xx_hal::serial::Event;

    use super::*;
    // Shared resources go here
    #[shared]
    struct Shared {
        led: PE1<Output<PushPull>>,
        fdcan1_ctrl: FdCanControl<Can<FDCAN1>, FdCanMode>,
        fdcan1_tx: Tx<Can<FDCAN1>, FdCanMode>,
        fdcan1_rx0: Rx<Can<FDCAN1>, FdCanMode, Fifo0>,
        fdcan1_rx1: Rx<Can<FDCAN1>, FdCanMode, Fifo1>,
        // rtc: Rtc,
    }

    // Local resources go here
    #[local]
    struct Local {
        button: PC13<Input>,
        watchdog: IndependentWatchdog,
        serial_rx: serial::Rx<stm32::USART2>,
        serial_tx: serial::Tx<stm32::USART2>
    }

    #[init]
    fn init(mut cx: init::Context) -> (Shared, Local) {
        defmt::info!("init");
        // configure power domain
        let mut pwr = cx
            .device
            .PWR
            .constrain()
            .backup_regulator()
            .smps()
            .vos0(&cx.device.SYSCFG)
            .freeze();
        let backup = pwr.backup().unwrap();

        // RCC
        let rcc = cx.device.RCC.constrain();
        let ccdr = rcc
            .sysclk(480.MHz())
            .pll1_strategy(rcc::PllConfigStrategy::Iterative)
            .pll1_q_ck(32.MHz())
            .freeze(pwr, &cx.device.SYSCFG);

        // GPIO
        let gpioa = cx.device.GPIOA.split(ccdr.peripheral.GPIOA);
        let gpiob = cx.device.GPIOB.split(ccdr.peripheral.GPIOB);
        let gpioc = cx.device.GPIOC.split(ccdr.peripheral.GPIOC);
        let gpiod = cx.device.GPIOD.split(ccdr.peripheral.GPIOD);
        let gpioe = cx.device.GPIOE.split(ccdr.peripheral.GPIOE);
        let gpiog = cx.device.GPIOG.split(ccdr.peripheral.GPIOG);
        let gpioh = cx.device.GPIOH.split(ccdr.peripheral.GPIOH);

        // Button
        let mut button = gpioc.pc13.into_floating_input();
        button.make_interrupt_source(&mut cx.device.SYSCFG);
        button.trigger_on_edge(&mut cx.device.EXTI, Edge::Rising);
        button.enable_interrupt(&mut cx.device.EXTI);

        // USART
        let (mut serial_tx, mut serial_rx) = {
            let tx = gpiod.pd5.into_alternate();
            let rx = gpiod.pd6.into_alternate();

            let mut serial = cx.device.USART2.serial(
                (tx, rx),
                1_152_000.bps(),
                ccdr.peripheral.USART2,
                &ccdr.clocks,
            ).unwrap();

            serial.listen(Event::Rxne);

            serial.split()
        };

        // CAN
        let (fdcan1_ctrl, fdcan1_tx, fdcan1_rx0, fdcan1_rx1) = {
            let fdcan_prec = ccdr.peripheral.FDCAN.kernel_clk_mux(FdcanClkSel::Pll1Q);

            let rx = gpiod.pd0.into_alternate().speed(Speed::VeryHigh);
            let tx = gpiod.pd1.into_alternate().speed(Speed::VeryHigh);

            let mut can = cx.device.FDCAN1.fdcan(tx, rx, fdcan_prec);

            // throw error rather than trying to handle unexpected bus behaviour
            can.set_protocol_exception_handling(false);

            // k-clock 32MHz, bit rate 500kbit/s, sample point 87.5%
            can.set_nominal_bit_timing(NominalBitTiming {
                prescaler: NonZeroU16::new(4).unwrap(),
                seg1: NonZeroU8::new(13).unwrap(),
                seg2: NonZeroU8::new(2).unwrap(),
                sync_jump_width: NonZeroU8::new(1).unwrap(),
            });
            // k-clock 32MHz, bit rate 500kbit/s, sample point 87.5%
            can.set_data_bit_timing(DataBitTiming {
                prescaler: NonZeroU8::new(4).unwrap(),
                seg1: NonZeroU8::new(13).unwrap(),
                seg2: NonZeroU8::new(2).unwrap(),
                sync_jump_width: NonZeroU8::new(1).unwrap(),
                transceiver_delay_compensation: true,
            });

            can.enable_interrupt_line(InterruptLine::_0, true);
            can.enable_interrupts(Interrupts::RX_FIFO0_NEW_MSG | Interrupts::RX_FIFO1_NEW_MSG);

            can.into_external_loopback().split()
        };

        // setup and start independent watchdog
        // initialisation must complete before the watchdog triggers
        let watchdog = {
            let mut wd = IndependentWatchdog::new(cx.device.IWDG1);
            wd.start(100_u32.millis());
            wd
        };

        // configure real-time clock
        // let rtc = {
        //     Rtc::open_or_init(
        //         cx.device.RTC,
        //         backup.RTC,
        //         RtcClock::Lse {
        //             freq: 32_768.Hz(),
        //             bypass: false,
        //             css: false,
        //         },
        //         &ccdr.clocks,
        //     )
        // };
    
        // Monotonics
        let token = rtic_monotonics::create_systick_token!();
        Systick::start(cx.core.SYST, ccdr.clocks.sysclk().to_Hz(), token);

        watchdog::spawn().ok();
        blink::spawn().ok();
        // serial_gen::spawn().ok();
        can_gen::spawn().ok();

        defmt::info!("Initialisation finished.");

        (
            Shared {
                led: gpioe.pe1.into_push_pull_output(),
                fdcan1_ctrl,
                fdcan1_tx,
                fdcan1_rx0,
                fdcan1_rx1,
                // rtc,
            },
            Local { 
                button,
                watchdog,
                serial_rx,
                serial_tx
            },
        )
    }

    #[task(local = [watchdog])]
    async fn watchdog(cx: watchdog::Context) {
        loop {
            cx.local.watchdog.feed();
            Systick::delay(80_u64.millis()).await;
        }
    }

    #[task(binds = EXTI15_10, shared = [led], local = [button])]
    fn button_click(mut cx: button_click::Context) {
        cx.local.button.clear_interrupt_pending_bit();
        cx.shared.led.lock(|led| {
            led.toggle();
        });
    }

    #[task(binds = USART2, local = [serial_rx])]
    fn usart2_callback(mut cx: usart2_callback::Context) {
        defmt::info!("usart2");
        let rx = cx.local.serial_rx;

        defmt::info!("{:?}", block!(rx.read()));
    }

    #[task(shared = [led], priority=1)]
    async fn blink(mut cx: blink::Context) {
        loop {
            cx.shared.led.lock(|led| {
                defmt::trace!("blinking");
                led.toggle();
            });
            Systick::delay(1000_u64.millis()).await;
        }
    }

    #[task(local = [serial_tx], priority=1)]
    async fn serial_gen(mut cx: serial_gen::Context) {
        loop {
            defmt::info!("writing");
            let tx = &mut *cx.local.serial_tx;
            writeln!(tx, "Hello, world!").unwrap();

            Systick::delay(1000_u64.millis()).await;
        }
    }

    extern "Rust" {   
        #[task(shared = [fdcan1_tx], priority=1)]
        async fn can_gen(mut cx: can_gen::Context);

        #[task(binds = FDCAN1_IT0, priority = 2, shared = [fdcan1_rx0])]
        fn can_rx0_pending(mut cx: can_rx0_pending::Context);

        #[task(binds = FDCAN1_IT1, priority = 2, shared = [fdcan1_rx1])]
        fn can_rx1_pending(mut cx: can_rx1_pending::Context);

        #[task(priority=1)]
        async fn can_receive(mut cx: can_receive::Context, frame: RxFrameInfo, buffer: [u8; 8]);
    }
}

// same panicking *behavior* as `panic-probe` but doesn't print a panic message
// this prevents the panic message being printed *twice* when `defmt::panic` is invoked
#[defmt::panic_handler]
fn panic() -> ! {
    cortex_m::asm::udf()
}

defmt::timestamp!("{=u64:us}", {
    Systick::now().duration_since_epoch().to_micros()
});