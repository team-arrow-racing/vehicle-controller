#![no_main]
#![no_std]
#![feature(type_alias_impl_trait)]
#![allow(clippy::transmute_ptr_to_ptr)]

use panic_probe as _;
use defmt_rtt as _; // global logger

use stm32h7xx_hal::{
    gpio::{
        gpiob::PB14,
        gpioc::{PC0, PC1, PC13, PC3},
        gpiod::{PD0, PD1},
        gpioe::PE1,
        gpioi::PI14,
        Edge, ExtiPin, Input, Output, PushPull, Speed
    },
    can::Can,
    device::FDCAN1,
    dma::{dma::{DmaConfig, StreamsTuple, Stream0, Stream1}, DBTransfer, MemoryToPeripheral, PeripheralToMemory, Transfer},
    rtc::{Rtc, RtcClock},
    rcc::{self, rec::{self, FdcanClkSel}, CoreClocks},
    prelude::*,
    serial::{self, config::Config, Event},
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
use core::{mem, mem::MaybeUninit};
use cortex_m_rt::entry;

use rtic_monotonics::{systick::*, Monotonic};

mod can_tasks;
mod lights_tasks;
mod serial_tasks;

use crate::can_tasks::*;
use crate::lights_tasks::*;
use crate::serial_tasks::*;

#[rtic::app(device = stm32h7xx_hal::pac, dispatchers = [UART4, SPI1])]
mod app {
    type FdCanMode = ExternalLoopbackMode;
    type RxTransfer<'a> = Transfer<
        Stream1<stm32::DMA1>,
        serial::Rx<stm32::USART2>,
        PeripheralToMemory, 
        &'static mut [u8; 10],
        DBTransfer
    >;
    type TxTransfer = Transfer<
        Stream0<stm32::DMA1>,
        serial::Tx<stm32::USART2>,
        MemoryToPeripheral,
        &'static mut [u8; 10],
        DBTransfer
    >;
    
    use super::*;
    // Shared resources go here
    #[shared]
    struct Shared {
        status_led: PE1<Output<PushPull>>,
        fdcan1_ctrl: FdCanControl<Can<FDCAN1>, FdCanMode>,
        fdcan1_tx: Tx<Can<FDCAN1>, FdCanMode>,
        fdcan1_rx0: Rx<Can<FDCAN1>, FdCanMode, Fifo0>,
        fdcan1_rx1: Rx<Can<FDCAN1>, FdCanMode, Fifo1>,
        is_left_ind_on: bool,
        is_right_ind_on: bool,
        // rtc: Rtc,
        receive: RxTransfer<'static>,
        transfer: TxTransfer
    }

    // Local resources go here
    #[local]
    struct Local {
        button: PC13<Input>,
        watchdog: IndependentWatchdog,
        // serial_rx: serial::Rx<stm32::USART2>,
        // serial_tx: serial::Tx<stm32::USART2>,
        left_ind_light: PC1<Output<PushPull>>,
        right_ind_light: PC0<Output<PushPull>>,
        rx_buffer: Option<&'static mut [u8; 10]>,
        buf_index: usize
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
            .pll1_q_ck(200.MHz())
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

        // Lights
        let left_ind_light = gpioc.pc1.into_push_pull_output();
        let right_ind_light = gpioc.pc0.into_push_pull_output();

        // USART
        let (mut serial_tx, mut serial_rx) = {
            let tx = gpiod.pd5.into_alternate();
            let rx = gpiod.pd6.into_alternate();

            let config = Config::new(115_200.bps()).lastbitclockpulse(true);

            let mut serial = cx.device.USART2.serial(
                (tx, rx),
                config,
                ccdr.peripheral.USART2,
                &ccdr.clocks,
            ).unwrap();

            serial.listen(Event::Rxne);
            serial.enable_dma_rx();
            serial.split()
        };    

        let tx_buffer = cortex_m::singleton!(: [u8; 10] = [0; 10]).unwrap();
        let rx_buffer1 = cortex_m::singleton!(: [u8; 10] = [0; 10]).unwrap();
        let rx_buffer2 = cortex_m::singleton!(: [u8; 10] = [0; 10]).unwrap();

        // Setup the DMA transfer on stream 0
        //
        // We need to specify the direction with a type annotation, since DMA
        // transfers both to and from the UART are possible
        let streams = StreamsTuple::new(cx.device.DMA1, ccdr.peripheral.DMA1);
        
        let config = DmaConfig::default().memory_increment(true);

        let transfer: Transfer<_, _, MemoryToPeripheral, _, _> =
            Transfer::init(streams.0, serial_tx, tx_buffer, None, config);
        
        let mut receive: Transfer<_, _, PeripheralToMemory, _, _> = 
            Transfer::init(
                streams.1,
                serial_rx,
                rx_buffer1,
                None,
                DmaConfig::default()
                    .memory_increment(true)
                    .fifo_enable(true)
                    .fifo_error_interrupt(true)
                    .transfer_complete_interrupt(true)
            );

        receive.start(|_rx| {});
    
        defmt::info!("Chunked transfer complete!");
        
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
        transmit_uart::spawn().ok();
        // serial_gen::spawn().ok();
        // can_gen::spawn().ok();

        defmt::info!("Initialisation finished.");

        (
            Shared {
                status_led: gpioe.pe1.into_push_pull_output(),
                fdcan1_ctrl,
                fdcan1_tx,
                fdcan1_rx0,
                fdcan1_rx1,
                is_left_ind_on: false,
                is_right_ind_on: false,
                receive,
                transfer
                // rtc,
            },
            Local { 
                button,
                watchdog,
                // serial_rx,
                // serial_tx,
                left_ind_light,
                right_ind_light,
                rx_buffer: Some(rx_buffer2),
                buf_index: 0
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

    #[task(binds = EXTI15_10, shared = [status_led], local = [button])]
    fn button_click(mut cx: button_click::Context) {
        cx.local.button.clear_interrupt_pending_bit();
        cx.shared.status_led.lock(|led| {
            led.toggle();
        });
    }

    #[task(shared = [status_led], priority=1)]
    async fn blink(mut cx: blink::Context) {
        loop {
            cx.shared.status_led.lock(|led| {
                defmt::trace!("blinking");
                led.toggle();
            });
            Systick::delay(1000_u64.millis()).await;
        }
    }

    #[task(priority=1, shared=[transfer])]
    async fn transmit_uart(mut cx: transmit_uart::Context) {
        loop {
            cx.shared.transfer.lock(|transfer| {
                transfer.start(|serial| {
                    // This closure runs right after enabling the stream
                    serial.enable_dma_tx();
                });        
                defmt::info!("Started transfer!");
        
                // Wait for transfer to complete
                while !transfer.get_transfer_complete_flag() {}
        
                defmt::info!("Continuing with chunked transfer!");
        
                transfer.pause(|serial| {
                    // At this point, the DMA transfer is done, but the data is still in the
                    // UART output FIFO. Wait for it to complete
                    while !serial.is_txe() {}
                });        
            });
            Systick::delay(1000_u64.millis()).await;
        }
    }

    #[task(binds = USART2, priority=1, local = [rx_buffer], shared = [receive])]
    fn usart2_callback(mut cx: usart2_callback::Context) {
        cx.shared.receive.lock(|transfer| {
            // defmt::info!("usart2_callback");            
            if transfer.get_transfer_complete_flag() {
                // let bytes_count = 10 - transfer;
                defmt::info!("info");
                let new_buffer = cx.local.rx_buffer.take().unwrap();

                let (buffer, _, _) = transfer.next_transfer(new_buffer).unwrap();

                let _bytes = &buffer[..2];

                defmt::info!("{:?}", _bytes);

                *cx.local.rx_buffer = Some(buffer);
            }
        });
    }

    #[task(binds = DMA1_STR1, priority=1, shared=[receive])]
    fn dma_callback(mut cx: dma_callback::Context) {
        defmt::info!("dma_callback");
        cx.shared.receive.lock(|transfer| {
            if transfer.get_transfer_complete_flag() {
                defmt::info!("dma done");
            }
        });
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

        #[task(shared = [is_left_ind_on, is_right_ind_on], local = [left_ind_light, right_ind_light], priority=1)]
        async fn toggle_indicators(mut cx: toggle_indicators::Context);

        #[task(priority=1)]
        async fn toggle_brakes(mut cx: toggle_brakes::Context);

        // #[task(binds = USART2, local = [serial_rx, rx_buffer, buf_index])]
        // fn usart2_callback(mut cx: usart2_callback::Context);

        // #[task(local = [serial_tx], priority=1)]
        // async fn serial_gen(mut cx: serial_gen::Context);
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