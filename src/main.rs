#![no_main]
#![no_std]

use defmt_rtt as _;
use panic_probe as _;

use bxcan::{filter::Mask32, Interrupts};
use stm32l4xx_hal::{
    can::Can,
    gpio::{Alternate, Output, PushPull, PA10, PA9, PB13},
    pac::USART1,
    prelude::*,
    serial::{Config, Serial},
    watchdog::IndependentWatchdog,
};
use systick_monotonic::{
    fugit::{MillisDurationU32, MillisDurationU64},
    Systick,
};

type Duration = MillisDurationU64;

mod comms;
use comms::msg;

mod queued_can;
use queued_can::QueuedCan;

use solar_car::device;

use wurth_calypso::Calypso;

#[rtic::app(device = stm32l4xx_hal::pac, dispatchers = [SPI1])]
mod app {
    use super::*;

    #[monotonic(binds = SysTick, default = true)]
    type MonoTimer = Systick<1000>;

    #[shared]
    struct Shared {
        can: QueuedCan,
        calypso: Calypso<
            Serial<
                USART1,
                (PA9<Alternate<PushPull, 7>>, PA10<Alternate<PushPull, 7>>),
            >,
        >,
    }

    #[local]
    struct Local {
        watchdog: IndependentWatchdog,
        status_led: PB13<Output<PushPull>>,
    }

    #[init]
    fn init(cx: init::Context) -> (Shared, Local, init::Monotonics) {
        defmt::info!("starting init.");

        // peripherals
        let mut flash = cx.device.FLASH.constrain();
        let mut rcc = cx.device.RCC.constrain();
        let mut pwr = cx.device.PWR.constrain(&mut rcc.apb1r1);
        let mut gpioa = cx.device.GPIOA.split(&mut rcc.ahb2);
        let mut gpiob = cx.device.GPIOB.split(&mut rcc.ahb2);

        // configure system clock
        let clocks = rcc.cfgr.sysclk(80.MHz()).freeze(&mut flash.acr, &mut pwr);

        // configure monotonic time
        let mono = Systick::new(cx.core.SYST, clocks.sysclk().to_Hz());

        // configure status led
        let status_led = gpiob
            .pb13
            .into_push_pull_output(&mut gpiob.moder, &mut gpiob.otyper);

        // configure can bus
        let can = {
            let rx = gpioa.pa11.into_alternate(
                &mut gpioa.moder,
                &mut gpioa.otyper,
                &mut gpioa.afrh,
            );
            let tx = gpioa.pa12.into_alternate(
                &mut gpioa.moder,
                &mut gpioa.otyper,
                &mut gpioa.afrh,
            );

            let can = bxcan::Can::builder(Can::new(
                &mut rcc.apb1r1,
                cx.device.CAN1,
                (tx, rx),
            ))
            .set_bit_timing(0x001c_0009); // 500kbit/s

            let mut can = can.enable();
            can.modify_filters().enable_bank(0, Mask32::accept_all());
            can.enable_interrupts(
                Interrupts::TRANSMIT_MAILBOX_EMPTY
                    | Interrupts::FIFO0_MESSAGE_PENDING
                    | Interrupts::FIFO1_MESSAGE_PENDING,
            );
            nb::block!(can.enable_non_blocking()).unwrap();

            let mut can = QueuedCan::new(can);

            can.transmit(msg::startup()).unwrap();

            can
        };

        // broadcast startup message.

        // configure watchdog
        let watchdog = {
            let mut wd = IndependentWatchdog::new(cx.device.IWDG);
            wd.stop_on_debug(&cx.device.DBGMCU, true);
            wd.start(MillisDurationU32::millis(100));

            wd
        };

        // configure calypso
        let calypso = {
            let tx = gpioa.pa9.into_alternate(
                &mut gpioa.moder,
                &mut gpioa.otyper,
                &mut gpioa.afrh,
            );
            let rx = gpioa.pa10.into_alternate(
                &mut gpioa.moder,
                &mut gpioa.otyper,
                &mut gpioa.afrh,
            );

            let serial = Serial::usart1(
                cx.device.USART1,
                (tx, rx),
                Config::default().baudrate(921600.bps()).parity_even(),
                clocks,
                &mut rcc.apb2,
            );

            Calypso::new(serial)
        };

        // start heartbeat
        heartbeat::spawn_after(Duration::millis(1000)).unwrap();

        // start main loop
        run::spawn().unwrap();

        defmt::info!("finished init.");

        (
            Shared { can, calypso },
            Local {
                watchdog,
                status_led,
            },
            init::Monotonics(mono),
        )
    }

    #[task(shared = [can], local = [watchdog])]
    fn run(mut cx: run::Context) {
        cx.local.watchdog.feed();

        cx.shared.can.lock(|can| {
            can.try_transmit();
        });

        run::spawn_after(Duration::millis(10)).unwrap();
    }

    #[task(shared = [can], local = [status_led])]
    fn heartbeat(mut cx: heartbeat::Context) {
        if cx.local.status_led.is_set_low() {
            defmt::debug!("heartbeat!");

            cx.local.status_led.set_high();

            // send heartbeat message
            cx.shared.can.lock(|can| {
                can.transmit(device::heartbeat_msg(
                    device::Device::VehicleController,
                ))
                .unwrap();
            });

            heartbeat::spawn_after(Duration::millis(100)).unwrap();
        } else {
            cx.local.status_led.set_low();

            heartbeat::spawn_after(Duration::millis(900)).unwrap();
        }
    }

    /// Triggers on RX mailbox event.
    #[task(shared = [can], binds = CAN1_RX0)]
    fn can_rx0_pending(mut cx: can_rx0_pending::Context) {
        cx.shared.can.lock(|can| {
            can.try_receive().unwrap();
        });
    }

    /// Triggers on RX mailbox event.
    #[task(shared = [can], binds = CAN1_RX1)]
    fn can_rx1_pending(mut cx: can_rx1_pending::Context) {
        cx.shared.can.lock(|can| {
            can.try_receive().unwrap();
        });
    }

    /// triggers on TX mailbox empty.
    #[task(shared = [can], binds = CAN1_TX)]
    fn can_tx_empty(mut cx: can_tx_empty::Context) {
        cx.shared.can.lock(|can| {
            // try and send another message if there is one queued.
            can.try_transmit();
        });
    }

    #[idle]
    fn idle(_: idle::Context) -> ! {
        defmt::trace!("idling.");
        loop {
            cortex_m::asm::nop();
        }
    }

    defmt::timestamp!("{=u64}ms", { monotonics::now().ticks() });
}

// same panicking *behavior* as `panic-probe` but doesn't print a panic message
// this prevents the panic message being printed *twice* when `defmt::panic` is invoked
#[defmt::panic_handler]
fn panic() -> ! {
    cortex_m::asm::udf()
}
