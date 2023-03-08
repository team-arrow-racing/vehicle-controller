//! Vehicle Controller
//!
//! Main ECU for the solar car.
//!
//! # Task priority assignment
//!
//! It would be more idomatic to have these assigned in a enum or some constants
//! but RTIC doesn't yet support variables (static or otherwise) in task
//! definitions.
//!
//! | Priority | Use |
//! | --- | --- |
//! | 0 | `idle` task and background tasks. |
//! | 1 (default) | General and asychronous tasks. |
//! | 2 | Synchronous comms tasks. |
//! | 3 | System critical tasks. |

#![no_main]
#![no_std]

use defmt_rtt as _;
use panic_probe as _;

use bxcan::{filter::Mask32, Interrupts};
use stm32l4xx_hal::{
    can::Can,
    gpio::{Alternate, Output, PushPull, PA11, PA12, PB13},
    pac::CAN1,
    prelude::*,
    watchdog::IndependentWatchdog,
};
use systick_monotonic::{
    fugit::{MillisDurationU32, MillisDurationU64},
    Systick,
};

type Duration = MillisDurationU64;

use solar_car::{com, device};

use elmar_mppt::{Mppt, ID_BASE, ID_INC};

mod horn;
mod lighting;
use horn::Horn;

static DEVICE: device::Device = device::Device::VehicleController;

#[rtic::app(device = stm32l4xx_hal::pac, dispatchers = [SPI1, SPI2, SPI3, QUADSPI])]
mod app {
    use bxcan::Id;

    use super::*;

    #[monotonic(binds = SysTick, default = true)]
    type MonoTimer = Systick<1000>;

    #[shared]
    struct Shared {
        can: bxcan::Can<
            Can<
                CAN1,
                (PA12<Alternate<PushPull, 9>>, PA11<Alternate<PushPull, 9>>),
            >,
        >,
        horn: Horn,
        mppt_a: Mppt,
        mppt_b: Mppt,
    }

    #[local]
    struct Local {
        watchdog: IndependentWatchdog,
        status_led: PB13<Output<PushPull>>,
    }

    #[init]
    fn init(cx: init::Context) -> (Shared, Local, init::Monotonics) {
        defmt::trace!("task: init");

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

            // configure filters
            can.modify_filters().enable_bank(0, Mask32::accept_all());

            // configure interrupts
            can.enable_interrupts(
                Interrupts::TRANSMIT_MAILBOX_EMPTY
                    | Interrupts::FIFO0_MESSAGE_PENDING
                    | Interrupts::FIFO1_MESSAGE_PENDING,
            );
            nb::block!(can.enable_non_blocking()).unwrap();

            // broadcast startup message.
            can.transmit(&com::startup::message(DEVICE)).unwrap();

            can
        };

        // configure watchdog
        let watchdog = {
            let mut wd = IndependentWatchdog::new(cx.device.IWDG);
            wd.stop_on_debug(&cx.device.DBGMCU, true);
            wd.start(MillisDurationU32::millis(100));

            wd
        };

        let mppt_a = Mppt::new(ID_BASE);
        let mppt_b = Mppt::new(ID_BASE + ID_INC);

        // configure horn
        let horn_output = gpiob
            .pb12
            .into_push_pull_output(&mut gpiob.moder, &mut gpiob.otyper)
            .erase();

        let horn = Horn::new(horn_output);

        // start heartbeat task
        heartbeat::spawn_after(Duration::millis(1000)).unwrap();

        // start horn task
        horn::spawn_after(Duration::millis(100)).unwrap();

        // start main loop
        run::spawn().unwrap();

        (
            Shared {
                can,
                horn,
                mppt_a,
                mppt_b,
            },
            Local {
                watchdog,
                status_led,
            },
            init::Monotonics(mono),
        )
    }

    #[task(shared = [can, mppt_a, mppt_b])]
    fn init_mppts(mut cx: init_mppts::Context) {
        defmt::debug!("task: init_mppts");

        const MAX_VOLTAGE: f32 = 60.0;
        const MAX_CURRENT: f32 = 7.0;

        cx.shared.can.lock(|can| {
            cx.shared.mppt_a.lock(|mppt| {
                nb::block!(can.transmit(&mppt.set_mode(elmar_mppt::Mode::On)))
                    .unwrap();
                nb::block!(
                    can.transmit(&mppt.set_maximum_output_voltage(MAX_VOLTAGE))
                )
                .unwrap();
                nb::block!(
                    can.transmit(&mppt.set_maximum_input_current(MAX_CURRENT))
                )
                .unwrap();
            });

            cx.shared.mppt_b.lock(|mppt| {
                nb::block!(can.transmit(&mppt.set_mode(elmar_mppt::Mode::On)))
                    .unwrap();
                nb::block!(
                    can.transmit(&mppt.set_maximum_output_voltage(MAX_VOLTAGE))
                )
                .unwrap();
                nb::block!(
                    can.transmit(&mppt.set_maximum_input_current(MAX_CURRENT))
                )
                .unwrap();
            });
        });
    }

    #[task(priority = 1, local = [watchdog])]
    fn run(cx: run::Context) {
        defmt::trace!("task: run");

        cx.local.watchdog.feed();

        run::spawn_after(Duration::millis(10)).unwrap();
    }

    /// Live, laugh, love
    #[task(priority = 1, shared = [can], local = [status_led])]
    fn heartbeat(mut cx: heartbeat::Context) {
        defmt::trace!("task: heartbeat");

        cx.local.status_led.toggle();

        if cx.local.status_led.is_set_low() {
            cx.shared.can.lock(|can| {
                can.transmit(&com::heartbeat::message(DEVICE)).unwrap();
            });
        }

        heartbeat::spawn_after(Duration::millis(500)).unwrap();
    }

    /// Beep beep!
    #[task(priority = 1, shared = [horn])]
    fn horn(mut cx: horn::Context) {
        defmt::trace!("task: horn");

        cx.shared.horn.lock(|horn| {
            horn.run();
        });

        horn::spawn_after(Duration::millis(100)).unwrap();
    }

    /// Triggers on RX mailbox event.
    #[task(priority = 1, shared = [can], binds = CAN1_RX0)]
    fn can_rx0_pending(_: can_rx0_pending::Context) {
        defmt::trace!("task: can rx0 pending");

        can_receive::spawn().unwrap();
    }

    /// Triggers on RX mailbox event.
    #[task(priority = 1, shared = [can], binds = CAN1_RX1)]
    fn can_rx1_pending(_: can_rx1_pending::Context) {
        defmt::trace!("task: can rx1 pending");

        can_receive::spawn().unwrap();
    }

    #[task(priority = 2, shared = [can, mppt_a, mppt_b])]
    fn can_receive(mut cx: can_receive::Context) {
        defmt::trace!("task: can receive");

        cx.shared.can.lock(|can| loop {
            match can.receive() {
                Ok(frame) => match frame.id() {
                    Id::Standard(_) => {
                        cx.shared.mppt_a.lock(|mppt| {
                            match mppt.receive(&frame) {
                                Ok(_) => {}
                                Err(e) => defmt::error!("{=str}", e),
                            }
                        });

                        cx.shared.mppt_b.lock(|mppt| {
                            match mppt.receive(&frame) {
                                Ok(_) => {}
                                Err(e) => defmt::error!("{=str}", e),
                            }
                        });
                    }
                    _ => {}
                },
                Err(nb::Error::WouldBlock) => break, // done
                Err(nb::Error::Other(_)) => {}       // go to next frame
            }
        });
    }

    #[idle]
    fn idle(_: idle::Context) -> ! {
        defmt::trace!("task: idle");

        loop {
            cortex_m::asm::nop();
        }
    }
}

// same panicking *behavior* as `panic-probe` but doesn't print a panic message
// this prevents the panic message being printed *twice* when `defmt::panic` is invoked
#[defmt::panic_handler]
fn panic() -> ! {
    cortex_m::asm::udf()
}

// Show a millisecond timestamp next to debug output.
// Unit conversion isn't required because ticks = milliseconds for our case.
defmt::timestamp!("time={=u64}ms", {
    app::monotonics::MonoTimer::now().ticks()
});
