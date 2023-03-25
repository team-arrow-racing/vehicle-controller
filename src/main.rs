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

use bxcan::{filter::Mask32, Id, Interrupts};
use dwt_systick_monotonic::{fugit, DwtSystick};

use stm32l4xx_hal::{
    can::Can,
    gpio::{Alternate, Output, PushPull, PA11, PA12, PB13},
    pac::CAN1,
    prelude::*,
    watchdog::IndependentWatchdog,
};

use elmar_mppt::{Mppt, ID_BASE, ID_INC};
use solar_car::{
    com, device, j1939,
    j1939::pgn::{Number, Pgn}
};
mod horn;
mod state;
use state::State;
mod lighting;
use horn::Horn;
use lighting::Lamps;
use prohelion::wavesculptor::WaveSculptor;

/// Message format identifier
#[repr(u8)]
pub enum VCUMessageFormat {
    // broadcast messages

    /// Startup status message
    Startup = 0xF0,
    /// Heartbeat status message
    Heartbeat = 0xF1,

    // addressable messages

    /// Generic reset command message
    Reset = 0x00,
    /// Generic enable command message
    Enable = 0x01,
    /// Generic disable command message
    Disable = 0x02,
}

pub const PGN_MESSAGE_TEST: Number = Number {
    specific: device::Device::VehicleController as u8,
    format: VCUMessageFormat::Enable as u8,
    data_page: false,
    extended_data_page: false,
};

pub const PGN_LIGHTING_STATE: Number = Number {
    specific: device::Device::VehicleController as u8,
    format: VCUMessageFormat::Enable as u8,
    data_page: false,
    extended_data_page: false,
};

// TODO store last time we received a message

const DEVICE: device::Device = device::Device::VehicleController;
const SYSCLK: u32 = 80_000_000;

#[rtic::app(device = stm32l4xx_hal::pac, dispatchers = [SPI1, SPI2, SPI3, QUADSPI])]
mod app {
    use prohelion::wavesculptor;

    use super::*;

    #[monotonic(binds = SysTick, default = true)]
    type MonoTimer = DwtSystick<SYSCLK>;
    pub type Duration = fugit::TimerDuration<u64, SYSCLK>;
    pub type Instant = fugit::TimerInstant<u64, SYSCLK>;

    #[shared]
    struct Shared {
        can: bxcan::Can<
            Can<
                CAN1,
                (PA12<Alternate<PushPull, 9>>, PA11<Alternate<PushPull, 9>>),
            >,
        >,
        horn: Horn,
        lamps: Lamps,
        mppt_a: Mppt,
        mppt_b: Mppt,
        ws22: WaveSculptor,
        state: State,
    }

    #[local]
    struct Local {
        watchdog: IndependentWatchdog,
        status_led: PB13<Output<PushPull>>
    }

    #[init]
    fn init(mut cx: init::Context) -> (Shared, Local, init::Monotonics) {
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
        let mono = DwtSystick::new(
            &mut cx.core.DCB,
            cx.core.DWT,
            cx.core.SYST,
            clocks.sysclk().to_Hz(),
        );

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
            wd.start(fugit::MillisDurationU32::millis(100));

            wd
        };

        let mppt_a = Mppt::new(ID_BASE);
        let mppt_b = Mppt::new(ID_BASE + ID_INC);

        let ws22 = WaveSculptor::new(wavesculptor::ID_BASE);

        // configure horn
        let horn_output = gpiob
            .pb12
            .into_push_pull_output(&mut gpiob.moder, &mut gpiob.otyper)
            .erase();

        let horn = Horn::new(horn_output);

        // configure lighting
        let left_light_output = gpiob
            .pb14 // TODO figure out actual pin
            .into_push_pull_output(&mut gpiob.moder, &mut gpiob.otyper)
            .erase();

        let right_light_output = gpiob
            .pb15 // TODO figure out actual pin
            .into_push_pull_output(&mut gpiob.moder, &mut gpiob.otyper)
            .erase();

        let day_light_output = gpiob
            .pb10 // TODO figure out actual pin
            .into_push_pull_output(&mut gpiob.moder, &mut gpiob.otyper)
            .erase();

        let brake_light_output = gpiob
            .pb11
            .into_push_pull_output(&mut gpiob.moder, &mut gpiob.otyper)
            .erase();
        
        let lamps = Lamps::new(left_light_output, right_light_output, day_light_output, brake_light_output);

        let state = State::new();

        // start heartbeat task
        heartbeat::spawn_after(Duration::millis(1000)).unwrap();

        // start comms with aic
        aic_comms::spawn_after(Duration::millis(500)).unwrap();

        // start main loop
        run::spawn().unwrap();

        (
            Shared {
                can,
                horn,
                lamps,
                mppt_a,
                mppt_b,
                ws22,
                state,
            },
            Local {
                watchdog,
                status_led
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

    #[task(priority = 1, local = [watchdog], shared = [lamps, horn])]
    fn run(mut cx: run::Context) {
        defmt::trace!("task: run");

        cx.local.watchdog.feed();

        cx.shared.lamps.lock(|lamps| {
            lamps.run();
        });

        cx.shared.horn.lock(|horn| {
            horn.run();
        });

        run::spawn_after(Duration::millis(10)).unwrap();
    }

    #[task(priority = 1, shared = [can])]
    fn aic_comms(mut cx: aic_comms::Context) {
        defmt::trace!("task: aic_comms");

        cx.shared.can.lock(|can| {
            let _ = can.transmit(&com::array::vcu_comms(DEVICE));
        });
        
        aic_comms::spawn_after(Duration::millis(500)).unwrap();
    }

    /// Live, laugh, love
    #[task(priority = 1, shared = [can], local = [status_led])]
    fn heartbeat(mut cx: heartbeat::Context) {
        defmt::trace!("task: heartbeat");

        cx.local.status_led.toggle();

        if cx.local.status_led.is_set_low() {
            cx.shared.can.lock(|can| {
                let _ = can.transmit(&com::heartbeat::message(DEVICE));
            });
        }

        heartbeat::spawn_after(Duration::millis(500)).unwrap();
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

    #[task(priority = 2, shared = [can, lamps, mppt_a, mppt_b])]
    fn can_receive(mut cx: can_receive::Context) {
        defmt::debug!("task: can receive");

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
                    },
                    Id::Extended(id) => {
                        // convert to a J1939 id
                        let id: j1939::ExtendedId = id.into();

                        // is this message for us?
                        match id.pgn {
                            Pgn::Destination(pgn) => match pgn {
                                PGN_MESSAGE_TEST => {
                                    defmt::trace!("aur naur");
                                },
                                PGN_LIGHTING_STATE => {
                                    defmt::trace!("oooh lights");
                                    // cx.shared.lamps.lock(|lamps| lamps.set_left_indicator(true));
                                },
                                _ => {}
                            },
                            _ => {} // ignore broadcast messages
                        }
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
    app::monotonics::MonoTimer::now()
        .duration_since_epoch()
        .to_millis()
});
