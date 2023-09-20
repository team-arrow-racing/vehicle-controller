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

use bxcan::{filter::Mask32, Frame, Id, Interrupts};
use dwt_systick_monotonic::{fugit, DwtSystick};

use stm32l4xx_hal::{
    can::Can,
    gpio::{Analog, Alternate, Output, PushPull, 
        PA4,
        PA9, // LIN TX
        PA10, // LIN RX
        PA11, // CAN RX
        PA12, // CAN TX
        PB4, // STATUS LED
        PB6, // SWITCH 1
        PB7, // SWITCH 2
        PB8, // SWITCH 3
        PB9, // SWITCH 4
        PB13,
        PC1, // ADC IN
    },
    pac::CAN1,
    prelude::*,
    watchdog::IndependentWatchdog,
};

use rand::{Rng, SeedableRng};
use rand::rngs::SmallRng;

use solar_car::{
    com, device, j1939,
    j1939::pgn::{Number, Pgn},
};
mod horn;
mod state;
mod lighting;

use phln::wavesculptor::WaveSculptor;
use phln::driver_controls::DriverControls;

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

// TODO store last time we received a message

const DEVICE: device::Device = device::Device::VehicleController;
const SYSCLK: u32 = 80_000_000;

#[rtic::app(device = stm32l4xx_hal::pac, dispatchers = [SPI1, SPI2, SPI3, QUADSPI])]
mod app {
    use phln::wavesculptor;
    use phln::driver_controls;
    use super::*;

    #[monotonic(binds = SysTick, default = true)]
    type MonoTimer = DwtSystick<SYSCLK>;
    pub type Duration = fugit::TimerDuration<u64, SYSCLK>;
    pub type Instant = fugit::TimerInstant<u64, SYSCLK>;

    type Can1Pins = (PA12<Alternate<PushPull, 9>>, PA11<Alternate<PushPull, 9>>);

    #[shared]
    struct Shared {
        can: bxcan::Can<Can<CAN1,Can1Pins>>,
        ws22: WaveSculptor,
        driver_controls: DriverControls,
    }

    #[local]
    struct Local {
        watchdog: IndependentWatchdog,
        status_led: PB4<Output<PushPull>>,
        demo_light_data: u8,
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
        let mut gpioc = cx.device.GPIOC.split(&mut rcc.ahb2);

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
            .pb4
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
            .set_loopback(true)
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
            nb::block!(can.transmit(&com::startup::message(DEVICE))).unwrap();

            can
        };

        // configure watchdog
        let watchdog = {
            let mut wd = IndependentWatchdog::new(cx.device.IWDG);
            wd.stop_on_debug(&cx.device.DBGMCU, true);
            wd.start(fugit::MillisDurationU32::millis(100));

            wd
        };

        let ws22 = WaveSculptor::new(wavesculptor::ID_BASE);

        let driver_controls = DriverControls::new(driver_controls::ID_BASE_DEFAULT);

        let demo_light_data = 1;

        // start heartbeat task
        heartbeat::spawn_after(Duration::millis(1000)).unwrap();

        // start comms with aic
        feed_watchdog::spawn_after(Duration::millis(500)).unwrap();

        // start main loop
        run::spawn().unwrap();

        sim_ws::spawn().unwrap();

        (
            Shared {
                can,
                ws22,
                driver_controls,
            },
            Local {
                watchdog,
                status_led,
                demo_light_data
            },
            init::Monotonics(mono),
        )
    }

    #[task(priority = 1, local = [watchdog])]
    fn run(mut cx: run::Context) {
        defmt::trace!("task: run");

        cx.local.watchdog.feed();
    
        run::spawn_after(Duration::millis(10)).unwrap();
    }

    // Simulate a wavesculptor message
    #[task(priority = 2, shared=[can])]
    fn sim_ws(mut cx: sim_ws::Context) {
        cx.shared.can.lock(|can| {
            let mut small_rng = SmallRng::seed_from_u64(90u64);
            let vel: f32 = small_rng.gen_range(0..100) as f32;

            nb::block!(
                can.transmit(&com::wavesculptor::speed_message(DEVICE, vel))
            )
            .unwrap();

        });
        sim_ws::spawn_after(Duration::millis(500)).unwrap();
    }

    #[task(priority = 1, shared = [can])]
    fn feed_watchdog(mut cx: feed_watchdog::Context) {
        defmt::trace!("task: feed_watchdog");

        cx.shared.can.lock(|can| {
            nb::block!(can.transmit(&com::array::feed_watchdog(DEVICE))).unwrap();
        });

        feed_watchdog::spawn_after(Duration::millis(500)).unwrap();
    }

    /// Live, laugh, love
    #[task(priority = 1, shared = [can], local = [status_led])]
    fn heartbeat(mut cx: heartbeat::Context) {
        defmt::trace!("task: heartbeat");

        cx.local.status_led.toggle();

        if cx.local.status_led.is_set_low() {
            cx.shared.can.lock(|can| {
                nb::block!(can.transmit(&com::heartbeat::message(DEVICE))).unwrap();
            });
        }

        heartbeat::spawn_after(Duration::millis(500)).unwrap();
    }

    /// Triggers on RX mailbox event.
    #[task(priority = 1, shared = [can], binds = CAN1_RX0)]
    fn can_rx0_pending(_: can_rx0_pending::Context) {
        // defmt::trace!("task: can rx0 pending");

        can_receive::spawn().unwrap();
    }

    /// Triggers on RX mailbox event.
    #[task(priority = 1, shared = [can], binds = CAN1_RX1)]
    fn can_rx1_pending(_: can_rx1_pending::Context) {
        // defmt::trace!("task: can rx1 pending");

        can_receive::spawn().unwrap();
    }

    #[task(priority = 2, shared = [can])]
    fn can_receive(mut cx: can_receive::Context) {
        // defmt::trace!("task: can receive");

        cx.shared.can.lock(|can| loop {
            let frame = match can.receive() {
                Ok(frame) => frame,
                Err(nb::Error::WouldBlock) => break, // done
                Err(nb::Error::Other(_)) => continue, // go to next frame
            };

            let id = match frame.id() {
                Id::Standard(id) => {
                    defmt::debug!("STD FRAME: {:?} {:?}", id.as_raw(), frame);
                    continue;
                }
                Id::Extended(id) => id,
            };

            let id: j1939::ExtendedId = id.into();

            defmt::debug!("EXT FRAME: {:?} {:?}", id.to_bits(), frame);
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
