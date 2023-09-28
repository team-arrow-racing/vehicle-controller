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

use dwt_systick_monotonic::{fugit, DwtSystick};
use timed_debouncer::Debouncer;
use stm32l4xx_hal::{
    adc::{DmaMode, SampleTime, Sequence, ADC},
    can::Can,
    device::CAN1,
    delay::DelayCM,
    flash::FlashExt,
    gpio::{Analog, Alternate, Output, PushPull, 
        PA0,
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
    prelude::*,
    watchdog::IndependentWatchdog,
};

use solar_car::{
    com, device, j1939,
    j1939::pgn::{Number, Pgn},
};

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
    use bxcan::{filter::Mask32, Frame, Id, Interrupts};

    use super::*;

    #[monotonic(binds = SysTick, default = true)]
    type MonoTimer = DwtSystick<SYSCLK>;
    pub type Duration = fugit::TimerDuration<u64, SYSCLK>;
    pub type Instant = fugit::TimerInstant<u64, SYSCLK>;

    type Can1Pins = (PA12<Alternate<PushPull, 9>>, PA11<Alternate<PushPull, 9>>);

    #[shared]
    struct Shared {
        can: bxcan::Can<Can<CAN1,Can1Pins>>,
    }

    #[local]
    struct Local {
        watchdog: IndependentWatchdog,
        status_led: PB4<Output<PushPull>>,
        adc: ADC,
        accel_pedal: PA0<Analog>,
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
        let clocks = rcc
            .cfgr
            .sysclk(80.MHz())
            .pclk1(80.MHz())
            .pclk2(80.MHz())
            .freeze(&mut flash.acr, &mut pwr);


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
            .set_bit_timing(0x001c_0009); // 500kbit/s

            let mut can = can.enable();

            // configure filters
            can.modify_filters().enable_bank(0, Mask32::accept_all());

            // configure interrupts
            can.enable_interrupts(
                Interrupts::TRANSMIT_MAILBOX_EMPTY
                    | Interrupts::FIFO0_MESSAGE_PENDING
                    // | Interrupts::FIFO1_MESSAGE_PENDING,
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

        let mut delay = DelayCM::new(clocks);
        let adc = ADC::new(
            cx.device.ADC1,
            cx.device.ADC_COMMON,
            &mut rcc.ahb2,
            &mut rcc.ccipr,
            &mut delay,
        );
        let accel_pedal =
            gpioa.pa0.into_analog(&mut gpioa.moder, &mut gpioa.pupdr);

        // start heartbeat task
        // start comms with aic
        // feed_watchdog::spawn_after(Duration::millis(500)).unwrap();

        // start main loop
        read_adc_pin::spawn().unwrap();
        run::spawn().unwrap();
        heartbeat::spawn_after(Duration::millis(500)).unwrap();

        // sim_ws::spawn().unwrap();

        (
            Shared {
                can,
            },
            Local {
                watchdog,
                status_led,
                adc,
                accel_pedal
            },
            init::Monotonics(mono),
        )
    }

    #[task(priority = 1, local = [watchdog])]
    fn run(cx: run::Context) {
        defmt::trace!("task: run");

        cx.local.watchdog.feed();
    
        run::spawn_after(Duration::millis(10)).unwrap();
    }

    /// Live, laugh, love
    #[task(shared = [can], local=[status_led])]
    fn heartbeat(mut cx: heartbeat::Context) {
        defmt::trace!("task: heartbeat");

        cx.local.status_led.toggle();

        if cx.local.status_led.is_set_low() {
            cx.shared.can.lock(|can| {
                // let _ = can.transmit(&com::heartbeat::message(DEVICE)).unwrap();
            });
        }

        heartbeat::spawn_after(Duration::millis(500)).unwrap();
    }

    /// Triggers on RX mailbox event.
    #[task(priority = 2, shared = [can], binds = CAN1_RX0)]
    fn can_rx0_pending(mut cx: can_rx0_pending::Context) {
        defmt::trace!("task: can rx0 pending");

        cx.shared.can.lock(|can| match can.receive() {
            Ok(frame) => can_receive::spawn(frame).unwrap(),
            _ => {}
        })
    }

    /// Triggers on RX mailbox event.
    #[task(priority = 2, shared = [can], binds = CAN1_RX1)]
    fn can_rx1_pending(mut cx: can_rx1_pending::Context) {
        defmt::trace!("task: can rx1 pending");

        cx.shared.can.lock(|can| match can.receive() {
            Ok(frame) => can_receive::spawn(frame).unwrap(),
            _ => {}
        })
    }

    #[task(priority = 1, capacity=100)]
    fn can_receive(mut cx: can_receive::Context, frame: Frame) {
        defmt::trace!("task: can receive");
        match frame.id() {
            Id::Standard(id) => {
                defmt::debug!("STD FRAME: {:#06x} {:?}", id.as_raw(), frame);
            }
            Id::Extended(id) => {
                defmt::debug!("EXT FRAME: {:#06x} {:?}", id.as_raw(), frame);
            } // not used
        }
    }

    #[task(local = [adc, accel_pedal])]
    fn read_adc_pin(mut cx: read_adc_pin::Context) {
        let mut debouncer = Debouncer::new();
        let accel_throttle = cx.local.adc.read(cx.local.accel_pedal).unwrap();
        let d_val = debouncer.update(accel_throttle, monotonics::MonoTimer::now().ticks(), 800);

        defmt::debug!("{} {}", d_val, (d_val / 1900) as i16);

        read_adc_pin::spawn_after(Duration::millis(100)).unwrap();
    }

    // #[idle]
    // fn idle(_: idle::Context) -> ! {
    //     defmt::trace!("task: idle");

    //     loop {
    //         cortex_m::asm::nop();
    //     }
    // }
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
