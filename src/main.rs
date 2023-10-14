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

use bxcan::{filter::Mask32, Frame, Id, Interrupts, StandardId};
use dwt_systick_monotonic::{fugit, DwtSystick};

use stm32l4xx_hal::{
    adc::{DmaMode, SampleTime, Sequence, ADC},
    can::Can,
    delay::{Delay, DelayCM},
    dma::{dma1, RxDma, Transfer, W},
    gpio::{
        Alternate,
        Analog,
        Edge,
        Input,
        Output,
        PullUp,
        PushPull,
        PA10, // LIN RX
        PA11, // CAN RX
        PA12, // CAN TX
        PA4, // ADC1_IN9
        PA5, // ADC1_IN10 BRAKE PIN
        PA9, // LIN TX
        PB13,
        PB4, // STATUS LED
        PB6, // SWITCH 1
        PB7, // SWITCH 2
        PB8, // SWITCH 3
        PB9, // SWITCH 4
        PC1, // ADC IN2 PEDAL PIN
    },
    pac::CAN1,
    prelude::*,
    stm32::Interrupt,
    watchdog::IndependentWatchdog,
};

use cortex_m::peripheral::NVIC;

use elmar_mppt::{Mppt, ID_BASE, ID_INC};
use solar_car::{
    com::{
        self,
        lighting::LampsState,
        wavesculptor::{self, DriverModes},
    },
    device, j1939,
    j1939::pgn::{Number, Pgn},
};

mod horn;
mod lighting;
mod state;

use horn::Horn;
use lighting::Lamps;
use phln::driver_controls::DriverControls;
use phln::wavesculptor::WaveSculptor;
use state::State;

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

// TODO store last time we received a message

const DEVICE: device::Device = device::Device::VehicleController;
const SYSCLK: u32 = 80_000_000;
const ADC_PEDAL_MAX: f32 = 2500.0; // Max value read by ADC linear potentiometer
const ADC_DEADBAND: u16 = 700; // Cutoff threshold for ADC, values below this will be considered as 0
const MAX_FORWARD_RPMS: f32 = 4000.0;
const MAX_REVERSE_RPMS: f32 = -1500.0;
const BRAKING_PERCENTAGE: f32 = 0.1;

#[rtic::app(device = stm32l4xx_hal::pac, dispatchers = [SPI1, SPI2, SPI3, QUADSPI])]
mod app {
    use super::*;

    #[monotonic(binds = SysTick, default = true)]
    type MonoTimer = DwtSystick<SYSCLK>;
    pub type Duration = fugit::TimerDuration<u64, SYSCLK>;
    pub type Instant = fugit::TimerInstant<u64, SYSCLK>;

    type Can1Pins =
        (PA12<Alternate<PushPull, 9>>, PA11<Alternate<PushPull, 9>>);

    #[shared]
    struct Shared {
        can: bxcan::Can<Can<CAN1, Can1Pins>>,
        horn: Horn,
        lamps: Lamps,
        mppt_a: Mppt,
        mppt_b: Mppt,
        ws22: WaveSculptor,
        cruise: bool,
        mode: DriverModes,
        brake_pedal: PA5<Input<PullUp>>,
        state: State,
    }

    #[local]
    struct Local {
        watchdog: IndependentWatchdog,
        status_led: PB4<Output<PushPull>>,
        adc: ADC,
        accel_pedal: PC1<Analog>,
        driver_controls: DriverControls,
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
        let mut can = {
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
                    | Interrupts::FIFO0_MESSAGE_PENDING, // | Interrupts::FIFO1_MESSAGE_PENDING,
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

        let mppt_a = Mppt::new(ID_BASE);
        let mppt_b = Mppt::new(ID_BASE + ID_INC);

        let ws22 = WaveSculptor::new(phln::wavesculptor::ID_BASE);

        let driver_controls =
            DriverControls::new(phln::driver_controls::ID_BASE_DEFAULT);

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

        let lamps = Lamps::new(
            left_light_output,
            right_light_output,
            day_light_output,
            brake_light_output,
        );

        // Configure ADC
        let mut delay = DelayCM::new(clocks);
        let adc = ADC::new(
            cx.device.ADC1,
            cx.device.ADC_COMMON,
            &mut rcc.ahb2,
            &mut rcc.ccipr,
            &mut delay,
        );
        let accel_pedal =
            gpioc.pc1.into_analog(&mut gpioc.moder, &mut gpioc.pupdr);

        let brake_pedal = {
            
            let mut switch = gpioa
                .pa5
                .into_pull_up_input(&mut gpioa.moder, &mut gpioa.pupdr);
            
            switch.make_interrupt_source(&mut cx.device.SYSCFG, &mut rcc.apb2);
            switch.enable_interrupt(&mut cx.device.EXTI);
            switch.trigger_on_edge(&mut cx.device.EXTI, Edge::RisingFalling);

            unsafe {
                NVIC::unmask(Interrupt::EXTI9_5);
            }

            switch
        };

        read_adc_pin::spawn_after(Duration::millis(500)).unwrap();

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
                cruise: false,
                mode: DriverModes::Neutral,
                brake_pedal,
                state: State::Idle,
            },
            Local {
                watchdog,
                status_led,
                adc,
                accel_pedal,
                driver_controls,
            },
            init::Monotonics(mono),
        )
    }

    #[task(priority = 1, local = [watchdog], shared = [can, lamps, horn, ws22, state])]
    fn run(mut cx: run::Context) {
        defmt::trace!("task: run");

        cx.local.watchdog.feed();

        cx.shared.lamps.lock(|lamps| {
            lamps.run();
        });

        run::spawn_after(Duration::millis(10)).unwrap();
    }

    #[task(shared=[lamps, brake_pedal], binds = EXTI9_5)]
    fn exti9_5_pending(mut cx: exti9_5_pending::Context) {
        cx.shared.brake_pedal.lock(|brake_pedal| {
            if brake_pedal.check_interrupt() {
                brake_pedal.clear_interrupt_pending_bit();
                // Turn on brake lights if needed
                let is_braking = brake_pedal.is_low();
                cx.shared.lamps.lock(|lamps| {
                    lamps.set_lamp_state(LampsState::STOP, is_braking);
                });
            }
        });
    }

    #[task(shared=[can, cruise, mode, lamps, brake_pedal, ws22], local = [adc, accel_pedal, driver_controls])]
    fn read_adc_pin(mut cx: read_adc_pin::Context) {
        let accel_throttle = cx.local.adc.read(cx.local.accel_pedal).unwrap();

        let dc = cx.local.driver_controls;

        cx.shared.cruise.lock(|cruise| {
            cx.shared.mode.lock(|mode| {
                cx.shared.ws22.lock(|ws22| {
                    cx.shared.brake_pedal.lock(|brake_pedal| {
                        let is_braking = brake_pedal.is_low(); // assuming braking is a simple toggle
                                                                            // to bring car to a halt asap
                        
                        let percentage: f32 = {
                            if is_braking {
                                // TODO this OR mode is in Neutral - add once testing is done
                                BRAKING_PERCENTAGE // TODO this might need to be 0 - test and confirm
                            } else {
                                if *cruise {
                                    1.0
                                } else {
                                    if accel_throttle < ADC_DEADBAND {
                                        0.0
                                    } else {
                                        ((accel_throttle - ADC_DEADBAND) as f32 / (ADC_PEDAL_MAX - (ADC_DEADBAND as f32))).min(1.0)
                                    }
                                }
                            }
                        };

                        let current_rpms = match ws22.status().motor_velocity {
                            Some(rpms) => rpms,
                            None => MAX_FORWARD_RPMS,
                        };

                        let desired_rpms = {
                            if is_braking {
                                // TODO this OR mode is in Neutral - add once testing is done
                                0.0
                            } else {
                                if *mode == DriverModes::Reverse {
                                    MAX_REVERSE_RPMS
                                } else {
                                    if *cruise {
                                        current_rpms
                                    } else {
                                        MAX_FORWARD_RPMS
                                    }
                                }
                            }
                        };

                        defmt::debug!(
                            "{:?} {:?} {}",
                            accel_throttle,
                            percentage,
                            is_braking
                        );
                        // TODO if in cruise, velocity should be fixed to desired speed
                        // probably just retrieve the current speed from ws
                        let frame = dc.motor_drive(desired_rpms, percentage);

                        cx.shared.can.lock(|can| {
                            nb::block!(can.transmit(&frame)).unwrap();
                        });
                    });
                });
            });
        });

        read_adc_pin::spawn_after(Duration::millis(100)).unwrap();
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
