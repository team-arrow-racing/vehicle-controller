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
        Input,
        Output,
        PullUp,
        PushPull,
        PA10, // LIN RX
        PA11, // CAN RX
        PA12, // CAN TX
        PA4,
        PA9, // LIN TX
        PB13,
        PB4, // STATUS LED
        PB6, // SWITCH 1
        PB7, // SWITCH 2
        PB8, // SWITCH 3
        PB9, // SWITCH 4
        PC1, // ADC IN
    },
    pac::CAN1,
    prelude::*,
    watchdog::IndependentWatchdog,
};

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
const ADC_PEDAL_MAX: f32 = 3300.0; // Max value read by ADC linear potentiometer
const ADC_DEADBAND: u16 = 500; // Cutoff threshold for ADC, values below this will be considered as 0
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
        state: State,
        enable_contactors: bool,
    }

    #[local]
    struct Local {
        watchdog: IndependentWatchdog,
        status_led: PB4<Output<PushPull>>,
        adc: ADC,
        accel_pedal: PC1<Analog>,
        brake_pedal: PA4<Input<PullUp>>,
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

        let state = State::new();

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
        let brake_pedal = gpioa
            .pa4
            .into_pull_up_input(&mut gpioa.moder, &mut gpioa.pupdr);

        // start heartbeat task
        heartbeat::spawn_after(Duration::millis(1000)).unwrap();

        // start comms with aic
        feed_watchdog::spawn_after(Duration::millis(500)).unwrap();
        feed_contactors_heartbeat::spawn_after(Duration::millis(500)).unwrap();

        // init_mppts::spawn().unwrap();

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
                state,
                enable_contactors: true, // TODO for demo, this may be removed in future
            },
            Local {
                watchdog,
                status_led,
                adc,
                accel_pedal,
                brake_pedal,
                driver_controls,
            },
            init::Monotonics(mono),
        )
    }

    #[task(shared = [can, mppt_a, mppt_b])]
    fn init_mppts(mut cx: init_mppts::Context) {
        defmt::trace!("task: init_mppts");

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
                defmt::debug!("task: init mppt a complete");
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
                defmt::debug!("task: init mppt b complete");
            });
        });
    }

    #[task(priority = 1, local = [watchdog], shared = [can, lamps, horn, ws22])]
    fn run(mut cx: run::Context) {
        defmt::trace!("task: run");

        cx.local.watchdog.feed();

        cx.shared.lamps.lock(|lamps| {
            lamps.run();
        });

        cx.shared.horn.lock(|horn| {
            horn.run();
        });

        // send can frames to steering wheel
        cx.shared.can.lock(|can| {
            cx.shared.ws22.lock(|ws22| {
                if let Some(velocity) = ws22.status().vehicle_velocity {
                    nb::block!(can.transmit(&wavesculptor::speed_message(
                        DEVICE, velocity
                    )))
                    .unwrap();
                }

                if let Some(voltage) = ws22.status().bus_voltage {
                    nb::block!(can.transmit(&wavesculptor::battery_message(
                        DEVICE, voltage
                    )))
                    .unwrap();
                }

                if let Some(temp) = ws22.status().motor_temperature {
                    nb::block!(can.transmit(
                        &wavesculptor::temperature_message(DEVICE, temp)
                    ))
                    .unwrap();
                }
            });
        });

        run::spawn_after(Duration::millis(10)).unwrap();
    }

    #[task(priority = 1, shared = [can])]
    fn feed_watchdog(mut cx: feed_watchdog::Context) {
        defmt::trace!("task: feed_watchdog");

        cx.shared.can.lock(|can| {
            nb::block!(can.transmit(&com::array::feed_watchdog(DEVICE)))
                .unwrap();
        });

        feed_watchdog::spawn_after(Duration::millis(500)).unwrap();
    }

    #[task(priority = 1, shared = [can, enable_contactors])]
    fn feed_contactors_heartbeat(mut cx: feed_contactors_heartbeat::Context) {
        defmt::trace!("task: feed_contactors_heartbeat");
        // TODO this should only happen when a switch on the dash is on
        cx.shared.enable_contactors.lock(|ea| {
            if *ea {
                cx.shared.can.lock(|can| {
                    // Feed BMS watchdog
                    let bms_id = StandardId::new(0x505).unwrap();
                    let frame =
                        Frame::new_data(bms_id, [0x70, 0, 0, 0, 0, 0, 0, 0]);
                    nb::block!(can.transmit(&frame)).unwrap();
                });
            }
        });

        feed_contactors_heartbeat::spawn_after(Duration::millis(100)).unwrap();
    }

    /// Live, laugh, love
    #[task(priority = 1, shared = [can], local = [status_led])]
    fn heartbeat(mut cx: heartbeat::Context) {
        defmt::trace!("task: heartbeat");

        cx.local.status_led.toggle();

        if cx.local.status_led.is_set_low() {
            cx.shared.can.lock(|can| {
                nb::block!(can.transmit(&com::heartbeat::message(DEVICE)))
                    .unwrap();
            });
        }

        heartbeat::spawn_after(Duration::millis(500)).unwrap();
    }

    /// RX 0 interrupt pending handler.
    #[task(priority = 2, shared = [can], binds = CAN1_RX0)]
    fn can_rx0_pending(mut cx: can_rx0_pending::Context) {
        defmt::trace!("task: can rx0 pending");

        cx.shared.can.lock(|can| match can.receive() {
            Ok(frame) => can_receive::spawn(frame).unwrap(),
            _ => {}
        })
    }

    /// RX 1 interrupt pending handler.
    #[task(priority = 2, shared = [can], binds = CAN1_RX1)]
    fn can_rx1_pending(mut cx: can_rx1_pending::Context) {
        defmt::trace!("task: can rx1 pending");

        cx.shared.can.lock(|can| match can.receive() {
            Ok(frame) => can_receive::spawn(frame).unwrap(),
            _ => {}
        })
    }

    #[task(priority = 1, shared = [lamps, horn, mppt_a, mppt_b, ws22, cruise, mode, enable_contactors], capacity=100)]
    fn can_receive(mut cx: can_receive::Context, frame: Frame) {
        // defmt::trace!("task: can receive");

        let id = match frame.id() {
            Id::Standard(id) => {
                defmt::debug!("STD FRAME: {:#06x} {:?}", id.as_raw(), frame);

                cx.shared.ws22.lock(|ws22| {
                    if id.as_raw() >= phln::wavesculptor::ID_BASE {
                        let _res = ws22.receive(frame);
                    } else {
                        // must be MPPT frame, handle accordingly
                        cx.shared.mppt_a.lock(|mppt_a| {
                            cx.shared.mppt_b.lock(|mppt_b| {
                                handle_mppt_frame(&frame, mppt_a, mppt_b)
                            });
                        });
                    }
                });
                return;
            }
            Id::Extended(id) => id,
        };

        let id: j1939::ExtendedId = id.into();

        defmt::debug!("EXT FRAME: {:#06x} {:?}", id.to_bits(), frame);

        let id: j1939::ExtendedId = id.into();

        match id.pgn {
            Pgn::Destination(pgn) => match pgn {
                com::array::PGN_ENABLE_CONTACTORS => {
                    cx.shared.enable_contactors.lock(|ea| {
                        if let Some(data) = frame.data() {
                            *ea = data[0] != 0;
                        }
                    });
                },
                com::lighting::PGN_LIGHTING_STATE => cx
                    .shared
                    .lamps
                    .lock(|lamps| handle_lighting_frame(lamps, &frame)),
                com::horn::PGN_HORN_MESSAGE => {
                    cx.shared.horn.lock(|horn| {
                        horn.set_on();
                    });
                },
                wavesculptor::PGN_SET_DRIVE_CONTROL_TYPE => {
                    cx.shared.cruise.lock(|cruise| {
                        if let Some(data) = frame.data() {
                            *cruise = data[0] != 0;
                        }
                    });
                },
                wavesculptor::PGN_SET_DRIVER_MODE => {
                    cx.shared.mode.lock(|mode| {
                        if let Some(data) = frame.data() {
                            let new_mode = DriverModes::from(data[0]);
                            if new_mode == DriverModes::Reverse {
                                // Do not go into reverse if car is moving forward
                                cx.shared.ws22.lock(|ws22| {
                                    if let Some(rpms) =
                                        ws22.status().motor_velocity
                                    {
                                        if (rpms as i32) > 2 {
                                            return;
                                        }
                                    }
                                });
                            }
                            *mode = new_mode;
                        }
                    });
                },
                _ => {
                    defmt::debug!("whut happun")
                }
            },
            _ => {} // ignore broadcast messages
        }
    }

    fn handle_mppt_frame(frame: &Frame, mppt_a: &mut Mppt, mppt_b: &mut Mppt) {
        match mppt_a.receive(&frame) {
            Ok(_) => {}
            Err(e) => defmt::error!("{=str}", e),
        };

        match mppt_b.receive(&frame) {
            Ok(_) => {}
            Err(e) => defmt::error!("{=str}", e),
        };
    }

    fn handle_lighting_frame(lamps: &mut Lamps, frame: &Frame) {
        defmt::debug!("received lighting frame data {:?}", frame.data());
        match frame.data() {
            Some(bytes) => {
                let lamp = bytes[0]; // TODO confirm data is just in first index
                let state = bytes[1];
                match LampsState::from_bits(lamp) {
                    Some(data) => {
                        lamps.set_lamp_state(data, state != 0);
                    }
                    _ => defmt::debug!("Got invalid lighting data"),
                }
            }
            _ => {}
        }
    }

    #[task(shared=[can, cruise, mode, lamps, ws22], local = [adc, accel_pedal, brake_pedal, driver_controls])]
    fn read_adc_pin(mut cx: read_adc_pin::Context) {
        let accel_throttle = cx.local.adc.read(cx.local.accel_pedal).unwrap();
        let is_braking = cx.local.brake_pedal.is_high(); // assuming braking is a simple toggle
                                                         // to bring car to a halt asap

        // Turn on brake lights if needed
        cx.shared.lamps.lock(|lamps| {
            lamps.set_lamp_state(LampsState::STOP, is_braking);
        });

        let dc = cx.local.driver_controls;

        cx.shared.cruise.lock(|cruise| {
            cx.shared.mode.lock(|mode| {
                cx.shared.ws22.lock(|ws22| {
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
                                    // (accel_throttle - ADC_DEADBAND) as f32 / (ADC_PEDAL_MAX - (ADC_DEADBAND as f32))
                                    (accel_throttle as f32
                                        / (ADC_PEDAL_MAX - ADC_DEADBAND as f32))
                                        .max(1.0)
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

                    // defmt::debug!(
                    //     "{:?} {:?} {}",
                    //     accel_throttle,
                    //     percentage,
                    //     is_braking
                    // );
                    // TODO if in cruise, velocity should be fixed to desired speed
                    // probably just retrieve the current speed from ws
                    let frame = dc.motor_drive(desired_rpms, percentage);

                    cx.shared.can.lock(|can| {
                        nb::block!(can.transmit(&frame)).unwrap();
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
