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
    adc::{DmaMode, SampleTime, Sequence, ADC},
    delay::{Delay, DelayCM},
    dma::{dma1, RxDma, Transfer, W},
    prelude::*,
    watchdog::IndependentWatchdog,
};

use elmar_mppt::{Mppt, ID_BASE, ID_INC};
use solar_car::{
    com, device, j1939,
    j1939::pgn::{Number, Pgn},
};
mod horn;
mod state;
mod lighting;

use state::State;
use horn::Horn;
use lighting::Lamps;
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
const SEQUENCE_LEN: usize = 3;

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
        horn: Horn,
        lamps: Lamps,
        mppt_a: Mppt,
        mppt_b: Mppt,
        ws22: WaveSculptor,
        cruise: com::wavesculptor::ControlTypes,
        mode: com::wavesculptor::DriverModes,
        state: State,
    }

    #[local]
    struct Local {
        watchdog: IndependentWatchdog,
        status_led: PB4<Output<PushPull>>,
        adc: ADC,
        adc_pin: PC1<Analog>,
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

        let mppt_a = Mppt::new(ID_BASE);
        let mppt_b = Mppt::new(ID_BASE + ID_INC);

        let ws22 = WaveSculptor::new(wavesculptor::ID_BASE);

        let driver_controls = DriverControls::new(driver_controls::ID_BASE_DEFAULT);

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
        let adc_pin = gpioc.pc1.into_analog(&mut gpioc.moder, &mut gpioc.pupdr);

        // start heartbeat task
        heartbeat::spawn_after(Duration::millis(1000)).unwrap();

        // start comms with aic
        feed_watchdog::spawn_after(Duration::millis(500)).unwrap();
        feed_bms::spawn().unwrap();
        // send_rpm::spawn_after(Duration::millis(5000)).unwrap();
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
                cruise: com::wavesculptor::ControlTypes::Torque,
                mode: com::wavesculptor::DriverModes::Neutral,
                state,
            },
            Local {
                watchdog,
                status_led,
                adc,
                adc_pin,
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
        // defmt::trace!("task: run");

        cx.local.watchdog.feed();

        // cx.shared.lamps.lock(|lamps| {
        //     lamps.run();
        // });

        // cx.shared.horn.lock(|horn| {
        //     horn.run();
        // });

        // send can frames to steering wheel
        cx.shared.can.lock(|can| {
            cx.shared.ws22.lock(|ws22| {
                if let Some(velocity) = ws22.status().vehicle_velocity {
                    nb::block!(can.transmit(&com::wavesculptor::speed_message(DEVICE, velocity))).unwrap();
                }

                if let Some(voltage) = ws22.status().bus_voltage {
                    nb::block!(can.transmit(&com::wavesculptor::battery_message(DEVICE, voltage))).unwrap();
                }

                if let Some(temp) = ws22.status().motor_temperature {
                    nb::block!(can.transmit(&com::wavesculptor::temperature_message(DEVICE, temp))).unwrap();
                }
            });
        });
    
        run::spawn_after(Duration::millis(10)).unwrap();
    }

    #[task(priority = 1, shared = [can])]
    fn feed_watchdog(mut cx: feed_watchdog::Context) {
        defmt::trace!("task: feed_watchdog");

        cx.shared.can.lock(|can| {
            nb::block!(can.transmit(&com::array::feed_watchdog(DEVICE))).unwrap();
        });
        
        feed_watchdog::spawn_after(Duration::millis(500)).unwrap();
    }

    #[task(priority = 1, shared = [can])]
    fn feed_bms(mut cx: feed_bms::Context) {
        // TODO this should only happen when a switch on the dash is on
        cx.shared.can.lock(|can| {
            // Feed BMS watchdog
            let bms_id = StandardId::new(0x505).unwrap();
            let frame = Frame::new_data(bms_id, [0x70, 0, 0, 0, 0, 0, 0, 0]);
            nb::block!(can.transmit(&frame)).unwrap();
        });

        feed_bms::spawn_after(Duration::millis(100)).unwrap();
    }

    // #[task(priority = 1, shared = [can], local=[driver_controls])]
    // fn send_rpm(mut cx: send_rpm::Context) {
    //     // TODO this should only happen when a switch on the dash is on
    //     let dc = cx.local.driver_controls;
    //     cx.shared.can.lock(|can| {
    //         // Feed BMS watchdog
    //         let frame = dc.motor_drive(100f32, 0.01f32);
    //         nb::block!(can.transmit(&frame)).unwrap();
    //     });

    //     send_rpm::spawn_after(Duration::millis(200)).unwrap();
    // }

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

    #[task(priority = 2, shared = [lamps, mppt_a, mppt_b, ws22, cruise, mode], capacity=100)]
    fn can_receive(mut cx: can_receive::Context, frame: Frame) {
        // defmt::trace!("task: can receive");

        let id = match frame.id() {
            Id::Standard(id) => {
                defmt::debug!("STD FRAME: {:#06x} {:?}", id.as_raw(), frame);
                
                cx.shared.ws22.lock(|ws22| {
                    if id.as_raw() >= wavesculptor::ID_BASE {
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
                PGN_MESSAGE_TEST => defmt::debug!("aur naur"),
                com::lighting::PGN_LIGHTING_STATE => cx
                    .shared
                    .lamps
                    .lock(|lamps| handle_lighting_frame(lamps, &frame)),
                com::horn::PGN_HORN_MESSAGE => defmt::debug!("honk"),
                com::wavesculptor::PGN_SET_DRIVE_CONTROL_TYPE => {
                    cx.shared.cruise.lock(|cruise| {
                        if let Some(data) = frame.data() {
                            *cruise = com::wavesculptor::ControlTypes::from(data[0]);
                        }
                    });
                },
                com::wavesculptor::PGN_SET_DRIVER_MODE => {
                    cx.shared.mode.lock(|mode| {
                        if let Some(data) = frame.data() {
                            *mode = com::wavesculptor::DriverModes::from(data[0]);
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
                let bytes_int = bytes[0]; // TODO confirm data is just in first index
                match com::lighting::LampsState::from_bits(bytes_int) {
                    Some(data) => {
                        lamps.set_state(data);
                        lamps.run();
                    }
                    _ => defmt::debug!("Got invalid lighting data"),
                }
            }
            _ => {}
        }
    }

    #[task(shared=[can, cruise, mode, ws22], local = [adc, adc_pin, driver_controls])]
    fn read_adc_pin(mut cx: read_adc_pin::Context) {
        let adc_value = cx.local.adc.read(cx.local.adc_pin).unwrap();
        let dc = cx.local.driver_controls;

        cx.shared.cruise.lock(|cruise| {
            cx.shared.mode.lock(|mode| {
                cx.shared.ws22.lock(|ws22| {
                    let percentage: f32 = {
                        if *cruise == com::wavesculptor::ControlTypes::Cruise {
                            1.0
                        } else {
                            if adc_value < 200 {
                                0.0
                            } else {
                                (adc_value - 200) as f32 / 3700.0
                            }
                        }
                    };

                    let current_rpms = match ws22.status().motor_velocity {
                        Some(rpms) => rpms,
                        None => 630f32
                    };

                    let desired_rpms = {
                        
                        if *mode == com::wavesculptor::DriverModes::Reverse {
                            -630f32
                        } else {
                            if *cruise == com::wavesculptor::ControlTypes::Cruise {current_rpms} else {630f32}
                        }
                    };
                    
                    defmt::debug!("{:?} {:?}", adc_value, percentage);
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
