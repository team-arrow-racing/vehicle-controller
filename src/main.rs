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

use stm32l4xx_hal::{
    gpio::{
        Edge,
        Input,
        Output,
        PullUp,
        PushPull,
        PullDown,
        PA6,
        PA7,
        PB3,
        PB4,
        PB6,
        PB7,
        PC6,
        PC8
    },
    prelude::*,
    stm32::Interrupt,
    watchdog::IndependentWatchdog,
};

use cortex_m::peripheral::NVIC;

use solar_car::{
    com::lighting::LampsState,
    device
};

mod lighting;

use lighting::Lamps;

// TODO store last time we received a message

const DEVICE: device::Device = device::Device::VehicleController;
const SYSCLK: u32 = 80_000_000;

#[rtic::app(device = stm32l4xx_hal::pac, dispatchers = [SPI1, SPI2, SPI3, QUADSPI])]
mod app {
    use super::*;

    #[monotonic(binds = SysTick, default = true)]
    type MonoTimer = DwtSystick<SYSCLK>;
    pub type Duration = fugit::TimerDuration<u64, SYSCLK>;
    pub type Instant = fugit::TimerInstant<u64, SYSCLK>;

    #[shared]
    struct Shared {
        lamps: Lamps,
        brake_light: PB6<Output<PushPull>>,
    }

    #[local]
    struct Local {
        watchdog: IndependentWatchdog,
        status_led: PB3<Output<PushPull>>,
        brake_pedal: PB7<Input<PullUp>>,
        btn_indicator_left: PA6<Input<PullDown>>, // A5
        btn_indicator_right: PA7<Input<PullDown>>, // A6
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
            .pb3
            .into_push_pull_output(&mut gpiob.moder, &mut gpiob.otyper);

        // configure watchdog
        let watchdog = {
            let mut wd = IndependentWatchdog::new(cx.device.IWDG);
            wd.stop_on_debug(&cx.device.DBGMCU, true);
            wd.start(fugit::MillisDurationU32::millis(100));

            wd
        };

        // configure lighting
        let left_light_output = gpiob
            .pb1 // TODO figure out actual pin
            .into_push_pull_output(&mut gpiob.moder, &mut gpiob.otyper)
            .erase();

        let right_light_output = gpioa
            .pa8 // TODO figure out actual pin
            .into_push_pull_output(&mut gpioa.moder, &mut gpioa.otyper)
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

        let btn_indicator_left = {
            let mut btn = gpioa
                .pa6
                .into_pull_down_input(&mut gpioa.moder, &mut gpioa.pupdr);

            btn.make_interrupt_source(&mut cx.device.SYSCFG, &mut rcc.apb2);
            btn.enable_interrupt(&mut cx.device.EXTI);
            btn.trigger_on_edge(&mut cx.device.EXTI, Edge::RisingFalling);

            unsafe {
                NVIC::unmask(Interrupt::EXTI9_5);
            }

            btn
        };

        let btn_indicator_right = {
            let mut btn = gpioa
                .pa7
                .into_pull_down_input(&mut gpioa.moder, &mut gpioa.pupdr);

            btn.make_interrupt_source(&mut cx.device.SYSCFG, &mut rcc.apb2);
            btn.enable_interrupt(&mut cx.device.EXTI);
            btn.trigger_on_edge(&mut cx.device.EXTI, Edge::RisingFalling);

            unsafe {
                NVIC::unmask(Interrupt::EXTI9_5);
            }

            btn
        };

        let brake_pedal = {
            let mut switch = gpiob
                .pb7
                .into_pull_up_input(&mut gpiob.moder, &mut gpiob.pupdr);

            switch.make_interrupt_source(&mut cx.device.SYSCFG, &mut rcc.apb2);
            switch.enable_interrupt(&mut cx.device.EXTI);
            switch.trigger_on_edge(&mut cx.device.EXTI, Edge::RisingFalling);

            unsafe {
                NVIC::unmask(Interrupt::EXTI9_5);
            }

            switch
        };

        let mut brake_light = gpiob
            .pb6
            .into_push_pull_output(&mut gpiob.moder, &mut gpiob.otyper);
        brake_light.set_high();

        // start main loop
        run::spawn().unwrap();
        heartbeat::spawn().unwrap();

        (
            Shared {
                lamps,
                brake_light
            },
            Local {
                watchdog,
                status_led,
                btn_indicator_left,
                btn_indicator_right,
                brake_pedal,
                
            },
            init::Monotonics(mono),
        )
    }

    #[task(priority = 1, local = [watchdog], shared = [lamps, brake_light])]
    fn run(mut cx: run::Context) {
        defmt::trace!("task: run");

        cx.local.watchdog.feed();

        cx.shared.lamps.lock(|lamps| {
            lamps.run();
        });

        cx.shared.brake_light.lock(|brake_light| {
            defmt::debug!("brake light {}", brake_light.is_set_high());
        });

        run::spawn_after(Duration::millis(10)).unwrap();
    }

    #[task(local = [status_led])]
    fn heartbeat(cx: heartbeat::Context) {
        defmt::trace!("task: heartbeat");
        cx.local.status_led.toggle();
        // repeat every second
        heartbeat::spawn_after(500.millis().into()).unwrap();
    }

    #[task(priority=2, shared=[lamps, brake_light], local=[btn_indicator_left, btn_indicator_right, brake_pedal], binds = EXTI9_5)]
    fn exti9_5_pending(mut cx: exti9_5_pending::Context) {

        let btn_ind_left = cx.local.btn_indicator_left;
        let btn_ind_right = cx.local.btn_indicator_right;
        let brake_pedal = cx.local.brake_pedal;

        if btn_ind_left.check_interrupt() {
            defmt::debug!("left ind {}", btn_ind_left.is_high());
            btn_ind_left.clear_interrupt_pending_bit();
            cx.shared.lamps.lock(|lamps| {
                lamps.set_lamp_state(LampsState::INDICATOR_LEFT, btn_ind_left.is_high());
            });
        }

        if btn_ind_right.check_interrupt() {
            defmt::debug!("right ind {}", btn_ind_right.is_high());
            btn_ind_right.clear_interrupt_pending_bit();
            cx.shared.lamps.lock(|lamps| {
                lamps.set_lamp_state(LampsState::INDICATOR_RIGHT, btn_ind_right.is_high());
            });
        }

        if brake_pedal.check_interrupt() {
            brake_pedal.clear_interrupt_pending_bit();
            cx.shared.brake_light.lock(|brake_light| {
                brake_light.set_state(PinState::from(brake_pedal.is_high()));
            });
        }
    }

    #[task(shared=[lamps], binds = EXTI15_10)]
    fn exti15_10_pending(mut cx: exti15_10_pending::Context) {

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
