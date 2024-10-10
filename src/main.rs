#![no_main]
#![no_std]
#![feature(type_alias_impl_trait)]
#![allow(clippy::transmute_ptr_to_ptr)]

mod canbus;
mod init;

// import tasks
use canbus::*;
use init::*;

// global logger
use defmt_rtt as _;
use panic_probe as _;
use stm32h7xx_hal as hal;

use fdcan::{frame::RxFrameInfo, FdCanControl, Fifo0, Fifo1, NormalOperationMode, Rx, Tx};
use hal::gpio::ErasedPin;
use hal::{can::Can, gpio::Output, independent_watchdog::IndependentWatchdog, pac::FDCAN1};
use rtic_monotonics::{
    systick::{ExtU64, Systick},
    Monotonic,
};
use solar_car::com::lighting::LampsState;

#[rtic::app(device = stm32h7xx_hal::pac, dispatchers = [UART4, SPI1])]
mod app {
    use stm32h7xx_hal::gpio::PinState;

    use super::*;

    type FdCanMode = NormalOperationMode;

    pub struct Lights {
        pub brake_lights: u8,
        pub right_indicator: u8,
        pub left_indicator: u8
    }

    #[shared]
    pub struct Shared {
        pub fdcan1_ctrl: FdCanControl<Can<FDCAN1>, FdCanMode>,
        pub fdcan1_tx: Tx<Can<FDCAN1>, FdCanMode>,
        pub fdcan1_rx0: Rx<Can<FDCAN1>, FdCanMode, Fifo0>,
        pub fdcan1_rx1: Rx<Can<FDCAN1>, FdCanMode, Fifo1>,
        pub light_states: Lights
    }

    #[local]
    pub struct Local {
        pub watchdog: IndependentWatchdog,
        pub led_ok: ErasedPin<Output>,
        pub led_warn: ErasedPin<Output>,
        pub led_error: ErasedPin<Output>,
        pub brake_light_output: ErasedPin<Output>,
        pub right_indicator_output: ErasedPin<Output>,
        pub left_indicator_output: ErasedPin<Output>
    }

    #[task(local = [watchdog])]
    async fn watchdog(cx: watchdog::Context) {
        loop {
            cx.local.watchdog.feed();
            Systick::delay(80_u64.millis()).await;
        }
    }

    #[task(local = [led_ok])]
    async fn heartbeat(mut cx: heartbeat::Context){
        loop {
            cx.local.led_ok.set_high();
            Systick::delay(500.millis()).await;
            cx.local.led_ok.set_low();
            Systick::delay(500.millis()).await;
        }
    }

    #[task(local = [led_error])]
    async fn trigger_led_error(mut cx: trigger_led_error::Context){
        cx.local.led_error.set_high();
    }

    #[task(local = [led_warn])]
    async fn trigger_led_warn(mut cx: trigger_led_warn::Context){
        cx.local.led_warn.set_high();
    }

    #[task(priority = 1, shared = [light_states], local = [right_indicator_output])]
    async fn toggle_right_indicator(mut cx: toggle_right_indicator::Context){
        let right_ind: &mut ErasedPin<Output> = cx.local.right_indicator_output;
        let time = Systick::now();
        let on = (time.duration_since_epoch().to_millis() % 1000) > 500;

        cx.shared.light_states.lock(|ls| {
            right_ind.set_state(PinState::from(on && (ls.right_indicator > 0)));
        })
    }

    #[task(priority = 1, shared = [light_states], local = [left_indicator_output])]
    async fn toggle_left_indicator(mut cx: toggle_left_indicator::Context){
        let left_ind: &mut ErasedPin<Output> = cx.local.left_indicator_output;
        let time = Systick::now();
        let on = (time.duration_since_epoch().to_millis() % 1000) > 500;

        cx.shared.light_states.lock(|ls| {
            left_ind.set_state(PinState::from(on && (ls.left_indicator > 0)));
        })
    }

    #[task(priority = 1, shared = [light_states], local = [brake_light_output])]
    async fn toggle_brake_lights(mut cx: toggle_brake_lights::Context){
        let brake: &mut ErasedPin<Output> = cx.local.brake_light_output;
        cx.shared.light_states.lock(|ls| {
            brake.set_state(PinState::from(ls.brake_lights > 0));
        })
    }

    extern "Rust" {
        #[init]
        fn init(mut cx: init::Context) -> (Shared, Local);

        #[task(binds = FDCAN1_IT0, priority = 2, shared = [fdcan1_rx0])]
        fn can_rx0_pending(mut cx: can_rx0_pending::Context);

        #[task(binds = FDCAN1_IT1, priority = 2, shared = [fdcan1_rx1])]
        fn can_rx1_pending(mut cx: can_rx1_pending::Context);

        #[task(priority = 1)]
        async fn can_receive(mut cx: can_receive::Context, frame: RxFrameInfo, buffer: [u8; 8]);

        #[task(shared = [light_states])]
        async fn update_light_states(mut cx: update_light_states::Context, state: LampsState);
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