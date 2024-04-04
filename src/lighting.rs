use crate::app::*;
use rtic::Mutex;
use rtic_monotonics::{systick::*, Monotonic};
use stm32h7xx_hal::gpio::PinState;

pub async fn toggle_indicators(mut cx: toggle_indicators::Context<'_>) {
    loop {
        let l_light = &mut *cx.local.left_ind_light;
        let r_light = &mut *cx.local.right_ind_light;
        let time = Systick::now();

        let on: bool = (time.duration_since_epoch().to_millis() % 1000) > 500;

        cx.shared.is_left_ind_on.lock(|l_on| {
            l_light.set_state(PinState::from(*l_on && on));
            defmt::trace!("l light {}", l_light.is_set_high());
        });

        cx.shared.is_right_ind_on.lock(|r_on| {
            r_light.set_state(PinState::from(*r_on && on));
            defmt::trace!("r light {}", r_light.is_set_high());
        });

        Systick::delay(10_u64.millis()).await;
    }
}
