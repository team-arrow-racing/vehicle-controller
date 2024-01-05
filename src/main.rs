#![no_main]
#![no_std]

use defmt_rtt as _;
use panic_probe as _;
use stm32h7xx_hal as hal;

use hal::prelude::*;
use rtic_monotonics::systick::Systick;
use rtic_monotonics::Monotonic;

#[rtic::app(device = stm32h7xx_hal::pac)]
mod app {
    use super::*;

    #[shared]
    struct Shared {}

    #[local]
    struct Local {}

    #[init]
    fn init(cx: init::Context) -> (Shared, Local) {
        let syscfg = cx.device.SYSCFG;
        let pwr = cx.device.PWR.constrain();
        let pwrcfg = pwr.smps().freeze();
        let rcc = cx.device.RCC.constrain();
        let clocks = rcc.sysclk(240.MHz()).freeze(pwrcfg, &syscfg);

        let systick_mono_token = rtic_monotonics::create_systick_token!();
        Systick::start(
            cx.core.SYST,
            clocks.clocks.sysclk().to_Hz(),
            systick_mono_token,
        );

        defmt::info!("Initialisation finished.");

        (Shared {}, Local {})
    }
}

defmt::timestamp!("{=u64:us}", {
    Systick::now().duration_since_epoch().to_micros()
});
