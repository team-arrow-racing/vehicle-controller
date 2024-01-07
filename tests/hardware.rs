//! Hardware Self-test

#![no_std]
#![no_main]

use defmt_rtt as _;
use panic_probe as _;
use stm32h7xx_hal as hal;

use hal::device::Peripherals;
use hal::prelude::*;

#[defmt_test::tests]
mod tests {
    use super::*;

    #[test]
    fn fdcan1_transceiver_present() {
        let device = Peripherals::take().unwrap();

        let syscfg = device.SYSCFG;
        let rcc = device.RCC.constrain();

        // configure power domain
        let pwr = device.PWR.constrain().smps().freeze();

        // configure clocks
        let clocks = rcc.freeze(pwr, &syscfg);

        let gpiob = device.GPIOB.split(clocks.peripheral.GPIOB);

        let rx = gpiob.pb8.into_pull_down_input();
        let tx = gpiob.pb9.into_pull_down_input();

        assert!(rx.is_high());
        assert!(tx.is_high());
    }
}
