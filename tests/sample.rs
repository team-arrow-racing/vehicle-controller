#![no_std]
#![no_main]

use defmt_rtt as _;
use panic_probe as _;
use stm32h7xx_hal as hal;

#[defmt_test::tests]
mod tests {
    #[test]
    fn test() {
        defmt::info!("Hello test!");
    }
}
