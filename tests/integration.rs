#![no_std]
#![no_main]

use defmt_rtt as _;
use panic_probe as _;
use stm32l4xx_hal as _;

#[defmt::panic_handler]
fn panic() -> ! {
    cortex_m::asm::udf()
}

#[defmt_test::tests]
mod tests {
    use defmt::assert;

    #[test]
    fn it_works() {
        assert!(true)
    }
}
