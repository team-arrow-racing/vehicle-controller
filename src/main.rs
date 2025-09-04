// #![deny(warnings)]
#![deny(unsafe_code)]
#![no_main]
#![no_std]

use defmt_rtt as _;
use panic_probe as _;

use hal::gpio::{Alternate, Pin};
use hal::pac::USART1;
use hal::serial::{NoDMA, Rx, Tx};
use stm32g4xx_hal as hal;

use rtic_monotonics::{Monotonic, systick::Systick};

mod init;
mod radio;

use init::*;

#[rtic::app(device = hal::pac)]
mod app {
    use super::*;
    #[shared]
    pub struct Shared {}

    #[local]
    pub struct Local {
        pub tx: Tx<USART1, Pin<'C', 4, Alternate<7>>, NoDMA>,
        pub rx: Rx<USART1, Pin<'C', 5, Alternate<7>>, NoDMA>,
    }

    extern "Rust" {
        #[init]
        fn init(mut cx: init::Context) -> (Shared, Local);
    }
    #[task(binds = USART1, local = [rx])]
    fn radio_interrupt(cx: radio_interrupt::Context) {
        radio::parse_message(cx.local.rx);
    }
}

#[defmt::panic_handler]
fn panic() -> ! {
    cortex_m::asm::udf()
}

defmt::timestamp!("{=u64:us}", {
    Systick::now().duration_since_epoch().to_micros()
});
