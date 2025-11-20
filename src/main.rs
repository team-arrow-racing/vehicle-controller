// #![deny(warnings)]
#![deny(unsafe_code)]
#![no_main]
#![no_std]

use defmt_rtt as _;
use panic_probe as _;

use hal::gpio::{Alternate, Pin};
use hal::pac::USART1;
use hal::serial::{NoDMA, Rx, Tx};
use hal::can::Can;
use stm32g4xx_hal as hal;

use rtic_monotonics::{Monotonic, systick::Systick};

mod init;
mod radio;
mod canbus;

use init::*;

#[rtic::app(device = hal::pac)]
mod app {
    use super::*;
    type CanType = fdcan::ExternalLoopbackMode;
    #[shared]
    pub struct Shared {
        pub can1: fdcan::FdCan<Can<hal::pac::FDCAN1>, CanType>,
    }

    #[local]
    pub struct Local {
        pub usart_tx: Tx<USART1, Pin<'C', 4, Alternate<7>>, NoDMA>,
        pub usart_rx: Rx<USART1, Pin<'C', 5, Alternate<7>>, NoDMA>,
    }

    extern "Rust" {
        #[init]
        fn init(mut cx: init::Context) -> (Shared, Local);
    }
    #[task(binds = USART1, local = [usart_rx])]
    fn radio_interrupt(cx: radio_interrupt::Context) {
        radio::radio_rx(cx.local.usart_rx);
    }

    #[task(binds = FDCAN1_IT0, shared = [can1])]
    fn can1_interrupt(mut cx: can1_interrupt::Context) {
        let mut buffer: [u8; 8] = [0; 8];
        cx.shared.can1.lock(|can1| {
            if let Ok(_rxreader) = can1.receive0(&mut buffer) {
                defmt::info!("Received CAN message: {:x}", buffer);
            }
        });
    }

    #[idle]
    fn idle(_cx: idle::Context) -> ! {
        loop {
            // Sleep until next interrupt
            // cortex_m::asm::wfi();
        }
    }
}

#[defmt::panic_handler]
fn panic() -> ! {
    cortex_m::asm::udf()
}

defmt::timestamp!("{=u64:us}", {
    Systick::now().duration_since_epoch().to_micros()
});
