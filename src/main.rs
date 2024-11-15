#![no_main]
#![no_std]
#![feature(type_alias_impl_trait)]
#![allow(clippy::transmute_ptr_to_ptr)]

mod async_io_adapters;
mod canbus;
mod init;
mod radio;

// import tasks
use canbus::*;
use init::*;
use radio::*;

// global logger
use defmt_rtt as _;
use panic_probe as _;
use stm32h7xx_hal as hal;

use fdcan::{frame::RxFrameInfo, FdCanControl, Fifo0, Fifo1, NormalOperationMode, Rx, Tx};
use hal::{
    can::Can,
    dma::{dma::Stream1, DBTransfer, PeripheralToMemory, Transfer},
    gpio::ErasedPin,
    gpio::Output,
    independent_watchdog::IndependentWatchdog,
    pac::FDCAN1,
    pac::{DMA1, USART3},
    serial,
};
use rtic_monotonics::{
    systick::{ExtU64, Systick},
    Monotonic,
};

#[rtic::app(device = stm32h7xx_hal::pac, dispatchers = [UART4, SPI1])]
mod app {
    use super::*;

    type FdCanMode = NormalOperationMode;

    #[shared]
    pub struct Shared {
        pub fdcan1_ctrl: FdCanControl<Can<FDCAN1>, FdCanMode>,
        pub fdcan1_tx: Tx<Can<FDCAN1>, FdCanMode>,
        pub fdcan1_rx0: Rx<Can<FDCAN1>, FdCanMode, Fifo0>,
        pub fdcan1_rx1: Rx<Can<FDCAN1>, FdCanMode, Fifo1>,

        pub radio_dma_transfer: Transfer<
            Stream1<DMA1>,
            serial::Rx<USART3>,
            PeripheralToMemory,
            &'static mut [u8; 256],
            DBTransfer,
        >,
    }

    #[local]
    pub struct Local {
        pub watchdog: IndependentWatchdog,
        pub led_ok: ErasedPin<Output>,
        pub led_warn: ErasedPin<Output>,
        pub led_error: ErasedPin<Output>,
    }

    #[task(local = [watchdog])]
    async fn watchdog(cx: watchdog::Context) {
        loop {
            cx.local.watchdog.feed();
            Systick::delay(80_u64.millis()).await;
        }
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

        #[task(binds = DMA1_STR1, shared = [radio_dma_transfer])]
        fn radio_ingress(cx: radio_ingress::Context);
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
