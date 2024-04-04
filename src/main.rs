#![no_main]
#![no_std]
#![feature(type_alias_impl_trait)]
#![allow(clippy::transmute_ptr_to_ptr)]

mod canbus;
mod init;
mod lighting;
mod serial_tasks;

// import tasks
use canbus::*;
use init::*;
use lighting::*;

// global logger
use defmt_rtt as _;
use panic_probe as _;
use stm32h7xx_hal as hal;

use fdcan::{ExternalLoopbackMode, FdCanControl, Fifo0, Fifo1, Rx, Tx};
use hal::{
    can::Can,
    dma::{dma::Stream0, DBTransfer, MemoryToPeripheral, Transfer},
    dma::{dma::Stream1, PeripheralToMemory},
    gpio::{ExtiPin, Input, PC0, PC1, PC13},
    gpio::{Output, PushPull, PE1},
    independent_watchdog::IndependentWatchdog,
    pac,
    pac::FDCAN1,
    serial,
};
use rtic_monotonics::systick::ExtU64;
use rtic_monotonics::systick::Systick;
use rtic_monotonics::Monotonic;

#[rtic::app(device = stm32h7xx_hal::pac, dispatchers = [UART4, SPI1])]
mod app {
    type FdCanMode = ExternalLoopbackMode;
    type RxTransfer<'a> = Transfer<
        Stream1<pac::DMA1>,
        serial::Rx<pac::USART2>,
        PeripheralToMemory,
        &'static mut [u8; 10],
        DBTransfer,
    >;
    type TxTransfer = Transfer<
        Stream0<pac::DMA1>,
        serial::Tx<pac::USART2>,
        MemoryToPeripheral,
        &'static mut [u8; 10],
        DBTransfer,
    >;

    use fdcan::frame::RxFrameInfo;

    use super::*;
    // Shared resources go here
    #[shared]
    pub struct Shared {
        pub status_led: PE1<Output<PushPull>>,
        pub fdcan1_ctrl: FdCanControl<Can<FDCAN1>, FdCanMode>,
        pub fdcan1_tx: Tx<Can<FDCAN1>, FdCanMode>,
        pub fdcan1_rx0: Rx<Can<FDCAN1>, FdCanMode, Fifo0>,
        pub fdcan1_rx1: Rx<Can<FDCAN1>, FdCanMode, Fifo1>,
        pub is_left_ind_on: bool,
        pub is_right_ind_on: bool,
        // rtc: Rtc,
        pub receive: RxTransfer<'static>,
        pub transfer: TxTransfer,
    }

    // Local resources go here
    #[local]
    pub struct Local {
        pub button: PC13<Input>,
        pub watchdog: IndependentWatchdog,
        // serial_rx: serial::Rx<stm32::USART2>,
        // serial_tx: serial::Tx<stm32::USART2>,
        pub left_ind_light: PC1<Output<PushPull>>,
        pub right_ind_light: PC0<Output<PushPull>>,
        pub rx_buffer: Option<&'static mut [u8; 10]>,
        pub buf_index: usize,
    }

    #[task(local = [watchdog])]
    async fn watchdog(cx: watchdog::Context) {
        loop {
            cx.local.watchdog.feed();
            Systick::delay(80_u64.millis()).await;
        }
    }

    #[task(binds = EXTI15_10, shared = [status_led], local = [button])]
    fn button_click(mut cx: button_click::Context) {
        cx.local.button.clear_interrupt_pending_bit();
        cx.shared.status_led.lock(|led| {
            led.toggle();
        });
    }

    #[task(shared = [status_led], priority=1)]
    async fn blink(mut cx: blink::Context) {
        loop {
            cx.shared.status_led.lock(|led| {
                defmt::trace!("blinking");
                led.toggle();
            });
            Systick::delay(1000_u64.millis()).await;
        }
    }

    #[task(priority=1, shared=[transfer])]
    async fn transmit_uart(mut cx: transmit_uart::Context) {
        loop {
            cx.shared.transfer.lock(|transfer| {
                transfer.start(|serial| {
                    // This closure runs right after enabling the stream
                    serial.enable_dma_tx();
                });
                defmt::info!("Started transfer!");

                // Wait for transfer to complete
                while !transfer.get_transfer_complete_flag() {}

                defmt::info!("Continuing with chunked transfer!");

                transfer.pause(|serial| {
                    // At this point, the DMA transfer is done, but the data is still in the
                    // UART output FIFO. Wait for it to complete
                    while !serial.is_txe() {}
                });
            });
            Systick::delay(1000_u64.millis()).await;
        }
    }

    #[task(binds = USART2, priority=1, local = [rx_buffer], shared = [receive])]
    fn usart2_callback(mut cx: usart2_callback::Context) {
        cx.shared.receive.lock(|transfer| {
            // defmt::info!("usart2_callback");
            if transfer.get_transfer_complete_flag() {
                // let bytes_count = 10 - transfer;
                defmt::info!("info");
                let new_buffer = cx.local.rx_buffer.take().unwrap();

                let (buffer, _, _) = transfer.next_transfer(new_buffer).unwrap();

                let _bytes = &buffer[..2];

                defmt::info!("{:?}", _bytes);

                *cx.local.rx_buffer = Some(buffer);
            }
        });
    }

    #[task(binds = DMA1_STR1, priority=1, shared=[receive])]
    fn dma_callback(mut cx: dma_callback::Context) {
        defmt::info!("dma_callback");
        cx.shared.receive.lock(|transfer| {
            if transfer.get_transfer_complete_flag() {
                defmt::info!("dma done");
            }
        });
    }

    extern "Rust" {
        #[init]
        fn init(mut cx: init::Context) -> (Shared, Local);

        #[task(shared = [fdcan1_tx], priority=1)]
        async fn can_gen(mut cx: can_gen::Context);

        #[task(binds = FDCAN1_IT0, priority = 2, shared = [fdcan1_rx0])]
        fn can_rx0_pending(mut cx: can_rx0_pending::Context);

        #[task(binds = FDCAN1_IT1, priority = 2, shared = [fdcan1_rx1])]
        fn can_rx1_pending(mut cx: can_rx1_pending::Context);

        #[task(priority = 1)]
        async fn can_receive(mut cx: can_receive::Context, frame: RxFrameInfo, buffer: [u8; 8]);

        #[task(shared = [is_left_ind_on, is_right_ind_on], local = [left_ind_light, right_ind_light], priority=1)]
        async fn toggle_indicators(mut cx: toggle_indicators::Context);

        #[task(priority = 1)]
        async fn toggle_brakes(mut cx: toggle_brakes::Context);

        // #[task(binds = USART2, local = [serial_rx, rx_buffer, buf_index])]
        // fn usart2_callback(mut cx: usart2_callback::Context);

        // #[task(local = [serial_tx], priority=1)]
        // async fn serial_gen(mut cx: serial_gen::Context);
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
