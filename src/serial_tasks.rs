use stm32h7xx_hal::prelude::*;
use core::fmt::Write;
use rtic::Mutex;

use rtic_monotonics::{systick::*, Monotonic};
use stm32h7xx_hal::nb::block;
use crate::app::*;

// pub fn usart2_callback(mut cx: usart2_callback::Context) {
//     defmt::info!("usart2");
//     let rx = cx.local.serial_rx;
//     let buf = cx.local.rx_buffer;
//     let ind = cx.local.buf_index;

//     if let Ok(byte) = block!(rx.read()) {
//         defmt::info!("{:#02x}", byte);
//         if byte == 0xFF {
//             *ind = 0;
//             *buf = [0; 10];
//             buf[*ind] = byte;
//         } else {
//             buf[*ind] = byte;
//         }
//         *ind += 1;
//         defmt::info!("{:#02x}", buf);
//         // TlvcReader::begin(&[byte][..]).unwrap();
//     }
// }

// pub async fn serial_gen(mut cx: serial_gen::Context<'_>) {
//     loop {
//         defmt::info!("writing");
//         let tx = &mut *cx.local.serial_tx;
//         // Test for retrieving firmware version
//         // Serial number
//         let buf: [u8; 4] = [0xFF, 0x0C, 0x00, 0xF3];
//         for c in buf {
//             let _ = tx.write(c);
//         }

//         Systick::delay(1000_u64.millis()).await;
//     }
// }