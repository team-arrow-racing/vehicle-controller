use defmt_rtt as _;
use embedded_io::Read;
use embedded_io::ReadReady;
use embedded_io::Write;
use hal::gpio::{Alternate, Pin};
use hal::pac::USART1;
use hal::serial::*;
use panic_probe as _;
use stm32g4xx_hal as hal;

pub fn radio_tx(data: &[u8], tx: &mut Tx<USART1, Pin<'C', 4, Alternate<7>>, NoDMA>) {
    let preamble = [0x02, 0x00, data.len() as u8];
    let mut check: u8 = 0;
    for byte in preamble {
        check ^= byte;
    }
    for byte in data {
        check ^= byte;
    }
    tx.write_all(&preamble).unwrap();
    tx.write_all(data).unwrap();
    tx.write_all(&[check]).unwrap();
}

pub fn radio_rx(rx: &mut Rx<USART1, Pin<'C', 5, Alternate<7>>, NoDMA>) -> usize {
    let mut buffer = [0; 256];
    loop {
        if rx.read(&mut buffer[..1]).is_err() {
            defmt::warn!("radio: Read Error!");
            return 0;
        }
        if buffer[0] == 0x02 {
            break;
        }
        if !rx.read_ready().unwrap() {
            return 0;
        }
    }

    if rx.read(&mut buffer[1..2]).is_err() {
        return 0;
    }

    let mut len = 0;

    match buffer[1] {
        0x81 => {
            rx.read(&mut buffer[2..3]).unwrap();
            len = buffer[2] as usize;
            rx.read_exact(&mut buffer[3..3 + len + 1]).unwrap();
            let mut check = 0;
            for &byte in &buffer[0..3 + len] {
                check ^= byte;
            }
            if buffer[3 + len] != check {
                defmt::warn!(
                    "radio: Checksum mismatch: expected {}, got {}",
                    check,
                    buffer[3 + len]
                );
                return 0;
            }
            defmt::debug!("radio: Received message: {}", &buffer[3..3 + len - 1]);
        }
        _ => {
            defmt::warn!("radio: Unknown message type: {}", buffer[1]);
        }
    }

    return len;
}
