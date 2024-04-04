use crate::app::*;
use fdcan::{frame::RxFrameInfo, id::Id};
use rtic::Mutex;
use stm32h7xx_hal::nb::block;

pub fn can_rx0_pending(mut cx: can_rx0_pending::Context) {
    defmt::trace!("RX0 received");
    cx.shared.fdcan1_rx0.lock(|rx| {
        let mut buffer = [0_u8; 8];
        let rxframe = block!(rx.receive(&mut buffer));

        if let Ok(rxframe) = rxframe {
            can_receive::spawn(rxframe.unwrap(), buffer).ok();
        }
    });
}

pub fn can_rx1_pending(mut cx: can_rx1_pending::Context) {
    defmt::trace!("RX1 received");
    cx.shared.fdcan1_rx1.lock(|rx| {
        let mut buffer = [0_u8; 8];
        let rxframe = block!(rx.receive(&mut buffer));

        if let Ok(rxframe) = rxframe {
            can_receive::spawn(rxframe.unwrap(), buffer).ok();
        }
    });
}

pub async fn can_receive(_cx: can_receive::Context<'_>, frame: RxFrameInfo, buffer: [u8; 8]) {
    let id = frame.id;
    match id {
        Id::Standard(id) => defmt::info!("Received Header: {:#02x}", id.as_raw()),
        Id::Extended(id) => defmt::info!("Received Header: {:#03x}", id.as_raw()),
    };
    defmt::info!("received data: {:#02x}", buffer);
}
