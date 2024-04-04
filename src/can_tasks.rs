use fdcan::{
    frame::{FrameFormat, TxFrameHeader, RxFrameInfo},
    id::{StandardId, Id},
};
use stm32h7xx_hal::nb::block;
use rtic::Mutex;
use rtic_monotonics::systick::*;
use crate::app::*;

pub async fn can_gen(mut cx: can_gen::Context<'_>) {
    loop {
        cx.shared.fdcan1_tx.lock(|tx| {
            defmt::info!("Sending CAN");
            let header = TxFrameHeader {
                len: 8,
                id: StandardId::new(0x42).unwrap().into(),
                frame_format: FrameFormat::Fdcan,
                bit_rate_switching: true,
                marker: None
            };

            let buffer = [0, 1, 2, 3, 4, 5, 6, 7];

            block!(tx.transmit(header, &buffer)).unwrap();
        });

        Systick::delay(1000_u64.millis()).await;
    }
}

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

pub async fn can_receive(cx: can_receive::Context<'_>, frame: RxFrameInfo, buffer: [u8; 8]) {
    let id = frame.id;
    match id {
        Id::Standard(id) => defmt::info!("Received Header: {:#02x}", id.as_raw()),
        Id::Extended(id) => defmt::info!("Received Header: {:#03x}", id.as_raw()),
        _ => {}
    };
    defmt::info!("received data: {:#02x}", buffer);
}