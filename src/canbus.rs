use crate::app::*;
use fdcan::{frame::RxFrameInfo, id::Id};
use rtic::Mutex;
use stm32h7xx_hal::nb::block;

use solar_car::com::lighting::{LampsState, PGN_LIGHTING_STATE};
use j1939::pgn::Number;

fn pgn_from_rawid(rawid: u32) -> Number {
    //Isolates bit 9-18 for the pgn
    let raw_pgn = (rawid & 0x00FF0000) >> 8;

    //Split the PGN bitwise into sections
    let specific = (raw_pgn & 0xFF) as u8;
    let format = ((raw_pgn >> 8) & 0xFF) as u8;
    let data_page = ((raw_pgn >> 16) & 0x01) != 0;
    let extended_data_page = ((raw_pgn >> 17) & 0x01) != 0;
    
    Number {
        specific,
        format,
        data_page,
        extended_data_page,
    }
}

pub async fn update_light_states(mut cx: update_light_states::Context<'_>, state: LampsState){
    //update states from CAN lighting frame
    cx.shared.light_states.lock(|ls|{
        ls.brake_lights = state.contains(LampsState::STOP) as u8;
        ls.left_indicator = state.contains(LampsState::INDICATOR_LEFT) as u8;
        ls.right_indicator = state.contains(LampsState::INDICATOR_RIGHT) as u8;
    });
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

pub async fn can_receive(_cx: can_receive::Context<'_>, frame: RxFrameInfo, buffer: [u8; 8]) {
    let id = frame.id;
    match id {
        Id::Standard(id) => defmt::info!("Received Header: {:#02x}", id.as_raw()),
        Id::Extended(id) =>{

         defmt::info!("Received Header: {:#03x}", id.as_raw());
        
         let pgn = pgn_from_rawid(id.as_raw());

         match pgn {
             PGN_LIGHTING_STATE => {
                 let lamp_state = LampsState::from_bits(buffer[0]).unwrap_or_else(LampsState::empty);
             
                 //update stored lamp states directly from CAN for every frame
                 update_light_states::spawn(lamp_state).ok();

                 //update light states whenever lighting message is received
                 toggle_brake_lights::spawn().ok();
                 toggle_left_indicator::spawn().ok();
                 toggle_right_indicator::spawn().ok();

             },
             _ => {
                 defmt::info!("Received Unknown Message");
                 trigger_led_error::spawn().ok();
             }
            }
        }
    }
}
