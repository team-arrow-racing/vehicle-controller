use crate::app::*;
use core::mem::MaybeUninit;
use rtic::mutex_prelude::*;

#[link_section = ".axisram.radio_ingress"]
pub static mut INGRESS_BUFFER: MaybeUninit<[u8; 256]> = MaybeUninit::uninit();

pub fn radio_ingress(mut cx: radio_ingress::Context<'_>) {
    defmt::trace!("Radio frame received");

    cx.shared.radio_dma_transfer.lock(|transfer| {
        transfer.clear_transfer_complete_interrupt();
    });
}
