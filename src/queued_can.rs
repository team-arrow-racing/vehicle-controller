use stm32l4xx_hal::{
    can::Can,
    gpio::{Alternate, PushPull, PA11, PA12},
    pac::CAN1,
};

use heapless::spsc::Queue;

type Interface =
    Can<CAN1, (PA12<Alternate<PushPull, 9>>, PA11<Alternate<PushPull, 9>>)>;

type TxQueue = Queue<bxcan::Frame, 32>;

pub struct QueuedCan {
    can: bxcan::Can<Interface>,
    tx_queue: TxQueue,
}

impl QueuedCan {
    pub fn new(can: bxcan::Can<Interface>) -> Self {
        QueuedCan {
            can,
            tx_queue: TxQueue::new(),
        }
    }

    pub fn transmit(
        &mut self,
        frame: bxcan::Frame,
    ) -> Result<(), &'static str> {
        match self.tx_queue.enqueue(frame) {
            Ok(v) => Ok(()),
            Err(_) => Err("queue is full"),
        }
    }

    pub fn try_transmit(&mut self) {
        match self.tx_queue.dequeue() {
            Some(f) => {
                self.can.transmit(&f).unwrap();
            }
            None => {}
        }
    }
}
