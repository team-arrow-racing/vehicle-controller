use core::{future::poll_fn, task::Poll};

use embedded_io_async::ErrorKind;
use hal::dma::traits::{Direction, Stream, TargetAddress};

use crate::hal;

pub struct SerialRx<STREAM, PERIPHERAL, DIR, BUF, TXFRT>
where
    STREAM: Stream,
    PERIPHERAL: TargetAddress<DIR>,
    DIR: Direction,
{
    transfer: hal::dma::Transfer<STREAM, PERIPHERAL, DIR, BUF, TXFRT>,
}

impl<STREAM, PERIPHERAL, DIR, BUF, TXFRT> embedded_io_async::Read
    for SerialRx<STREAM, PERIPHERAL, DIR, BUF, TXFRT>
where
    STREAM: Stream,
    PERIPHERAL: TargetAddress<DIR>,
    DIR: Direction,
{
    async fn read(&mut self, buf: &mut [u8]) -> Result<usize, Self::Error> {
        poll_fn(|cx| {
            if self.transfer.get_transfer_complete_flag() {
                Poll::Ready(Ok(0))
            } else {
                Poll::Pending
            }
        })
        .await
    }
}

impl<STREAM, PERIPHERAL, DIR, BUF, TXFRT> embedded_io_async::ErrorType
    for SerialRx<STREAM, PERIPHERAL, DIR, BUF, TXFRT>
where
    STREAM: Stream,
    PERIPHERAL: TargetAddress<DIR>,
    DIR: Direction,
{
    type Error = ErrorKind;
}
