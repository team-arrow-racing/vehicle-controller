use defmt_rtt as _;
use fdcan::frame::FrameFormat;
use fdcan::frame::TxFrameHeader;
use fdcan::id::StandardId;
use panic_probe as _;
use hal::can::Can;
use hal::nb::block;
use stm32g4xx_hal as hal;

pub fn fdcan_tx(data: &[u8], can: &mut fdcan::FdCan<Can<hal::pac::FDCAN1>, fdcan::ExternalLoopbackMode>) {
    defmt::info!("Create Message Data");
    let mut buffer: [u8; 8] = [0xAA, 0xAA, 0xAA, 0xAA, 0xFF, 0xFF, 0xFF, 0xFF];
    defmt::info!("Create Message Header");
    let header = TxFrameHeader {
        len: 2 * 4,
        id: StandardId::new(0x1).unwrap().into(),
        frame_format: FrameFormat::Standard,
        bit_rate_switching: false,
        marker: None,
    };

    defmt::info!("Transmit initial message");
    block!(can.transmit(header, &buffer)).unwrap();

    #[allow(irrefutable_let_patterns)]
    if let Ok(rxheader) = block!(can.receive0(&mut buffer)) {
        defmt::info!("Message: {}", buffer)
    }
}
