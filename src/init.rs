use crate::app::{Local, Shared, init};

use defmt_rtt as _;
use panic_probe as _;

use hal::prelude::*;
use hal::rcc;
use hal::rcc::RccExt;
use hal::serial::*;
use panic_probe as _;
use rtic_monotonics::systick::Systick;
use stm32g4xx_hal as hal;

pub fn init(cx: init::Context) -> (Shared, Local) {
    defmt::info!("Team Arrow Vehicle Controller");
    defmt::info!("(C) 2025 Team Arrow Racing");

    let dp = cx.device;
    // let cp = cx.core;

    let rcc = dp.RCC.constrain();
    let pwr = dp.PWR.constrain().freeze();
    let mut rcc = rcc.freeze(rcc::Config::hsi(), pwr);

    defmt::info!("systick: Starting...");
    Systick::start(
        cx.core.SYST,
        rcc.clocks.sys_clk.to_Hz(),
        rtic_monotonics::create_systick_token!(),
    );

    defmt::info!("radio: Initialising...");
    let gpioc = dp.GPIOC.split(&mut rcc);
    let pin_tx = gpioc.pc4.into_alternate();
    let pin_rx = gpioc.pc5.into_alternate();
    let usart = dp
        .USART1
        .usart(
            pin_tx,
            pin_rx,
            FullConfig::default()
                .baudrate(115200.bps())
                .fifo_enable()
                .rx_fifo_enable_interrupt()
                .rx_fifo_threshold(FifoThreshold::FIFO_4_BYTES),
            &mut rcc,
        )
        .unwrap();
    let (tx, mut rx) = usart.split();
    rx.listen();
    defmt::info!("radio: Ready!");

    defmt::info!("Finished Initialisation!");

    return (Shared {}, Local { tx, rx });
}
