use crate::{app::{Local, Shared, init}, canbus};

use core::num::{NonZeroU8, NonZeroU16};
use defmt_rtt as _;
use panic_probe as _;

use fdcan::{
    config::NominalBitTiming,
    filter::{StandardFilter, StandardFilterSlot},
};
use hal::can::CanExt;
use hal::gpio::Speed;
use hal::prelude::*;
use hal::rcc;
use hal::rcc::RccExt;
use hal::serial::*;
use hal::time::RateExtU32;
use panic_probe as _;
use rtic_monotonics::systick::Systick;
use stm32g4xx_hal as hal;

pub fn init(cx: init::Context) -> (Shared, Local) {
    defmt::info!("Team Arrow Vehicle Controller");
    defmt::info!("(C) 2025 Team Arrow Racing");

    let dp = cx.device;
    // let cp = cx.core;

    defmt::info!("init: clocks: Starting...");
    let rcc = dp.RCC.constrain();
    let pwr = dp.PWR.constrain().freeze();
    let mut rcc = rcc.freeze(rcc::Config::hse(24.MHz()), pwr);

    let btr = NominalBitTiming {
        prescaler: NonZeroU16::new(12).unwrap(),
        seg1: NonZeroU8::new(13).unwrap(),
        seg2: NonZeroU8::new(2).unwrap(),
        sync_jump_width: NonZeroU8::new(1).unwrap(),
    };

    defmt::info!("init: systick: Starting...");
    Systick::start(
        cx.core.SYST,
        rcc.clocks.sys_clk.to_Hz(),
        rtic_monotonics::create_systick_token!(),
    );

    defmt::info!("init: gpio: Setting...");
    let gpiob = dp.GPIOB.split(&mut rcc);
    let gpioc = dp.GPIOC.split(&mut rcc);

    defmt::info!("init: radio: Initialising...");
    let usart = {
        let pin_tx = gpioc.pc4.into_alternate();
        let pin_rx = gpioc.pc5.into_alternate();

        dp.USART1
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
            .unwrap()
    };

    let (usart_tx, mut usart_rx) = usart.split();
    usart_rx.listen();

    defmt::info!("init: can: Initialising...");
    let mut can1 = {
        let rx = gpiob.pb8.into_alternate().speed(Speed::VeryHigh);
        let tx = gpiob.pb9.into_alternate().speed(Speed::VeryHigh);
        let mut can = dp.FDCAN1.fdcan(tx, rx, &mut rcc);
        can.set_protocol_exception_handling(false);
        can.set_nominal_bit_timing(btr);
        can.set_standard_filter(
            StandardFilterSlot::_0,
            StandardFilter::accept_all_into_fifo0(),
        );
        can.into_external_loopback()
        // can.into_normal()
    };

    canbus::fdcan_tx(&[], &mut can1);

    defmt::info!("init: Finished Initialisation!");

    return (Shared { can1 }, Local { usart_tx, usart_rx });
}
