use crate::app::{blink, init, transmit_uart, watchdog, Local, Shared};
use core::num::{NonZeroU16, NonZeroU8};
use fdcan::{
    config::{DataBitTiming, NominalBitTiming},
    interrupt::{InterruptLine, Interrupts},
};
use rtic_monotonics::systick::*;
use stm32h7xx_hal::{
    dma::{
        dma::{DmaConfig, StreamsTuple},
        MemoryToPeripheral, PeripheralToMemory, Transfer,
    },
    gpio::{Edge, ExtiPin, Speed},
    independent_watchdog::IndependentWatchdog,
    prelude::*,
    rcc::{self, rec::FdcanClkSel},
    serial::{config::Config, Event},
};

pub fn init(mut cx: init::Context) -> (Shared, Local) {
    defmt::info!("init");
    // configure power domain
    let mut pwr = cx
        .device
        .PWR
        .constrain()
        .backup_regulator()
        .smps()
        .vos0(&cx.device.SYSCFG)
        .freeze();
    let backup = pwr.backup().unwrap();

    // RCC
    let rcc = cx.device.RCC.constrain();
    let ccdr = rcc
        .sysclk(480.MHz())
        .pll1_strategy(rcc::PllConfigStrategy::Iterative)
        .pll1_q_ck(200.MHz())
        .freeze(pwr, &cx.device.SYSCFG);

    // GPIO
    let gpioa = cx.device.GPIOA.split(ccdr.peripheral.GPIOA);
    let gpiob = cx.device.GPIOB.split(ccdr.peripheral.GPIOB);
    let gpioc = cx.device.GPIOC.split(ccdr.peripheral.GPIOC);
    let gpiod = cx.device.GPIOD.split(ccdr.peripheral.GPIOD);
    let gpioe = cx.device.GPIOE.split(ccdr.peripheral.GPIOE);
    let gpiog = cx.device.GPIOG.split(ccdr.peripheral.GPIOG);
    let gpioh = cx.device.GPIOH.split(ccdr.peripheral.GPIOH);

    // Button
    let mut button = gpioc.pc13.into_floating_input();
    button.make_interrupt_source(&mut cx.device.SYSCFG);
    button.trigger_on_edge(&mut cx.device.EXTI, Edge::Rising);
    button.enable_interrupt(&mut cx.device.EXTI);

    // Lights
    let left_ind_light = gpioc.pc1.into_push_pull_output();
    let right_ind_light = gpioc.pc0.into_push_pull_output();

    // USART
    let (serial_tx, serial_rx) = {
        let tx = gpiod.pd5.into_alternate();
        let rx = gpiod.pd6.into_alternate();

        let config = Config::new(115_200.bps()).lastbitclockpulse(true);

        let mut serial = cx
            .device
            .USART2
            .serial((tx, rx), config, ccdr.peripheral.USART2, &ccdr.clocks)
            .unwrap();

        serial.listen(Event::Idle);
        serial.enable_dma_rx();
        serial.split()
    };

    let tx_buffer = cortex_m::singleton!(: [u8; 10] = [0; 10]).unwrap();
    let rx_buffer1 = cortex_m::singleton!(: [u8; 10] = [0; 10]).unwrap();
    let rx_buffer2 = cortex_m::singleton!(: [u8; 10] = [0; 10]).unwrap();

    // Setup the DMA transfer on stream 0
    //
    // We need to specify the direction with a type annotation, since DMA
    // transfers both to and from the UART are possible
    let streams = StreamsTuple::new(cx.device.DMA1, ccdr.peripheral.DMA1);

    let config = DmaConfig::default().memory_increment(true);

    let transfer: Transfer<_, _, MemoryToPeripheral, _, _> =
        Transfer::init(streams.0, serial_tx, tx_buffer, None, config);

    let mut receive: Transfer<_, _, PeripheralToMemory, _, _> = Transfer::init(
        streams.1,
        serial_rx,
        rx_buffer1,
        None,
        DmaConfig::default()
            .memory_increment(true)
            .fifo_enable(true)
            .fifo_error_interrupt(true)
            .transfer_complete_interrupt(true),
    );

    receive.start(|_rx| {});

    defmt::info!("Chunked transfer complete!");

    // CAN
    let (fdcan1_ctrl, fdcan1_tx, fdcan1_rx0, fdcan1_rx1) = {
        let fdcan_prec = ccdr.peripheral.FDCAN.kernel_clk_mux(FdcanClkSel::Pll1Q);

        let rx = gpiod.pd0.into_alternate().speed(Speed::VeryHigh);
        let tx = gpiod.pd1.into_alternate().speed(Speed::VeryHigh);

        let mut can = cx.device.FDCAN1.fdcan(tx, rx, fdcan_prec);

        // throw error rather than trying to handle unexpected bus behaviour
        can.set_protocol_exception_handling(false);

        // k-clock 32MHz, bit rate 500kbit/s, sample point 87.5%
        can.set_nominal_bit_timing(NominalBitTiming {
            prescaler: NonZeroU16::new(4).unwrap(),
            seg1: NonZeroU8::new(13).unwrap(),
            seg2: NonZeroU8::new(2).unwrap(),
            sync_jump_width: NonZeroU8::new(1).unwrap(),
        });
        // k-clock 32MHz, bit rate 500kbit/s, sample point 87.5%
        can.set_data_bit_timing(DataBitTiming {
            prescaler: NonZeroU8::new(4).unwrap(),
            seg1: NonZeroU8::new(13).unwrap(),
            seg2: NonZeroU8::new(2).unwrap(),
            sync_jump_width: NonZeroU8::new(1).unwrap(),
            transceiver_delay_compensation: true,
        });

        can.enable_interrupt_line(InterruptLine::_0, true);
        can.enable_interrupts(Interrupts::RX_FIFO0_NEW_MSG | Interrupts::RX_FIFO1_NEW_MSG);

        can.into_external_loopback().split()
    };

    // setup and start independent watchdog
    // initialisation must complete before the watchdog triggers
    let watchdog = {
        let mut wd = IndependentWatchdog::new(cx.device.IWDG1);
        wd.start(100_u32.millis());
        wd
    };

    // configure real-time clock
    // let rtc = {
    //     Rtc::open_or_init(
    //         cx.device.RTC,
    //         backup.RTC,
    //         RtcClock::Lse {
    //             freq: 32_768.Hz(),
    //             bypass: false,
    //             css: false,
    //         },
    //         &ccdr.clocks,
    //     )
    // };

    // Monotonics
    let token = rtic_monotonics::create_systick_token!();
    Systick::start(cx.core.SYST, ccdr.clocks.sysclk().to_Hz(), token);

    watchdog::spawn().ok();
    blink::spawn().ok();
    transmit_uart::spawn().ok();
    // serial_gen::spawn().ok();
    // can_gen::spawn().ok();

    defmt::info!("Initialisation finished.");

    (
        Shared {
            status_led: gpioe.pe1.into_push_pull_output(),
            fdcan1_ctrl,
            fdcan1_tx,
            fdcan1_rx0,
            fdcan1_rx1,
            is_left_ind_on: false,
            is_right_ind_on: false,
            receive,
            transfer, // rtc,
        },
        Local {
            button,
            watchdog,
            // serial_rx,
            // serial_tx,
            left_ind_light,
            right_ind_light,
            rx_buffer: Some(rx_buffer2),
            buf_index: 0,
        },
    )
}
