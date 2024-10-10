use crate::app::{init, watchdog, Lights, Local, Shared};
use crate::hal::{
    gpio::Speed,
    independent_watchdog::IndependentWatchdog,
    prelude::*,
    rcc::{self, rec::FdcanClkSel},
};
use core::num::{NonZeroU16, NonZeroU8};
use fdcan::{
    config::{DataBitTiming, NominalBitTiming},
    interrupt::{InterruptLine, Interrupts},
};
use rtic_monotonics::systick::*;

pub fn init(cx: init::Context) -> (Shared, Local) {
    defmt::info!("init");

    // Setup and start independent watchdog.
    // Initialisation must complete before the watchdog triggers
    let watchdog = {
        let mut wd = IndependentWatchdog::new(cx.device.IWDG1);
        wd.start(100_u32.millis());
        wd
    };

    // configure power domain
    let pwr = cx
        .device
        .PWR
        .constrain()
        .backup_regulator()
        .smps()
        .vos0(&cx.device.SYSCFG)
        .freeze();

    // RCC
    let rcc = cx.device.RCC.constrain();
    let ccdr = rcc
        .sysclk(480.MHz())
        .pll1_strategy(rcc::PllConfigStrategy::Iterative)
        .pll1_q_ck(200.MHz())
        .freeze(pwr, &cx.device.SYSCFG);

    // Monotonics
    Systick::start(
        cx.core.SYST,
        ccdr.clocks.sysclk().to_Hz(),
        rtic_monotonics::create_systick_token!(),
    );

    // GPIO
    let gpioa = cx.device.GPIOA.split(ccdr.peripheral.GPIOA);
    let gpiob = cx.device.GPIOB.split(ccdr.peripheral.GPIOB);
    let gpiod = cx.device.GPIOD.split(ccdr.peripheral.GPIOD);
    


    // Status LEDs
    let led_ok = gpiob.pb0.into_push_pull_output().erase();
    let led_warn = gpiob.pb7.into_push_pull_output().erase();
    let led_error = gpiob.pb14.into_push_pull_output().erase();

    //Light outputs
    let brake_light_output = gpioa.pa1.into_push_pull_output().erase();
    let right_indicator_output = gpioa.pa2.into_push_pull_output().erase();
    let left_indicator_output = gpioa.pa3.into_push_pull_output().erase();

    let light_states = Lights {
        brake_lights: 0,
        right_indicator: 0,
        left_indicator: 0
    };

    // CAN
    let (fdcan1_ctrl, fdcan1_tx, fdcan1_rx0, fdcan1_rx1) = {
        let tx = gpiod.pd1.into_alternate().speed(Speed::VeryHigh);
        let rx = gpiod.pd0.into_alternate().speed(Speed::VeryHigh);
        let fdcan_prec = ccdr.peripheral.FDCAN.kernel_clk_mux(FdcanClkSel::Pll1Q);
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
        can.enable_interrupt_line(InterruptLine::_1, true);
        can.enable_interrupts(Interrupts::RX_FIFO0_NEW_MSG | Interrupts::RX_FIFO1_NEW_MSG);

        can.into_normal().split()
    };

    watchdog::spawn().ok();

    defmt::info!("Initialisation finished.");

    (
        Shared {
            fdcan1_ctrl,
            fdcan1_tx,
            fdcan1_rx0,
            fdcan1_rx1,
            light_states,
        },
        Local {
            watchdog,
            led_ok,
            led_warn,
            led_error,
            brake_light_output,
            right_indicator_output,
            left_indicator_output,
        },
    )
}
