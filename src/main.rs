#![no_main]
#![no_std]
#![feature(type_alias_impl_trait)]

use defmt_rtt as _;
use panic_probe as _;
use stm32h7xx_hal as hal;

use core::num::{NonZeroU16, NonZeroU8};
use fdcan::{
    config::{DataBitTiming, NominalBitTiming},
    FdCanControl, Fifo0, Fifo1, NormalOperationMode, Rx, Tx,
};
use hal::gpio::Speed;
use hal::prelude::*;
use hal::rcc::rec::FdcanClkSel;
use hal::{can::Can, device::FDCAN1};
use rtic_monotonics::systick::*;
use rtic_monotonics::Monotonic;

#[rtic::app(device = stm32h7xx_hal::pac)]
mod app {
    use super::*;

    #[shared]
    struct Shared {
        fdcan1_ctrl: FdCanControl<Can<FDCAN1>, NormalOperationMode>,
        fdcan1_tx: Tx<Can<FDCAN1>, NormalOperationMode>,
        fdcan1_rx0: Rx<Can<FDCAN1>, NormalOperationMode, Fifo0>,
        fdcan1_rx1: Rx<Can<FDCAN1>, NormalOperationMode, Fifo1>,
    }

    #[local]
    struct Local {}

    #[init]
    fn init(cx: init::Context) -> (Shared, Local) {
        let syscfg = cx.device.SYSCFG;
        let rcc = cx.device.RCC.constrain();

        let pwr = cx.device.PWR.constrain().smps().vos0(&syscfg).freeze();

        let clocks = rcc
            .sysclk(480.MHz())
            .pll1_strategy(hal::rcc::PllConfigStrategy::Iterative)
            .pll1_q_ck(32.MHz())
            .freeze(pwr, &syscfg);

        let systick_mono_token = rtic_monotonics::create_systick_token!();
        Systick::start(
            cx.core.SYST,
            clocks.clocks.sysclk().to_Hz(),
            systick_mono_token,
        );

        let gpiob = cx.device.GPIOB.split(clocks.peripheral.GPIOB);

        let (fdcan1_ctrl, fdcan1_tx, fdcan1_rx0, fdcan1_rx1) = {
            let fdcan_prec = clocks.peripheral.FDCAN.kernel_clk_mux(FdcanClkSel::Pll1Q);

            let rx = gpiob.pb8.into_alternate().speed(Speed::VeryHigh);
            let tx = gpiob.pb9.into_alternate().speed(Speed::VeryHigh);

            let mut can = cx.device.FDCAN1.fdcan(tx, rx, fdcan_prec);

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

            can.into_normal().split()
        };

        defmt::info!("Initialisation finished.");

        (
            Shared {
                fdcan1_ctrl,
                fdcan1_tx,
                fdcan1_rx0,
                fdcan1_rx1,
            },
            Local {},
        )
    }
}

defmt::timestamp!("{=u64:us}", {
    Systick::now().duration_since_epoch().to_micros()
});
