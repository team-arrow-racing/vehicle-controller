#![no_main]
#![no_std]

use defmt_rtt as _;
use panic_probe as _;

use stm32l4xx_hal::{
    can::Can,
    gpio::{Output, Alternate, PushPull, PA11, PA12, PB13},
    pac::CAN1,
    prelude::*,
    watchdog::IndependentWatchdog,
};

use systick_monotonic::{
    fugit::{MillisDurationU32, MillisDurationU64},
    Systick,
};

use bxcan::{
    filter::Mask32,
    Interrupts,
};

type Duration = MillisDurationU64;

#[rtic::app(device = stm32l4xx_hal::pac, dispatchers = [SPI1])]
mod app {
    use super::*;

    #[monotonic(binds = SysTick, default = true)]
    type MonoTimer = Systick<1000>;

    #[shared]
    struct Shared {
        can: bxcan::Can<Can<CAN1, (PA12<Alternate<PushPull, 9>>, PA11<Alternate<PushPull, 9>>)>>,
    }

    #[local]
    struct Local {
        watchdog: IndependentWatchdog,
        status_led: PB13<Output<PushPull>>,
    }

    #[init]
    fn init(cx: init::Context) -> (Shared, Local, init::Monotonics) {
        defmt::info!("init");

        // peripherals
        let mut flash = cx.device.FLASH.constrain();
        let mut rcc = cx.device.RCC.constrain();
        let mut pwr = cx.device.PWR.constrain(&mut rcc.apb1r1);
        let mut gpioa = cx.device.GPIOA.split(&mut rcc.ahb2);
        let mut gpiob = cx.device.GPIOB.split(&mut rcc.ahb2);

        // configure system clock
        let clocks = rcc.cfgr.sysclk(80.MHz()).freeze(&mut flash.acr, &mut pwr);

        // configure monotonic time
        let mono = Systick::new(cx.core.SYST, clocks.sysclk().to_Hz());

        // configure status led
        let status_led = gpiob
            .pb13
            .into_push_pull_output(&mut gpiob.moder, &mut gpiob.otyper);

        // configure can bus
        let can = {
            let rx =
                gpioa
                    .pa11
                    .into_alternate(&mut gpioa.moder, &mut gpioa.otyper, &mut gpioa.afrh);
            let tx =
                gpioa
                    .pa12
                    .into_alternate(&mut gpioa.moder, &mut gpioa.otyper, &mut gpioa.afrh);

            let can = Can::new(&mut rcc.apb1r1, cx.device.CAN1, (tx, rx));

            bxcan::Can::builder(can)
        }
        .set_bit_timing(0x001c_0009) // 500kbit/s
        .set_loopback(false);

        let mut can = can.enable();

        can.modify_filters().enable_bank(0, Mask32::accept_all());

        can.enable_interrupts(
            Interrupts::TRANSMIT_MAILBOX_EMPTY | Interrupts::FIFO0_MESSAGE_PENDING,
        );
        nb::block!(can.enable_non_blocking()).unwrap();

        // configure watchdog
        let mut watchdog = IndependentWatchdog::new(cx.device.IWDG);
        watchdog.stop_on_debug(&cx.device.DBGMCU, true);
        watchdog.start(MillisDurationU32::millis(100));

        // start heartbeat
        heartbeat::spawn_after(Duration::millis(1000)).unwrap();

        // start main loop
        run::spawn().unwrap();

        (
            Shared { can },
            Local { watchdog, status_led },
            init::Monotonics(mono),
        )
    }

    #[task(shared = [], local = [watchdog])]
    fn run(cx: run::Context) {
        cx.local.watchdog.feed();

        run::spawn_after(Duration::millis(10)).unwrap();
    }

    #[task(shared = [can], local = [status_led])]
    fn heartbeat(cx: heartbeat::Context) {
        cx.local.status_led.toggle();

        if cx.local.status_led.is_set_high() {
            defmt::debug!("heartbeat!");
            heartbeat::spawn_after(Duration::millis(10)).unwrap();
        }
        else {
            heartbeat::spawn_after(Duration::millis(990)).unwrap();
        }
    }

    #[task(shared = [can], binds = CAN1_RX0)]
    fn can_receive(mut cx: can_receive::Context) {
        cx.shared.can.lock(|can| {
            loop {
                match can.receive() {
                    Ok(frame) => {
                        match frame.id() {
                            bxcan::Id::Standard(id) => {
                                defmt::println!("received standard-id message: id: {:#03x} {=[u8]:#03x}", id.as_raw(), frame.data().unwrap());
                            },
                            bxcan::Id::Extended(_) => todo!(),
                        }
                    }
                    Err(nb::Error::WouldBlock) => break,
                    Err(nb::Error::Other(_)) => {} // Ignore overrun errors.
                }
            }
        });
    }

    #[idle]
    fn idle(_: idle::Context) -> ! {
        loop {
            cortex_m::asm::nop();
        }
    }

    defmt::timestamp!("{=u64}ms", { monotonics::now().ticks() });
}

// same panicking *behavior* as `panic-probe` but doesn't print a panic message
// this prevents the panic message being printed *twice* when `defmt::panic` is invoked
#[defmt::panic_handler]
fn panic() -> ! {
    cortex_m::asm::udf()
}
