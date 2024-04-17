#![no_std]
#![no_main]

use defmt::{info, unwrap};
use embassy_executor::Spawner;
use embassy_nrf::bind_interrupts;
use futures::pin_mut;
use mpsl::{pac, Flash, MultiprotocolServiceLayer};
use nrf_sdc::mpsl;
use static_cell::StaticCell;
use {defmt_rtt as _, panic_probe as _};

bind_interrupts!(struct Irqs {
    SWI0_EGU0 => mpsl::LowPrioInterruptHandler;
    POWER_CLOCK => mpsl::ClockInterruptHandler;
    RADIO => mpsl::HighPrioInterruptHandler;
    TIMER0 => mpsl::HighPrioInterruptHandler;
    RTC0 => mpsl::HighPrioInterruptHandler;
});

#[embassy_executor::task]
async fn mpsl_task(mpsl: &'static MultiprotocolServiceLayer<'static>) -> ! {
    mpsl.run().await
}

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    let p = embassy_nrf::init(Default::default());
    let pac_p = pac::Peripherals::take().unwrap();

    let mpsl_p = mpsl::Peripherals::new(
        pac_p.CLOCK,
        pac_p.RADIO,
        p.RTC0,
        p.TIMER0,
        p.TEMP,
        p.PPI_CH19,
        p.PPI_CH30,
        p.PPI_CH31,
    );
    let lfclk_cfg = mpsl::raw::mpsl_clock_lfclk_cfg_t {
        source: mpsl::raw::MPSL_CLOCK_LF_SRC_RC as u8,
        rc_ctiv: mpsl::raw::MPSL_RECOMMENDED_RC_CTIV as u8,
        rc_temp_ctiv: mpsl::raw::MPSL_RECOMMENDED_RC_TEMP_CTIV as u8,
        accuracy_ppm: mpsl::raw::MPSL_DEFAULT_CLOCK_ACCURACY_PPM as u16,
        skip_wait_lfclk_started: mpsl::raw::MPSL_DEFAULT_SKIP_WAIT_LFCLK_STARTED != 0,
    };
    static MPSL: StaticCell<MultiprotocolServiceLayer> = StaticCell::new();
    static SESSION_MEM: StaticCell<mpsl::SessionMem<1>> = StaticCell::new();

    let mpsl = MPSL.init(unwrap!(mpsl::MultiprotocolServiceLayer::with_timeslots(
        mpsl_p,
        Irqs,
        lfclk_cfg,
        SESSION_MEM.init(mpsl::SessionMem::new())
    )));
    spawner.must_spawn(mpsl_task(&*mpsl));

    let f = Flash::take(mpsl, p.NVMC);
    pin_mut!(f);

    info!("starting erase");
    unwrap!(f.as_mut().erase(0x80000, 0x82000).await);
    info!("erased!");

    let mut buf = [0; 4096];
    for offset in (0x80000..0x82000).step_by(4096) {
        info!("starting read");
        unwrap!(f.as_mut().read(offset, &mut buf));
        info!("read done!");
        for b in buf.iter() {
            assert_eq!(*b, 0xff);
        }
    }

    info!("matched!");

    info!("starting write");
    for offset in (0x80000..0x82000).step_by(4) {
        unwrap!(f.as_mut().write(offset, &[1, 2, 3, 4]).await);
    }
    info!("write done!");

    for offset in (0x80000..0x82000).step_by(4) {
        let mut buf = [0; 4];
        info!("starting read");
        unwrap!(f.as_mut().read(offset, &mut buf));
        info!("read done!");
        assert_eq!(&[1, 2, 3, 4], &buf[..]);
    }
    info!("matched!");
}
