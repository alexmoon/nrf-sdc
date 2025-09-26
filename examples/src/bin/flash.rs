#![no_std]
#![no_main]

use defmt::{info, unwrap};
use embassy_executor::Spawner;
use embassy_nrf::{bind_interrupts, config};
use futures::pin_mut;
use mpsl::{Flash, MultiprotocolServiceLayer};
use nrf_sdc::mpsl;
use static_cell::StaticCell;
use {defmt_rtt as _, panic_probe as _};

#[cfg(feature = "nrf52")]
bind_interrupts!(struct Irqs {
   EGU0_SWI0 => mpsl::LowPrioInterruptHandler;
   CLOCK_POWER => mpsl::ClockInterruptHandler;
   RADIO => mpsl::HighPrioInterruptHandler;
   TIMER0 => mpsl::HighPrioInterruptHandler;
   RTC0 => mpsl::HighPrioInterruptHandler;
});

#[cfg(feature = "nrf54l")]
bind_interrupts!(struct Irqs {
    SWI00 => mpsl::LowPrioInterruptHandler;
    CLOCK_POWER => mpsl::ClockInterruptHandler;
    RADIO_0 => mpsl::HighPrioInterruptHandler;
    TIMER10 => mpsl::HighPrioInterruptHandler;
    GRTC_3 => mpsl::HighPrioInterruptHandler;
});

#[embassy_executor::task]
async fn mpsl_task(mpsl: &'static MultiprotocolServiceLayer<'static>) -> ! {
    mpsl.run().await
}

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    let mut config: config::Config = Default::default();
    #[cfg(feature = "nrf54l")]
    {
        config.clock_speed = config::ClockSpeed::CK128;
    }
    config.hfclk_source = config::HfclkSource::ExternalXtal;
    config.lfclk_source = config::LfclkSource::ExternalXtal;
    let p = embassy_nrf::init(config);

    #[cfg(feature = "nrf52")]
    let mpsl_p = mpsl::Peripherals::new(p.RTC0, p.TIMER0, p.TEMP, p.PPI_CH19, p.PPI_CH30, p.PPI_CH31);

    #[cfg(feature = "nrf54l")]
    let mpsl_p = mpsl::Peripherals::new(
        p.GRTC_CH7,
        p.GRTC_CH8,
        p.GRTC_CH9,
        p.GRTC_CH10,
        p.GRTC_CH11,
        p.TIMER10,
        p.TIMER20,
        p.TEMP,
        p.PPI10_CH0,
        p.PPI20_CH1,
        p.PPIB11_CH0,
        p.PPIB21_CH0,
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
    spawner.spawn(unwrap!(mpsl_task(&*mpsl)));

    #[cfg(feature = "nrf52")]
    let f = Flash::take(mpsl, p.NVMC);
    #[cfg(feature = "nrf54l")]
    let f = Flash::take(mpsl, p.RRAMC);
    pin_mut!(f);

    #[cfg(feature = "nrf52")]
    {
        const PAGE_SIZE: usize = 4096;

        #[cfg(feature = "nrf52832")]
        const ERASE_START: u32 = 0x60000;

        #[cfg(feature = "nrf52840")]
        const ERASE_START: u32 = 0x80000;

        const ERASE_STOP: u32 = ERASE_START + 0x2000;

        info!("starting erase");
        unwrap!(f.as_mut().erase(ERASE_START, ERASE_STOP).await);
        info!("erased!");

        info!("checking erase");
        let mut buf = [0; PAGE_SIZE];
        for offset in (ERASE_START..ERASE_STOP).step_by(PAGE_SIZE) {
            unwrap!(f.as_mut().read(offset, &mut buf));
            for b in buf.iter() {
                assert_eq!(*b, 0xff);
            }
        }
        info!("matched!");

        info!("starting write");
        for offset in (ERASE_START..ERASE_STOP).step_by(4) {
            unwrap!(f.as_mut().write(offset, &[1, 2, 3, 4]).await);
        }
        info!("write done!");

        info!("checking write");
        for offset in (ERASE_START..ERASE_STOP).step_by(4) {
            let mut buf = [0; 4];
            unwrap!(f.as_mut().read(offset, &mut buf));
            assert_eq!(&[1, 2, 3, 4], &buf[..]);
        }
        info!("matched!");
    }

    #[cfg(feature = "nrf54l")]
    {
        const START: u32 = 0x80000;
        const STEP: usize = 16;
        const LINES: usize = 100;
        const STOP: u32 = START + (LINES * STEP) as u32;

        info!("starting erase");
        unwrap!(f.as_mut().erase(START, STOP).await);
        info!("erased!");

        info!("checking erase");
        const ERASE_LINE: [u8; STEP] = [0xFF; STEP];
        for offset in (START..STOP).step_by(STEP) {
            let mut buf = [0; STEP];
            unwrap!(f.as_mut().read(offset, &mut buf));
            assert_eq!(&ERASE_LINE, &buf[..]);
        }
        info!("matched!");

        info!("starting write");
        const WRITE_LINE: [u8; STEP] = [
            0x0, 0x1, 0x2, 0x3, 0x4, 0x5, 0x6, 0x7, 0x8, 0x9, 0xa, 0xb, 0xc, 0xd, 0xe, 0xf,
        ];
        for offset in (START..STOP).step_by(STEP) {
            unwrap!(f.as_mut().write(offset, &WRITE_LINE).await);
        }
        info!("write done!");

        info!("checking write");
        for offset in (START..STOP).step_by(STEP) {
            let mut buf = [0; STEP];
            unwrap!(f.as_mut().read(offset, &mut buf));
            assert_eq!(&WRITE_LINE, &buf[..]);
        }
        info!("matched!");
    }

    info!("Done!");
}
