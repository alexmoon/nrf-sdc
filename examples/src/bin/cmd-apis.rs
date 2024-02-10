#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]

use defmt::unwrap;
use embassy_executor::Spawner;
use embassy_executor::_export::StaticCell;
use embassy_nrf::gpio::{Level, Output, OutputDrive};
use embassy_nrf::interrupt::{self, take};
use embassy_time::{Duration, Timer};
use nrf_sdc::{self as sdc, hci, mpsl, pac};
use sdc::hci::BdAddr;
use sdc::mpsl::MultiprotocolServiceLayer;
use sdc::rng_pool::RngPool;
use {defmt_rtt as _, panic_probe as _};

fn build_sdc<'d>(
    p: nrf_sdc::Peripherals<'d>,
    rng: &'d RngPool<'d>,
    mpsl: &'d MultiprotocolServiceLayer<'d>,
    mem: &'d mut [u8],
) -> Result<nrf_sdc::SoftdeviceController<'d>, nrf_sdc::Error> {
    sdc::Builder::new()?.support_adv()?.build(p, rng, mpsl, mem)
}

fn bd_addr() -> BdAddr {
    unsafe {
        let ficr = &*pac::FICR::ptr();
        let high = u64::from(ficr.deviceid[1].read().bits());
        let addr = high << 32 | u64::from(ficr.deviceid[0].read().bits());
        let addr = addr | 0x0000_c000_0000_0000;
        BdAddr::new(unwrap!(addr.to_le_bytes()[..6].try_into()))
    }
}

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
    let mpsl_irqs = mpsl::Interrupts::new(
        take!(RADIO),
        take!(RTC0),
        take!(TIMER0),
        take!(POWER_CLOCK),
        take!(SWI1_EGU1),
    );
    let lfclk_cfg = mpsl::raw::mpsl_clock_lfclk_cfg_t {
        source: mpsl::raw::MPSL_CLOCK_LF_SRC_RC as u8,
        rc_ctiv: mpsl::raw::MPSL_RECOMMENDED_RC_CTIV as u8,
        rc_temp_ctiv: mpsl::raw::MPSL_RECOMMENDED_RC_TEMP_CTIV as u8,
        accuracy_ppm: mpsl::raw::MPSL_DEFAULT_CLOCK_ACCURACY_PPM as u16,
        skip_wait_lfclk_started: mpsl::raw::MPSL_DEFAULT_SKIP_WAIT_LFCLK_STARTED != 0,
    };
    static MPSL: StaticCell<MultiprotocolServiceLayer> = StaticCell::new();
    let mpsl = MPSL.init(unwrap!(mpsl::MultiprotocolServiceLayer::new(
        mpsl_p, mpsl_irqs, lfclk_cfg
    )));
    spawner.must_spawn(mpsl_task(&*mpsl));

    let sdc_p = sdc::Peripherals::new(
        pac_p.ECB, pac_p.AAR, p.NVMC, p.PPI_CH17, p.PPI_CH18, p.PPI_CH20, p.PPI_CH21, p.PPI_CH22, p.PPI_CH23,
        p.PPI_CH24, p.PPI_CH25, p.PPI_CH26, p.PPI_CH27, p.PPI_CH28, p.PPI_CH29,
    );

    let irq = take!(RNG);
    let mut pool = [0; 256];
    let rng = sdc::rng_pool::RngPool::new(p.RNG, irq, &mut pool, 64);

    let mut sdc_mem = [0; 1584];
    let sdc = unwrap!(build_sdc(sdc_p, &rng, mpsl, &mut sdc_mem));

    // Set the bluetooth address
    unwrap!(sdc.zephyr_write_bd_addr(bd_addr()));

    unwrap!(sdc.le_set_adv_params(
        hci::Duration::from_millis(1280),
        hci::Duration::from_millis(1280),
        hci::AdvertisingType::ADV_SCAN_IND,
        hci::AddressType::default(),
        hci::AddressType::default(),
        hci::BdAddr::default(),
        hci::AdvertisingChannelMap::default(),
        hci::AdvertisingFilterPolicy::default()
    ));
    unwrap!(sdc.le_set_adv_data(&[0x0a, 0x09, b'H', b'e', b'l', b'l', b'o', b'R', b'u', b's', b't']));
    unwrap!(sdc.le_set_adv_enable(true));

    let mut led = Output::new(p.P0_13, Level::Low, OutputDrive::Standard);

    loop {
        led.set_high();
        Timer::after(Duration::from_millis(300)).await;
        led.set_low();
        Timer::after(Duration::from_millis(300)).await;
    }
}
