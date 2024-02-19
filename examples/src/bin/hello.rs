#![no_std]
#![no_main]

use bt_hci::param::BdAddr;
use defmt::{info, unwrap};
use embassy_executor::Spawner;
use embassy_nrf::bind_interrupts;
use embassy_nrf::gpio::{Level, Output, OutputDrive};
use embassy_time::{Duration, Timer};
use nrf_sdc::{self as sdc, mpsl, pac};
use sdc::mpsl::MultiprotocolServiceLayer;
use sdc::raw::HCI_MSG_BUFFER_MAX_SIZE;
use sdc::rng_pool::RngPool;
use static_cell::StaticCell;
use {defmt_rtt as _, panic_probe as _};

bind_interrupts!(struct Irqs {
    RNG => sdc::rng_pool::InterruptHandler;
    SWI0_EGU0 => mpsl::LowPrioInterruptHandler;
    POWER_CLOCK => mpsl::ClockInterruptHandler;
    RADIO => mpsl::HighPrioInterruptHandler;
    TIMER0 => mpsl::HighPrioInterruptHandler;
    RTC0 => mpsl::HighPrioInterruptHandler;
});

fn build_sdc<'d, const N: usize>(
    p: nrf_sdc::Peripherals<'d>,
    rng: &'d RngPool<'d>,
    mpsl: &'d MultiprotocolServiceLayer<'d>,
    mem: &'d mut sdc::Mem<N>,
) -> Result<nrf_sdc::SoftdeviceController<'d>, nrf_sdc::Error> {
    sdc::Builder::new()?.support_adv()?.build(p, rng, mpsl, mem)
}

fn bd_addr() -> BdAddr {
    unsafe {
        let ficr = &*pac::FICR::ptr();
        let high = u64::from((ficr.deviceid[1].read().bits() & 0x0000ffff) | 0x0000c000);
        let addr = high << 32 | u64::from(ficr.deviceid[0].read().bits());
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
    let lfclk_cfg = mpsl::raw::mpsl_clock_lfclk_cfg_t {
        source: mpsl::raw::MPSL_CLOCK_LF_SRC_RC as u8,
        rc_ctiv: mpsl::raw::MPSL_RECOMMENDED_RC_CTIV as u8,
        rc_temp_ctiv: mpsl::raw::MPSL_RECOMMENDED_RC_TEMP_CTIV as u8,
        accuracy_ppm: mpsl::raw::MPSL_DEFAULT_CLOCK_ACCURACY_PPM as u16,
        skip_wait_lfclk_started: mpsl::raw::MPSL_DEFAULT_SKIP_WAIT_LFCLK_STARTED != 0,
    };
    static MPSL: StaticCell<MultiprotocolServiceLayer> = StaticCell::new();
    let mpsl = MPSL.init(unwrap!(mpsl::MultiprotocolServiceLayer::new(mpsl_p, Irqs, lfclk_cfg)));
    spawner.must_spawn(mpsl_task(&*mpsl));

    let sdc_p = sdc::Peripherals::new(
        pac_p.ECB, pac_p.AAR, p.NVMC, p.PPI_CH17, p.PPI_CH18, p.PPI_CH20, p.PPI_CH21, p.PPI_CH22, p.PPI_CH23,
        p.PPI_CH24, p.PPI_CH25, p.PPI_CH26, p.PPI_CH27, p.PPI_CH28, p.PPI_CH29,
    );

    let mut pool = [0; 256];
    let rng = sdc::rng_pool::RngPool::new(p.RNG, Irqs, &mut pool, 64);

    let mut sdc_mem = sdc::Mem::<1584>::new();
    let sdc = unwrap!(build_sdc(sdc_p, &rng, mpsl, &mut sdc_mem));

    let mut hci_buf = [0; HCI_MSG_BUFFER_MAX_SIZE as usize];

    // Set the bluetooth address
    unwrap!(sdc.zephyr_write_bd_addr(bd_addr()));

    // HCI_LE_Set_Advertising_Parameters
    unwrap!(sdc.hci_cmd_put(&[
        0x06, 0x20, // opcode (OGF 8, OCF 6)
        0x0f, // param len
        0x00, 0x08, // Advertising_Interval_Min
        0x00, 0x08, // Advertising_Interval_Max
        0x02, // Advertising_Type (ADV_SCAN_IND)
        0x00, // Own_Address_Type
        0x00, // Peer_Address_Type
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // Peer_Address
        0x07, // Advertising_Channel_Map
        0x00, // Advertising_Filter_Policy
    ]));
    let kind = unwrap!(sdc.hci_get(&mut hci_buf).await);
    info!("{:?}: {:x}", kind, hci_buf);

    // HCI_LE_Set_Advertising_Data
    unwrap!(sdc.hci_cmd_put(&[
        0x08, 0x20, // opcode (OGF 8, OCF 8)
        0x20, // param len
        0x0b, // Advertising_Data_Length
        0x0a, 0x09, b'H', b'e', b'l', b'l', b'o', b'R', b'u', b's', b't', // Advertising_Data
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00,
    ]));
    let kind = unwrap!(sdc.hci_get(&mut hci_buf).await);
    info!("{:?}: {:x}", kind, hci_buf);

    // HCI_Read_BD_ADDR
    unwrap!(sdc.hci_cmd_put(&[
        0x09, 0x10, // opcode (OGF 4, OCF 9)
        0x00, // param len
    ]));
    let kind = unwrap!(sdc.hci_get(&mut hci_buf).await);
    info!("{:?}: {:x}", kind, hci_buf);

    // HCI_LE_Set_Advertising_Enable
    unwrap!(sdc.hci_cmd_put(&[
        0x0a, 0x20, // opcode (OGF 8, OCF 10)
        0x01, // param len
        0x01, // Advertising_Enable
    ]));
    let kind = unwrap!(sdc.hci_get(&mut hci_buf).await);
    info!("{:?}: {:x}", kind, hci_buf);

    let mut led = Output::new(p.P0_13, Level::Low, OutputDrive::Standard);

    loop {
        led.set_high();
        Timer::after(Duration::from_millis(300)).await;
        led.set_low();
        Timer::after(Duration::from_millis(300)).await;
    }
}
