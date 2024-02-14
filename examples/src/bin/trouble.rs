#![no_std]
#![no_main]

use trouble_host::{
    ad_structure::{create_advertising_data, AdStructure, BR_EDR_NOT_SUPPORTED, LE_GENERAL_DISCOVERABLE},
    att::Uuid,
    attribute::Attribute,
    attribute_server::{AttributeServer, PRIMARY_SERVICE_UUID16, CHARACTERISTIC_UUID16},
    no_rng::NoRng,
    Ble, driver::{HciDriver, HciMessageType},
};
use defmt::{info, unwrap, trace};
use embassy_executor::Spawner;
use embassy_nrf::{bind_interrupts, pac};
use embassy_time::{Duration, Instant, Timer};
use sdc::rng_pool::RngPool;
use static_cell::StaticCell;
use nrf_sdc::{self as sdc, mpsl, mpsl::MultiprotocolServiceLayer, Error as SdcError};
use {defmt_rtt as _, panic_probe as _};

bind_interrupts!(struct Irqs {
    RNG => nrf_sdc::rng_pool::InterruptHandler;
    SWI0_EGU0 => nrf_sdc::mpsl::LowPrioInterruptHandler;
    POWER_CLOCK => nrf_sdc::mpsl::ClockInterruptHandler;
    RADIO => nrf_sdc::mpsl::HighPrioInterruptHandler;
    TIMER0 => nrf_sdc::mpsl::HighPrioInterruptHandler;
    RTC0 => nrf_sdc::mpsl::HighPrioInterruptHandler;
});

#[embassy_executor::task]
async fn mpsl_task(mpsl: &'static MultiprotocolServiceLayer<'static>) -> ! {
    mpsl.run().await
}

fn bd_addr() -> [u8; 6] {
    unsafe {
        let ficr = &*pac::FICR::ptr();
        let high = u64::from((ficr.deviceid[1].read().bits() & 0x0000ffff) | 0x0000c000);
        let addr = high << 32 | u64::from(ficr.deviceid[0].read().bits());
        unwrap!(addr.to_le_bytes()[..6].try_into())
    }
}

fn build_sdc<'d>(
    p: nrf_sdc::Peripherals<'d>,
    rng: &'d RngPool<'d>,
    mpsl: &'d MultiprotocolServiceLayer<'d>,
    mem: &'d mut [u8],
) -> Result<nrf_sdc::SoftdeviceController<'d>, nrf_sdc::Error> {
    sdc::Builder::new()?.support_adv()?.support_peripheral()?.peripheral_count(1)?.build(p, rng, mpsl, mem)
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

    let mut sdc_mem = [0; 1584];
    let sdc = unwrap!(build_sdc(sdc_p, &rng, mpsl, &mut sdc_mem));

    let bd_addr = bd_addr();
    let ret =
        unsafe { sdc::raw::sdc_hci_cmd_vs_zephyr_write_bd_addr(&sdc::raw::sdc_hci_cmd_vs_zephyr_write_bd_addr_t { bd_addr }) };

    trace!("Write BLE addr: {}", ret);
    let connector = SdcHci::new(sdc);
    Timer::after(Duration::from_millis(2000)).await;
    let mut ble = Ble::new(connector, || Instant::now().as_millis());

    unwrap!(ble.init().await);
    unwrap!(ble.cmd_set_le_advertising_parameters().await);

    unwrap!(ble.cmd_set_le_advertising_data(
        create_advertising_data(&[
            AdStructure::Flags(LE_GENERAL_DISCOVERABLE | BR_EDR_NOT_SUPPORTED),
            AdStructure::ServiceUuids16(&[Uuid::Uuid16(0x180f)]),
            AdStructure::CompleteLocalName("Trouble"),
        ])
        .unwrap(),
    ).await);
    unwrap!(ble.cmd_set_le_advertise_enable(true).await);
    info!("started advertising");


    let service_data: [u8; 2] = 0x180f_u16.to_le_bytes();
    let mut service_data_ref = &service_data;
    let char_data: [u8; 5] = [0x02, 0x03, 0x00, 0x19, 0x2a];
    let mut char_data_ref = &char_data;

    let data = [4; 1];
    let mut data_ref = &data;
    let mut attributes = [
        Attribute::new(PRIMARY_SERVICE_UUID16, &mut service_data_ref),
        Attribute::new(CHARACTERISTIC_UUID16, &mut char_data_ref),
        Attribute::new(Uuid::Uuid16(0x2a19), &mut data_ref),
    ];
    let mut r = NoRng;
    let mut server = AttributeServer::new(&mut ble, &mut attributes, &mut r);
    info!("running attribute server");
    unwrap!(server.run(&mut || {
        futures::future::pending()
    }).await);
}

pub struct SdcHci<'d> {
    sdc: nrf_sdc::SoftdeviceController<'d>,
}

impl<'d> SdcHci<'d> {
    pub fn new(
        sdc: nrf_sdc::SoftdeviceController<'d>
    ) -> Self {
        Self {
            sdc,
        }
    }
}

#[derive(Debug, defmt::Format)]
pub struct DriverError(SdcError);

impl trouble_host::driver::Error for DriverError {
    fn kind(&self) -> trouble_host::driver::ErrorKind {
        trouble_host::driver::ErrorKind::Other
    }
}

impl From<SdcError> for DriverError {
    fn from(e: SdcError) -> DriverError {
        DriverError(e)
    }
}

impl<'d> HciDriver for SdcHci<'d> {
    type Error = DriverError;
    async fn read(&mut self, buf: &mut [u8]) -> Result<HciMessageType, Self::Error> {
        use sdc::hci::MsgKind;
        Ok(match self.sdc.hci_get(buf).await? {
            MsgKind::Data => HciMessageType::Data,
            MsgKind::Event  => HciMessageType::Event,
        })
    }

    async fn write(&mut self, kind: HciMessageType, buf: &[u8]) -> Result<(), Self::Error> {
        match kind {
            HciMessageType::Data => {
                let r = self.sdc.hci_data_put(buf);
                Ok(r?)
            }
            HciMessageType::Command => {
                let r = self.sdc.hci_cmd_put(buf);
                Ok(r?)
            }
            _ => panic!("unsupported"),
        }
    }
}
