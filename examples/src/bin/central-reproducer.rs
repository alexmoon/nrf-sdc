#![no_std]
#![no_main]

use bt_hci::cmd::le::LeAddDeviceToFilterAcceptList;
use bt_hci::cmd::le::LeClearFilterAcceptList;
use bt_hci::cmd::le::LeSetRandomAddr;
use bt_hci::cmd::controller_baseband::{SetEventMask, Reset};
use bt_hci::cmd::le::LeSetAdvData;
use bt_hci::cmd::le::LeSetAdvEnable;
use bt_hci::cmd::le::LeCreateConn;
use bt_hci::event::{Event, le::LeEvent};
use bt_hci::ControllerToHostPacket;
use bt_hci::controller::Controller;
use bt_hci::param::ConnHandle;
use defmt::info;
use bt_hci::cmd::le::LeSetAdvParams;
use bt_hci::cmd::le::LeSetEventMask;
use bt_hci::cmd::SyncCmd;
use bt_hci::cmd::AsyncCmd;
use bt_hci::param::AddrKind;
use bt_hci::param::EventMask;
use bt_hci::param::LeEventMask;
use bt_hci::param::BdAddr;
use defmt::unwrap;
use embassy_executor::Spawner;
use embassy_nrf::peripherals::RNG;
use embassy_nrf::gpio::{Output, Level, OutputDrive};
use embassy_nrf::{bind_interrupts, pac, rng};
use nrf_sdc::mpsl::MultiprotocolServiceLayer;
use nrf_sdc::{self as sdc, mpsl};
use sdc::SoftdeviceController;
use static_cell::StaticCell;
use embassy_time::{Timer, Duration};
use sdc::vendor::ZephyrWriteBdAddr;
use {defmt_rtt as _, panic_probe as _};

bind_interrupts!(struct Irqs {
    RNG => rng::InterruptHandler<RNG>;
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

/// How many outgoing L2CAP buffers per link
const L2CAP_TXQ: u8 = 10;

/// How many incoming L2CAP buffers per link
const L2CAP_RXQ: u8 = 10;

/// Size of L2CAP packets
const L2CAP_MTU: usize = 27;

fn build_sdc<'d, const N: usize>(
    p: nrf_sdc::Peripherals<'d>,
    rng: &'d mut rng::Rng<RNG>,
    mpsl: &'d MultiprotocolServiceLayer,
    mem: &'d mut sdc::Mem<N>,
) -> Result<nrf_sdc::SoftdeviceController<'d>, nrf_sdc::Error> {
    sdc::Builder::new()?
        .support_adv()?
        .support_peripheral()?
        .support_central()?
        .peripheral_count(4)?
        .central_count(4)?
        .buffer_cfg(L2CAP_MTU as u8, L2CAP_MTU as u8, L2CAP_TXQ, L2CAP_RXQ)?
        .build(p, rng, mpsl, mem)
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
        source: mpsl::raw::MPSL_CLOCK_LF_SRC_XTAL as u8,
        rc_ctiv: 0,
        rc_temp_ctiv: 0,
        accuracy_ppm: 7,
        skip_wait_lfclk_started: mpsl::raw::MPSL_DEFAULT_SKIP_WAIT_LFCLK_STARTED != 0,
    };
    static MPSL: StaticCell<MultiprotocolServiceLayer> = StaticCell::new();
    let mpsl = MPSL.init(unwrap!(mpsl::MultiprotocolServiceLayer::new(mpsl_p, Irqs, lfclk_cfg)));
    spawner.must_spawn(mpsl_task(&*mpsl));

    let sdc_p = sdc::Peripherals::new(
        pac_p.ECB, pac_p.AAR, p.PPI_CH17, p.PPI_CH18, p.PPI_CH20, p.PPI_CH21, p.PPI_CH22, p.PPI_CH23, p.PPI_CH24,
        p.PPI_CH25, p.PPI_CH26, p.PPI_CH27, p.PPI_CH28, p.PPI_CH29,
    );

    let mut rng = rng::Rng::new(p.RNG, Irqs);

    static RNG: StaticCell<rng::Rng<'static, RNG>> = StaticCell::new();
    let rng = RNG.init(rng);
    static SDC_MEM: StaticCell<sdc::Mem<16384>> = StaticCell::new();
    let sdc_mem = SDC_MEM.init(sdc::Mem::new());

    let sdc = unwrap!(build_sdc(sdc_p, rng, mpsl, sdc_mem));

    static SDC: StaticCell<SoftdeviceController<'static>> = StaticCell::new();
    let sdc = SDC.init(sdc);

    unwrap!(Reset::new().exec(sdc).await);

    let address: BdAddr = BdAddr::new([0xff, 0x9f, 0x1b, 0x05, 0xe4, 0xff]);
    unwrap!(LeSetRandomAddr::new(address).exec(sdc).await);
    unwrap!(spawner.spawn(sdc_task(sdc)));

    unwrap!(LeClearFilterAcceptList::new().exec(sdc).await);

    let target: BdAddr = BdAddr::new([0xff, 0x8f, 0x1a, 0x05, 0xe4, 0xff]);
    unwrap!(LeAddDeviceToFilterAcceptList::new(AddrKind::RANDOM, target).exec(sdc).await);


    Timer::after(Duration::from_secs(1)).await;

    unwrap!(LeCreateConn::new(
        Duration::from_secs(1).into(),
        Duration::from_secs(1).into(),
        true,
        AddrKind::PUBLIC,
        BdAddr::default(),
        AddrKind::RANDOM,
        Duration::from_millis(80).into(),
        Duration::from_millis(80).into(),
        0,
        Duration::from_secs(8).into(),
        Duration::from_secs(0).into(),
        Duration::from_secs(0).into(),
    ).exec(sdc).await);

    let mut led = Output::new(p.P0_13, Level::Low, OutputDrive::Standard);

    loop {
        led.set_high();
        Timer::after(Duration::from_millis(300)).await;
        led.set_low();
        Timer::after(Duration::from_millis(300)).await;
    }
}

#[embassy_executor::task]
async fn sdc_task(sdc: &'static SoftdeviceController<'static>) {
    unwrap!(SetEventMask::new(
                    EventMask::new()
                        .enable_le_meta(true)
                        .enable_conn_request(true)
                        .enable_conn_complete(true)
                        .enable_disconnection_complete(true),
                )
                .exec(sdc).await);

    unwrap!(LeSetEventMask::new(
                LeEventMask::new()
                    .enable_le_conn_complete(true)
                )
                .exec(sdc).await);

    let mut rx = [0u8; 259];
    let mut handle: Option<ConnHandle> = None;
    loop {
        //info!("Waiting for event");
        let result = sdc.read(&mut rx).await;
        match result {
            Ok(ControllerToHostPacket::Acl(acl)) => {
                if let Some(h) = &handle {
                    assert_eq!(*h, acl.handle());
                } else {
                    info!("ACL on {:?}", acl.handle());
                    assert!(false);
                }
            }
            Ok(ControllerToHostPacket::Event(event)) => match event {
                Event::Le(LeEvent::LeConnectionComplete(e)) => {
                    info!("LeConnectionComplete {:?}", e.handle);
                    assert!(handle.is_none());
                    handle.replace(e.handle);
                }
                Event::DisconnectionComplete(e) => {
                    info!("DisconnectionComplete: {:?}", e.handle);
                    let h = unwrap!(handle.take());
                    assert_eq!(h, e.handle);
                    unwrap!(LeCreateConn::new(
                        Duration::from_secs(1).into(),
                        Duration::from_secs(1).into(),
                        true,
                        AddrKind::PUBLIC,
                        BdAddr::default(),
                        AddrKind::RANDOM,
                        Duration::from_millis(80).into(),
                        Duration::from_millis(80).into(),
                        0,
                        Duration::from_secs(8).into(),
                        Duration::from_secs(0).into(),
                        Duration::from_secs(0).into(),
                    ).exec(sdc).await);
                }
                _ => {
                    info!("Ignored event");
                }
            }
            _ => {
                info!("Ignored result");
            }
        }
    }
}
