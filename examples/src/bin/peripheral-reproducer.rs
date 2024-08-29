#![no_std]
#![no_main]

use defmt::info;
use bt_hci::cmd::controller_baseband::{SetEventMask, Reset};
use sdc::rng_pool::RngPool;
use bt_hci::controller::Controller;
use bt_hci::data::{AclPacket, AclPacketBoundary, AclBroadcastFlag};
use bt_hci::cmd::le::LeSetRandomAddr;
use bt_hci::cmd::link_control::Disconnect;
use bt_hci::param::DisconnectReason;
use bt_hci::cmd::le::LeSetAdvData;
use bt_hci::cmd::le::LeSetScanResponseData;
use bt_hci::cmd::le::LeSetAdvEnable;
use bt_hci::cmd::le::LeSetAdvParams;
use bt_hci::param::EventMask;
use bt_hci::param::ConnHandle;
use bt_hci::cmd::le::LeSetEventMask;
use bt_hci::param::LeEventMask;
use bt_hci::cmd::SyncCmd;
use bt_hci::event::Event;
use bt_hci::event::le::LeEvent;
use bt_hci::ControllerToHostPacket;
use bt_hci::param::BdAddr;
use defmt::unwrap;
use embassy_executor::Spawner;
use embassy_nrf::peripherals::RNG;
use embassy_nrf::gpio::{Output, Level, OutputDrive};
use embassy_nrf::{bind_interrupts, pac, rng};
use nrf_sdc::mpsl::MultiprotocolServiceLayer;
use nrf_sdc::{self as sdc, mpsl};
use static_cell::StaticCell;
use embassy_time::{Timer, Duration};
use nrf_sdc::SoftdeviceController;
use {defmt_rtt as _, panic_probe as _};

bind_interrupts!(struct Irqs {
    RNG => sdc::rng_pool::InterruptHandler;
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
    rng: &'d RngPool,
    mpsl: &'d MultiprotocolServiceLayer,
    mem: &'d mut sdc::Mem<N>,
) -> Result<nrf_sdc::SoftdeviceController<'d>, nrf_sdc::Error> {
    sdc::Builder::new()?
        .support_adv()?
        .support_ext_adv()?
        .support_peripheral()?
        .support_central()?
        .support_ext_central()?
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
        pac_p.ECB, pac_p.AAR, p.PPI_CH17, p.PPI_CH18, p.PPI_CH20, p.PPI_CH21, p.PPI_CH22, p.PPI_CH23, p.PPI_CH24,
        p.PPI_CH25, p.PPI_CH26, p.PPI_CH27, p.PPI_CH28, p.PPI_CH29,
    );

    static POOL: StaticCell<[u8; 256]> = StaticCell::new();
    let pool = POOL.init([0; 256]);
    let rng = sdc::rng_pool::RngPool::new(p.RNG, Irqs, pool, 64);

    static RNG: StaticCell<RngPool> = StaticCell::new();
    let rng = RNG.init(rng);

    static SDC_MEM: StaticCell<sdc::Mem<16920>> = StaticCell::new();
    let sdc_mem = SDC_MEM.init(sdc::Mem::new());

    let sdc = unwrap!(build_sdc(sdc_p, rng, mpsl, sdc_mem));

    static SDC: StaticCell<SoftdeviceController<'static>> = StaticCell::new();
    let sdc = SDC.init(sdc);

    unwrap!(Reset::new().exec(sdc).await);
    let address: BdAddr = BdAddr::new([0xff, 0x8f, 0x1a, 0x05, 0xe4, 0xff]);
    unwrap!(LeSetRandomAddr::new(address).exec(sdc).await);

    unwrap!(spawner.spawn(sdc_task(sdc)));

    unwrap!(
        LeSetAdvParams::new(
            bt_hci::param::Duration::from_millis(100),
            bt_hci::param::Duration::from_millis(100),
            bt_hci::param::AdvKind::AdvInd,
            bt_hci::param::AddrKind::RANDOM,
            bt_hci::param::AddrKind::PUBLIC,
            BdAddr::default(),
            bt_hci::param::AdvChannelMap::ALL,
            bt_hci::param::AdvFilterPolicy::default(),
        )
        .exec(sdc)
        .await
    );


    let adv_data = [2, 1, 6, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0];
    unwrap!(LeSetAdvData::new(3, adv_data).exec(sdc).await);

    let scan_data = [8, 7, b'T', b'r', b'o', b'u', b'b', b'l', b'e', 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0];
    unwrap!(LeSetScanResponseData::new(9, scan_data).exec(sdc).await);

    unwrap!(LeSetAdvEnable::new(true).exec(sdc).await);
    info!("enabled advertising");


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
                    info!("ACL {:?}", acl.handle());
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
                    unwrap!(Spawner::for_current_executor().await.spawn(timeout_task(sdc, Duration::from_millis(600), e.handle)));
                }
                Event::DisconnectionComplete(e) => {
                    info!("DisconnectionComplete: {:?}", e.handle);
                    let h = unwrap!(handle.take());
                    assert_eq!(h, e.handle);
                    unwrap!(LeSetAdvEnable::new(true).exec(sdc).await);
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


#[embassy_executor::task]
async fn timeout_task(sdc: &'static SoftdeviceController<'static>, duration: Duration, handle: ConnHandle) {
    let pdu = &[
        0x05,
        0x00,
        0x0D,
        0x00,

        0x14,
        0x42,
        0x0A,

        0x42,
        0x00,

        0x40,
        0x00,

        0x1b,
        0x00,

        0x17,
        0x00,

        0x00,
        0x00,
    ];
    let acl = AclPacket::new(
        handle,
        AclPacketBoundary::FirstNonFlushable,
        AclBroadcastFlag::PointToPoint,
        pdu,
    );
    unwrap!(sdc.write_acl_data(&acl).await);
    Timer::after(duration).await;
    unwrap!(Disconnect::new(handle, DisconnectReason::RemoteUserTerminatedConn).exec(sdc).await);
}
