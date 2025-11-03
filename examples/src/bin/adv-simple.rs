#![no_std]
#![no_main]

use bt_hci::cmd::le::{LeSetAdvData, LeSetAdvEnable, LeSetAdvParams};
use bt_hci::cmd::SyncCmd;
use bt_hci::param::BdAddr;
use defmt::unwrap;
use embassy_executor::Spawner;
use embassy_nrf::gpio::{Level, Output, OutputDrive};
use embassy_nrf::mode::Blocking;
use embassy_nrf::peripherals::CRACEN;
use embassy_nrf::{bind_interrupts, pac, cracen, config};
use embassy_time::{Duration, Timer};
use nrf_sdc::{self as sdc, mpsl};
use sdc::mpsl::MultiprotocolServiceLayer;
use sdc::vendor::ZephyrWriteBdAddr;
use static_cell::StaticCell;
use rand_core::SeedableRng;

use {defmt_rtt as _, panic_probe as _};

type Rng = ChaCha12Rng;

bind_interrupts!(struct Irqs {
    SWI00 => mpsl::LowPrioInterruptHandler;
    CLOCK_POWER => mpsl::ClockInterruptHandler;
    RADIO_0 => mpsl::HighPrioInterruptHandler;
    TIMER10 => mpsl::HighPrioInterruptHandler;
    GRTC_3 => mpsl::HighPrioInterruptHandler;
});

fn build_sdc<'d, const N: usize>(
    p: nrf_sdc::Peripherals<'d>,
    rng: &'d mut Rng,
    mpsl: &'d MultiprotocolServiceLayer,
    mem: &'d mut sdc::Mem<N>,
) -> Result<nrf_sdc::SoftdeviceController<'d>, nrf_sdc::Error> {
    sdc::Builder::new()?.support_adv()?.build(p, rng, mpsl, mem)
}

fn bd_addr() -> BdAddr {
    let ficr = pac::FICR;
    let high = u64::from(ficr.deviceaddr(1).read());
    let addr = high << 32 | u64::from(ficr.deviceaddr(0).read());
    let addr = addr | 0x0000_c000_0000_0000;
    BdAddr::new(unwrap!(addr.to_le_bytes()[..6].try_into()))
}

#[embassy_executor::task]
async fn mpsl_task(mpsl: &'static MultiprotocolServiceLayer<'static>) -> ! {
    mpsl.run().await
}

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    let mut config: config::Config = Default::default();
    config.clock_speed = config::ClockSpeed::CK128;
    let p = embassy_nrf::init(config);

    defmt::info!("Starting");
    let mpsl_p = mpsl::Peripherals::new(
        p.GRTC,
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

    let r = embassy_nrf::pac::GRTC_S;
    r.syscounter(0).active().write(|w| {
        w.set_active(true);
    });
    r.mode().write(|w| {
        use embassy_nrf::pac::grtc::vals::Autoen;
        w.set_syscounteren(true);
        w.set_autoen(Autoen::CPU_ACTIVE);
    });

    r.tasks_clear().write_value(1);
    r.tasks_start().write_value(1);


    Timer::after(Duration::from_millis(3000)).await;
    defmt::info!("Clock: {:?}", defmt::Debug2Format(&embassy_nrf::pac::OSCILLATORS_S.pll().currentfreq().read().currentfreq()));

    defmt::info!("MPSL init");
    let mpsl = MPSL.init(unwrap!(mpsl::MultiprotocolServiceLayer::new(mpsl_p, Irqs, lfclk_cfg)));
    spawner.spawn(unwrap!(mpsl_task(&*mpsl)));

    /*
    defmt::info!("SDC create");
    let sdc_p = sdc::Peripherals::new(
        p.PPI00_CH1,
        p.PPI00_CH3,
        p.PPI10_CH1,
        p.PPI10_CH2,
        p.PPI10_CH3,
        p.PPI10_CH4,
        p.PPI10_CH5,
        p.PPI10_CH6,
        p.PPI10_CH7,
        p.PPI10_CH8,
        p.PPI10_CH9,
        p.PPI10_CH10,
        p.PPI10_CH11,
        p.PPIB00_CH1,
        p.PPIB00_CH2,
        p.PPIB00_CH3,
        p.PPIB10_CH1,
        p.PPIB10_CH2,
        p.PPIB10_CH3,
    );

    defmt::info!("CRACEN start");
//    let mut rng = Rng::from_seed([0; 32]);
    //let mut rng = cracen::Cracen::new_blocking(p.CRACEN);

    defmt::info!("SDC mem ..");
    let mut sdc_mem = sdc::Mem::<4096>::new();
    defmt::info!("SDC build..");
    let sdc = unwrap!(build_sdc(sdc_p, &mut rng, mpsl, &mut sdc_mem));
    defmt::info!("SDC built");

    // Set the bluetooth address
    unwrap!(ZephyrWriteBdAddr::new(bd_addr()).exec(&sdc).await);

    unwrap!(
        LeSetAdvParams::new(
            bt_hci::param::Duration::from_millis(1280),
            bt_hci::param::Duration::from_millis(1280),
            bt_hci::param::AdvKind::AdvScanInd,
            bt_hci::param::AddrKind::PUBLIC,
            bt_hci::param::AddrKind::PUBLIC,
            BdAddr::default(),
            bt_hci::param::AdvChannelMap::ALL,
            bt_hci::param::AdvFilterPolicy::default(),
        )
        .exec(&sdc)
        .await
    );

    let adv_data = &[0x0a, 0x09, b'H', b'e', b'l', b'l', b'o', b'R', b'u', b's', b't'];
    let mut data = [0; 31];
    data[..adv_data.len()].copy_from_slice(adv_data);
    unwrap!(LeSetAdvData::new(adv_data.len() as u8, data).exec(&sdc).await);

    unwrap!(LeSetAdvEnable::new(true).exec(&sdc).await);
    */

    defmt::info!("Led prepare");
    let mut led = Output::new(p.P1_10, Level::Low, OutputDrive::Standard);

    loop {
        defmt::info!("blink");
        led.set_high();
        Timer::after(Duration::from_millis(300)).await;
        led.set_low();
        Timer::after(Duration::from_millis(300)).await;
    }
}
use chacha20::ChaChaCore;
use chacha20::cipher::consts::U6;
use chacha20::cipher::{KeyIvInit, StreamCipherCore};
use rand_core::block::{BlockRng, BlockRngCore, CryptoBlockRng};
use rand_core::{CryptoRng, RngCore};

const BUFFER_SIZE: usize = 16;

pub struct ChaCha12Rng {
    pub core: BlockRng<ChaCha12Core>,
}
pub struct ChaCha12Core(ChaChaCore<U6>);

impl SeedableRng for ChaCha12Rng {
    type Seed = [u8; 32];

    #[inline]
    fn from_seed(seed: Self::Seed) -> Self {
        Self {
            core: BlockRng::new(ChaCha12Core::from_seed(seed.into())),
        }
    }
}
impl BlockRngCore for ChaCha12Core {
    type Item = u32;
    type Results = [u32; BUFFER_SIZE];

    #[inline]
    fn generate(&mut self, r: &mut Self::Results) {
        fn u32_to_u8<const A: usize, const B: usize>(a: &mut [u32; A]) -> &mut [u8; B] {
            const { core::assert!(A * 4 == B) };
            unsafe { &mut *a.as_mut_ptr().cast() }
        }
        self.0.write_keystream_block(u32_to_u8::<16, 64>(r).into());
    }
}

impl CryptoBlockRng for ChaCha12Core {}
impl CryptoRng for ChaCha12Rng {}

impl SeedableRng for ChaCha12Core {
    type Seed = [u8; 32];

    #[inline]
    fn from_seed(seed: Self::Seed) -> Self {
        Self(ChaChaCore::<U6>::new((&seed).into(), (&[0u8; 12]).into()))
    }
}

impl RngCore for ChaCha12Rng {
    #[inline]
    fn next_u32(&mut self) -> u32 {
        self.core.next_u32()
    }
    #[inline]
    fn next_u64(&mut self) -> u64 {
        self.core.next_u64()
    }
    #[inline]
    fn fill_bytes(&mut self, dest: &mut [u8]) {
        self.core.fill_bytes(dest)
    }
}

impl From<ChaCha12Core> for ChaCha12Rng {
    fn from(core: ChaCha12Core) -> Self {
        ChaCha12Rng {
            core: BlockRng::new(core),
        }
    }
}
