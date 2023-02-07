use core::ffi::CStr;
use core::future::poll_fn;
use core::marker::PhantomData;
use core::task::Poll;

use embassy_nrf::interrupt::{self, Interrupt, InterruptExt, Priority};
use embassy_nrf::{peripherals, Peripheral, PeripheralRef};
use embassy_sync::waitqueue::AtomicWaker;

use crate::error::{Error, RetVal};
use crate::{hfclk, pac, raw, temp};

static WAKER: AtomicWaker = AtomicWaker::new();

pub struct Peripherals<'d> {
    pub clock: pac::CLOCK,
    pub radio: pac::RADIO,
    pub rtc0: PeripheralRef<'d, peripherals::RTC0>,
    pub timer0: PeripheralRef<'d, peripherals::TIMER0>,
    pub temp: PeripheralRef<'d, peripherals::TEMP>,

    pub ppi_ch19: PeripheralRef<'d, peripherals::PPI_CH19>,
    pub ppi_ch30: PeripheralRef<'d, peripherals::PPI_CH30>,
    pub ppi_ch31: PeripheralRef<'d, peripherals::PPI_CH31>,
}

impl<'d> Peripherals<'d> {
    #[allow(clippy::too_many_arguments)]
    pub fn new(
        clock: pac::CLOCK,
        radio: pac::RADIO,
        rtc0: impl Peripheral<P = peripherals::RTC0> + 'd,
        timer0: impl Peripheral<P = peripherals::TIMER0> + 'd,
        temp: impl Peripheral<P = peripherals::TEMP> + 'd,
        ppi_ch19: impl Peripheral<P = peripherals::PPI_CH19> + 'd,
        ppi_ch30: impl Peripheral<P = peripherals::PPI_CH30> + 'd,
        ppi_ch31: impl Peripheral<P = peripherals::PPI_CH31> + 'd,
    ) -> Self {
        Peripherals {
            clock,
            radio,
            rtc0: rtc0.into_ref(),
            timer0: timer0.into_ref(),
            temp: temp.into_ref(),
            ppi_ch19: ppi_ch19.into_ref(),
            ppi_ch30: ppi_ch30.into_ref(),
            ppi_ch31: ppi_ch31.into_ref(),
        }
    }
}

pub struct Interrupts<'d, I: Interrupt> {
    pub radio: PeripheralRef<'d, interrupt::RADIO>,
    pub rtc0: PeripheralRef<'d, interrupt::RTC0>,
    pub timer0: PeripheralRef<'d, interrupt::TIMER0>,
    pub power_clock: PeripheralRef<'d, interrupt::POWER_CLOCK>,
    pub low_priority: PeripheralRef<'d, I>,
}

impl<'d, I: Interrupt> Interrupts<'d, I> {
    pub fn new(
        radio: impl Peripheral<P = interrupt::RADIO> + 'd,
        rtc0: impl Peripheral<P = interrupt::RTC0> + 'd,
        timer0: impl Peripheral<P = interrupt::TIMER0> + 'd,
        power_clock: impl Peripheral<P = interrupt::POWER_CLOCK> + 'd,
        low_priority: impl Peripheral<P = I> + 'd,
    ) -> Self {
        Interrupts {
            radio: radio.into_ref(),
            rtc0: rtc0.into_ref(),
            timer0: timer0.into_ref(),
            power_clock: power_clock.into_ref(),
            low_priority: low_priority.into_ref(),
        }
    }
}

pub struct MultiprotocolServiceLayer<'d> {
    // Prevent Send, Sync
    _private: PhantomData<&'d *mut ()>,
}

unsafe extern "C" fn assert_handler(file: *const core::ffi::c_char, line: u32) {
    panic!(
        "MultiprotocolServiceLayer: {}:{}",
        CStr::from_ptr(file).to_str().unwrap_or("bad filename"),
        line
    )
}

impl<'d> Drop for MultiprotocolServiceLayer<'d> {
    fn drop(&mut self) {
        unsafe { raw::mpsl_uninit() };
    }
}

impl<'d> MultiprotocolServiceLayer<'d> {
    pub fn new<I: Interrupt>(
        p: Peripherals<'d>,
        irqs: Interrupts<'d, I>,
        clock_cfg: raw::mpsl_clock_lfclk_cfg_t,
    ) -> Result<Self, Error> {
        assert!(
            cortex_m::peripheral::SCB::vect_active() == cortex_m::peripheral::scb::VectActive::ThreadMode,
            "MultiprotocolServiceLayer must be initialized from thread mode"
        );

        // Peripherals are used by the MPSL library, so we merely take ownership and ignore them
        let _ = p;

        irqs.low_priority.set_priority(Priority::P4);
        irqs.low_priority.set_handler(Self::low_priority_isr);
        irqs.low_priority.unpend();

        let ret = unsafe {
            raw::mpsl_init(
                &clock_cfg,
                raw::IRQn_Type::from(irqs.low_priority.number()),
                Some(assert_handler),
            )
        };
        RetVal::from(ret).to_result()?;

        irqs.radio.set_priority(Priority::P0);
        irqs.rtc0.set_priority(Priority::P0);
        irqs.timer0.set_priority(Priority::P0);
        irqs.power_clock.set_priority(Priority::P4);

        irqs.radio.set_handler(Self::radio_isr);
        irqs.rtc0.set_handler(Self::rtc0_isr);
        irqs.timer0.set_handler(Self::timer0_isr);
        irqs.power_clock.set_handler(Self::power_clock_isr);

        Ok(MultiprotocolServiceLayer { _private: PhantomData })
    }

    pub fn build_revision() -> Result<[u8; raw::MPSL_BUILD_REVISION_SIZE as usize], Error> {
        let mut rev = [0; raw::MPSL_BUILD_REVISION_SIZE as usize];
        let ret = unsafe { raw::mpsl_build_revision_get(rev.as_mut_ptr()) };
        RetVal::from(ret).to_result().and(Ok(rev))
    }

    pub async fn run(&self) -> ! {
        poll_fn(|ctx| {
            WAKER.register(ctx.waker());
            unsafe { raw::mpsl_low_priority_process() };
            Poll::Pending
        })
        .await
    }

    pub fn get_temperature(&self) -> temp::Temperature {
        temp::Temperature(unsafe { raw::mpsl_temperature_get() })
    }

    pub async fn request_hfclk(&self) -> Result<hfclk::Hfclk, Error> {
        hfclk::Hfclk::new()
    }

    fn low_priority_isr(_ctx: *mut ()) {
        WAKER.wake();
    }

    fn radio_isr(_ctx: *mut ()) {
        unsafe { raw::MPSL_IRQ_RADIO_Handler() }
    }

    fn timer0_isr(_ctx: *mut ()) {
        unsafe { raw::MPSL_IRQ_TIMER0_Handler() }
    }

    fn rtc0_isr(_ctx: *mut ()) {
        unsafe { raw::MPSL_IRQ_RTC0_Handler() }
    }

    fn power_clock_isr(_ctx: *mut ()) {
        unsafe { raw::MPSL_IRQ_CLOCK_Handler() }
    }
}
