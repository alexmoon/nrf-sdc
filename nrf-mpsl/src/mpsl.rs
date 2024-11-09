use core::ffi::CStr;
use core::future::poll_fn;
use core::marker::PhantomData;
use core::mem::MaybeUninit;
use core::task::Poll;

use cortex_m::interrupt::InterruptNumber as _;
use embassy_nrf::interrupt::typelevel::{Binding, Handler, Interrupt, POWER_CLOCK, RADIO, RTC0, TIMER0};
use embassy_nrf::interrupt::Priority;
use embassy_nrf::{interrupt, peripherals, Peripheral, PeripheralRef};
use embassy_sync::waitqueue::AtomicWaker;

use crate::error::{Error, RetVal};
use crate::{hfclk, raw, temp};

static WAKER: AtomicWaker = AtomicWaker::new();

/// Struct containing all peripherals required for the MPSL to operate.
///
/// This is used to enforce at compile-time that your code doesn't use
/// these peripherals.
///
/// However, there's extra restrictions that are not enforced at compile-time
/// that you must ensure to fulfill manually:
///
/// - Do not use `cpsid/cpsie` directly for globally disabling/enabling interrupts,
///   as this can disrupt timing. Do not use the `critical-section` implementation of
///   `cortex-m`, use the one from this crate instead.
/// - Do not use `NVMC` directly as this causes the CPU to stall during flash operations.
///   Use the [`Flash`](crate::Flash) implementation from this crate instead, which
///   uses the timeslot system to schedule flash operations at times that don't disrupt radio.
/// - Do not use `RADIO` directly, except during timeslots you've allocated.
/// - Do not use `CLOCKS` directly, use the functions provided by this crate instead.
pub struct Peripherals<'d> {
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
        rtc0: impl Peripheral<P = peripherals::RTC0> + 'd,
        timer0: impl Peripheral<P = peripherals::TIMER0> + 'd,
        temp: impl Peripheral<P = peripherals::TEMP> + 'd,
        ppi_ch19: impl Peripheral<P = peripherals::PPI_CH19> + 'd,
        ppi_ch30: impl Peripheral<P = peripherals::PPI_CH30> + 'd,
        ppi_ch31: impl Peripheral<P = peripherals::PPI_CH31> + 'd,
    ) -> Self {
        Peripherals {
            rtc0: rtc0.into_ref(),
            timer0: timer0.into_ref(),
            temp: temp.into_ref(),
            ppi_ch19: ppi_ch19.into_ref(),
            ppi_ch30: ppi_ch30.into_ref(),
            ppi_ch31: ppi_ch31.into_ref(),
        }
    }
}

pub struct MultiprotocolServiceLayer<'d> {
    // Prevent Sync
    _private: PhantomData<core::cell::UnsafeCell<&'d ()>>,
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

pub struct Builder<'d> {
    // Prevent Send, Sync
    _private: PhantomData<&'d ()>,
}

impl<'d> MultiprotocolServiceLayer<'d> {
    pub fn new<T, I>(p: Peripherals<'d>, _irq: I, clock_cfg: raw::mpsl_clock_lfclk_cfg_t) -> Result<Self, Error>
    where
        T: Interrupt,
        I: Binding<T, LowPrioInterruptHandler>
            + Binding<interrupt::typelevel::RADIO, HighPrioInterruptHandler>
            + Binding<interrupt::typelevel::TIMER0, HighPrioInterruptHandler>
            + Binding<interrupt::typelevel::RTC0, HighPrioInterruptHandler>
            + Binding<interrupt::typelevel::POWER_CLOCK, ClockInterruptHandler>,
    {
        // Peripherals are used by the MPSL library, so we merely take ownership and ignore them
        let _ = p;

        T::set_priority(Priority::P4);
        T::unpend();

        let ret = unsafe { raw::mpsl_init(&clock_cfg, raw::IRQn_Type::from(T::IRQ.number()), Some(assert_handler)) };
        RetVal::from(ret).to_result()?;

        RADIO::set_priority(Priority::P0);
        RTC0::set_priority(Priority::P0);
        TIMER0::set_priority(Priority::P0);
        POWER_CLOCK::set_priority(Priority::P4);

        Ok(Self { _private: PhantomData })
    }

    pub fn with_timeslots<T, I, const SLOTS: usize>(
        p: Peripherals<'d>,
        _irq: I,
        clock_cfg: raw::mpsl_clock_lfclk_cfg_t,
        mem: &'d mut SessionMem<SLOTS>,
    ) -> Result<Self, Error>
    where
        T: Interrupt,
        I: Binding<T, LowPrioInterruptHandler>
            + Binding<interrupt::typelevel::RADIO, HighPrioInterruptHandler>
            + Binding<interrupt::typelevel::TIMER0, HighPrioInterruptHandler>
            + Binding<interrupt::typelevel::RTC0, HighPrioInterruptHandler>
            + Binding<interrupt::typelevel::POWER_CLOCK, ClockInterruptHandler>,
    {
        let me = Self::new(p, _irq, clock_cfg)?;
        let ret = unsafe { raw::mpsl_timeslot_session_count_set(mem.0.as_mut_ptr() as *mut _, SLOTS as u8) };
        RetVal::from(ret).to_result()?;
        Ok(me)
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

    pub fn ecb_block_encrypt(&self, key: &[u8; 16], cleartext: &[u8], ciphertext: &mut [u8]) -> Result<(), Error> {
        assert_eq!(cleartext.len(), 16);
        assert_eq!(ciphertext.len(), 16);
        unsafe {
            raw::mpsl_ecb_block_encrypt_extended(
                key.as_ptr(),
                cleartext.as_ptr(),
                ciphertext.as_mut_ptr(),
                // TODO: Support flags
                0,
            )
        };
        Ok(())
    }
}

#[repr(align(4))]
pub struct SessionMem<const N: usize>(MaybeUninit<[[u8; raw::MPSL_TIMESLOT_CONTEXT_SIZE as usize]; N]>);

impl<const N: usize> SessionMem<N> {
    pub fn new() -> Self {
        Self(MaybeUninit::uninit())
    }
}

impl<const N: usize> Default for SessionMem<N> {
    fn default() -> Self {
        Self::new()
    }
}

// Low priority interrupts
pub struct LowPrioInterruptHandler;
impl<T: Interrupt> Handler<T> for LowPrioInterruptHandler {
    unsafe fn on_interrupt() {
        WAKER.wake();
    }
}

pub struct ClockInterruptHandler;
impl Handler<interrupt::typelevel::POWER_CLOCK> for ClockInterruptHandler {
    unsafe fn on_interrupt() {
        raw::MPSL_IRQ_CLOCK_Handler();
    }
}

// High priority interrupts
pub struct HighPrioInterruptHandler;
impl Handler<interrupt::typelevel::RADIO> for HighPrioInterruptHandler {
    unsafe fn on_interrupt() {
        raw::MPSL_IRQ_RADIO_Handler();
    }
}

impl Handler<interrupt::typelevel::TIMER0> for HighPrioInterruptHandler {
    unsafe fn on_interrupt() {
        raw::MPSL_IRQ_TIMER0_Handler();
    }
}

impl Handler<interrupt::typelevel::RTC0> for HighPrioInterruptHandler {
    unsafe fn on_interrupt() {
        raw::MPSL_IRQ_RTC0_Handler();
    }
}
