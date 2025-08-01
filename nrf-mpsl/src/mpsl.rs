//! Main implementation of the multiprotocol service layer.
use core::ffi::CStr;
use core::future::poll_fn;
use core::marker::PhantomData;
use core::mem::MaybeUninit;
use core::task::Poll;

use cortex_m::interrupt::InterruptNumber as _;
use embassy_nrf::interrupt::typelevel::{Binding, Handler, Interrupt, CLOCK_POWER, RADIO, RTC0, TIMER0};
use embassy_nrf::interrupt::Priority;
use embassy_nrf::{interrupt, peripherals, Peri};
use embassy_sync::waitqueue::AtomicWaker;

use crate::error::{Error, RetVal};
use crate::{hfclk, raw, temp};

static WAKER: AtomicWaker = AtomicWaker::new();

/// Peripherals required for the multiprotocol service layer.
///
/// This is used to enforce at compile-time that the application does not use
/// any peripherals that are required by the MPSL.
///
/// # Panics
///
/// The following hardware restrictions must be followed. Panics may occur if they are not.
///
/// - Do not use `cpsid/cpsie` directly for globally disabling/enabling interrupts,
///   as this can disrupt timing. Do not use the `critical-section` implementation of
///   `cortex-m`, use the one from this crate instead.
/// - Do not use `NVMC` directly as this causes the CPU to stall during flash operations.
///   Use the [`Flash`](crate::Flash) implementation from this crate instead, which
///   uses the timeslot system to schedule flash operations at times that don't disrupt radio.
/// - Do not use 'ECB' directly.
/// - Do not use `RADIO` directly, except during timeslots you've allocated.
/// - Do not use `CLOCK_POWER` directly, use the functions provided by this crate instead.
pub struct Peripherals<'d> {
    /// Real-time counter 0.
    pub rtc0: Peri<'d, peripherals::RTC0>,
    /// Timer 0.
    pub timer0: Peri<'d, peripherals::TIMER0>,
    /// Timer 1.
    #[cfg(feature = "nrf53")]
    pub timer1: Peri<'d, peripherals::TIMER1>,
    /// Temperature sensor.
    pub temp: Peri<'d, peripherals::TEMP>,

    /// PPI channel 19.
    #[cfg(feature = "nrf52")]
    pub ppi_ch19: Peri<'d, peripherals::PPI_CH19>,
    /// PPI channel 30.
    #[cfg(feature = "nrf52")]
    pub ppi_ch30: Peri<'d, peripherals::PPI_CH30>,
    /// PPI channel 31.
    #[cfg(feature = "nrf52")]
    pub ppi_ch31: Peri<'d, peripherals::PPI_CH31>,
    /// PPI channel 0.
    #[cfg(feature = "nrf53")]
    pub ppi_ch0: Peri<'d, peripherals::PPI_CH0>,
    /// PPI channel 1.
    #[cfg(feature = "nrf53")]
    pub ppi_ch1: Peri<'d, peripherals::PPI_CH1>,
    /// PPI channel 2.
    #[cfg(feature = "nrf53")]
    pub ppi_ch2: Peri<'d, peripherals::PPI_CH2>,
}

impl<'d> Peripherals<'d> {
    /// Creates a new `Peripherals` instance.
    #[allow(clippy::too_many_arguments)]
    pub fn new(
        rtc0: Peri<'d, peripherals::RTC0>,
        timer0: Peri<'d, peripherals::TIMER0>,
        #[cfg(feature = "nrf53")] timer1: Peri<'d, peripherals::TIMER1>,
        temp: Peri<'d, peripherals::TEMP>,
        #[cfg(feature = "nrf52")] ppi_ch19: Peri<'d, peripherals::PPI_CH19>,
        #[cfg(feature = "nrf52")] ppi_ch30: Peri<'d, peripherals::PPI_CH30>,
        #[cfg(feature = "nrf52")] ppi_ch31: Peri<'d, peripherals::PPI_CH31>,
        #[cfg(feature = "nrf53")] ppi_ch0: Peri<'d, peripherals::PPI_CH0>,
        #[cfg(feature = "nrf53")] ppi_ch1: Peri<'d, peripherals::PPI_CH1>,
        #[cfg(feature = "nrf53")] ppi_ch2: Peri<'d, peripherals::PPI_CH2>,
    ) -> Self {
        Peripherals {
            rtc0,
            timer0,
            #[cfg(feature = "nrf53")]
            timer1,
            temp,
            #[cfg(feature = "nrf52")]
            ppi_ch19,
            #[cfg(feature = "nrf52")]
            ppi_ch30,
            #[cfg(feature = "nrf52")]
            ppi_ch31,
            #[cfg(feature = "nrf53")]
            ppi_ch0,
            #[cfg(feature = "nrf53")]
            ppi_ch1,
            #[cfg(feature = "nrf53")]
            ppi_ch2,
        }
    }
}

/// The multiprotocol service layer.
///
/// This is the main interface to the MPSL. It is responsible for initializing and running the MPSL.
pub struct MultiprotocolServiceLayer<'d> {
    // Prevent Sync
    _private: PhantomData<core::cell::UnsafeCell<&'d ()>>,
}

unsafe extern "C" fn assert_handler(file: *const core::ffi::c_char, line: u32) {
    panic!(
        "MultiprotocolServiceLayer: {}:{}",
        // SAFETY: the SDC should always give us valid utf8 strings.
        unsafe { core::str::from_utf8_unchecked(CStr::from_ptr(file).to_bytes()) },
        line
    )
}

impl Drop for MultiprotocolServiceLayer<'_> {
    fn drop(&mut self) {
        unsafe { raw::mpsl_uninit() };
    }
}

impl<'d> MultiprotocolServiceLayer<'d> {
    /// Initializes the multiprotocol service layer.
    ///
    /// This function should only be called once.
    pub fn new<T, I>(p: Peripherals<'d>, _irq: I, clock_cfg: raw::mpsl_clock_lfclk_cfg_t) -> Result<Self, Error>
    where
        T: Interrupt,
        I: Binding<T, LowPrioInterruptHandler>
            + Binding<interrupt::typelevel::RADIO, HighPrioInterruptHandler>
            + Binding<interrupt::typelevel::TIMER0, HighPrioInterruptHandler>
            + Binding<interrupt::typelevel::RTC0, HighPrioInterruptHandler>
            + Binding<interrupt::typelevel::CLOCK_POWER, ClockInterruptHandler>,
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
        CLOCK_POWER::set_priority(Priority::P4);

        Ok(Self { _private: PhantomData })
    }

    /// Initializes the multiprotocol service layer with timeslot support.
    ///
    /// This function should only be called once.
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
            + Binding<interrupt::typelevel::CLOCK_POWER, ClockInterruptHandler>,
    {
        let me = Self::new(p, _irq, clock_cfg)?;
        let ret = unsafe { raw::mpsl_timeslot_session_count_set(mem.0.as_mut_ptr() as *mut _, SLOTS as u8) };
        RetVal::from(ret).to_result()?;
        Ok(me)
    }

    /// Returns the build revision of the MPSL.
    pub fn build_revision() -> Result<[u8; raw::MPSL_BUILD_REVISION_SIZE as usize], Error> {
        let mut rev = [0; raw::MPSL_BUILD_REVISION_SIZE as usize];
        let ret = unsafe { raw::mpsl_build_revision_get(rev.as_mut_ptr()) };
        RetVal::from(ret).to_result().and(Ok(rev))
    }

    /// Runs the multiprotocol service layer.
    ///
    /// This function never returns.
    pub async fn run(&self) -> ! {
        poll_fn(|ctx| {
            WAKER.register(ctx.waker());
            unsafe { raw::mpsl_low_priority_process() };
            Poll::Pending
        })
        .await
    }

    /// Returns the temperature of the chip.
    pub fn get_temperature(&self) -> temp::Temperature {
        temp::Temperature(unsafe { raw::mpsl_temperature_get() })
    }

    /// Requests the high-frequency clock.
    ///
    /// This function returns a guard that will release the clock when dropped. An error will be returned if
    /// an `Hfclk` guard already exists.
    pub async fn request_hfclk(&self) -> Result<hfclk::Hfclk, Error> {
        hfclk::Hfclk::new()
    }

    /// Encrypts a block of data using the ECB peripheral.
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

/// Memory required for timeslot sessions.
///
/// This buffer must be provided to [`MultiprotocolServiceLayer::with_timeslots`] to enable timeslot support.
#[repr(align(4))]
pub struct SessionMem<const N: usize>(MaybeUninit<[[u8; raw::MPSL_TIMESLOT_CONTEXT_SIZE as usize]; N]>);

impl<const N: usize> SessionMem<N> {
    /// Creates a new `SessionMem` instance.
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
/// The low-priority interrupt handler.
pub struct LowPrioInterruptHandler;
impl<T: Interrupt> Handler<T> for LowPrioInterruptHandler {
    unsafe fn on_interrupt() {
        WAKER.wake();
    }
}

/// The clock interrupt handler.
pub struct ClockInterruptHandler;
impl Handler<interrupt::typelevel::CLOCK_POWER> for ClockInterruptHandler {
    unsafe fn on_interrupt() {
        raw::MPSL_IRQ_CLOCK_Handler();
    }
}

// High priority interrupts
/// The high-priority interrupt handler.
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
