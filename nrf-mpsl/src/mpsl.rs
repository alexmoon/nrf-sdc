//! Main implementation of the multiprotocol service layer.
use core::ffi::CStr;
use core::future::poll_fn;
use core::marker::PhantomData;
use core::mem::MaybeUninit;
use core::task::Poll;

use cfg_if::cfg_if;
use cortex_m::interrupt::InterruptNumber as _;
use embassy_nrf::interrupt::typelevel::{Binding, Handler, Interrupt, CLOCK_POWER};
use embassy_nrf::interrupt::Priority;
use embassy_nrf::{interrupt, peripherals, Peri};
use embassy_sync::waitqueue::AtomicWaker;

use crate::error::{Error, RetVal};
use crate::{hfclk, raw, temp};

static WAKER: AtomicWaker = AtomicWaker::new();

cfg_if! {
    if #[cfg(feature = "nrf52")] {
        use embassy_nrf::interrupt::typelevel::TIMER0;
        use embassy_nrf::interrupt::typelevel::RTC0;
        use embassy_nrf::interrupt::typelevel::RADIO;

        type RadioIrq = RADIO;
        type RtcIrq = RTC0;
        type TimerIrq = TIMER0;

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
            /// Temperature sensor.
            pub temp: Peri<'d, peripherals::TEMP>,

            /// PPI channel 19.
            pub ppi_ch19: Peri<'d, peripherals::PPI_CH19>,
            /// PPI channel 30.
            pub ppi_ch30: Peri<'d, peripherals::PPI_CH30>,
            /// PPI channel 31.
            pub ppi_ch31: Peri<'d, peripherals::PPI_CH31>,
        }

        impl<'d> Peripherals<'d> {
            /// Creates a new `Peripherals` instance.
            #[allow(clippy::too_many_arguments)]
            pub fn new(
                rtc0: Peri<'d, peripherals::RTC0>,
                timer0: Peri<'d, peripherals::TIMER0>,
                temp: Peri<'d, peripherals::TEMP>,
                ppi_ch19: Peri<'d, peripherals::PPI_CH19>,
                ppi_ch30: Peri<'d, peripherals::PPI_CH30>,
                ppi_ch31: Peri<'d, peripherals::PPI_CH31>,
            ) -> Self {
                Peripherals {
                    rtc0,
                    timer0,
                    temp,
                    ppi_ch19,
                    ppi_ch30,
                    ppi_ch31,
                }
            }
        }


    } else if #[cfg(feature = "nrf53")] {
        use embassy_nrf::interrupt::typelevel::TIMER0;
        use embassy_nrf::interrupt::typelevel::RADIO;
        use embassy_nrf::interrupt::typelevel::RTC0;

        type RadioIrq = RADIO;
        type RtcIrq = RTC0;
        type TimerIrq = TIMER0;
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
            pub timer1: Peri<'d, peripherals::TIMER1>,
            /// Temperature sensor.
            pub temp: Peri<'d, peripherals::TEMP>,

            /// PPI channel 0.
            pub ppi_ch0: Peri<'d, peripherals::PPI_CH0>,
            /// PPI channel 1.
            pub ppi_ch1: Peri<'d, peripherals::PPI_CH1>,
            /// PPI channel 2.
            pub ppi_ch2: Peri<'d, peripherals::PPI_CH2>,
        }

        impl<'d> Peripherals<'d> {
            /// Creates a new `Peripherals` instance.
            #[allow(clippy::too_many_arguments)]
            pub fn new(
                rtc0: Peri<'d, peripherals::RTC0>,
                timer0: Peri<'d, peripherals::TIMER0>,
                timer1: Peri<'d, peripherals::TIMER1>,
                temp: Peri<'d, peripherals::TEMP>,
                ppi_ch0: Peri<'d, peripherals::PPI_CH0>,
                ppi_ch1: Peri<'d, peripherals::PPI_CH1>,
                ppi_ch2: Peri<'d, peripherals::PPI_CH2>,
            ) -> Self {
                Peripherals {
                    rtc0,
                    timer0,
                    timer1,
                    temp,
                    ppi_ch0,
                    ppi_ch1,
                    ppi_ch2,
                }
            }
        }
    } else if #[cfg(feature = "_nrf54l")] {
        use embassy_nrf::interrupt::typelevel::TIMER10;
        use embassy_nrf::interrupt::typelevel::GRTC_3;
        use embassy_nrf::interrupt::typelevel::RADIO_0;

        type RadioIrq = RADIO_0;
        type RtcIrq = GRTC_3;
        type TimerIrq = TIMER10;

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
            /// Real-time counter channel 7.
            pub grtc_ch7: Peri<'d, peripherals::GRTC_CH7>,
            /// Real-time counter channel 8.
            pub grtc_ch8: Peri<'d, peripherals::GRTC_CH8>,
            /// Real-time counter channel 9.
            pub grtc_ch9: Peri<'d, peripherals::GRTC_CH9>,
            /// Real-time counter channel 10.
            pub grtc_ch10: Peri<'d, peripherals::GRTC_CH10>,
            /// Real-time counter channel 11.
            pub grtc_ch11: Peri<'d, peripherals::GRTC_CH11>,
            /// Timer 0.
            pub timer10: Peri<'d, peripherals::TIMER10>,
            /// Timer 1.
            pub timer20: Peri<'d, peripherals::TIMER20>,
            /// Temperature sensor
            pub temp: Peri<'d, peripherals::TEMP>,

            /// PPIC10 channel 0.
            pub ppi10_ch0: Peri<'d, peripherals::PPI10_CH0>,
            /// PPIC20 channel 1.
            pub ppi20_ch1: Peri<'d, peripherals::PPI20_CH1>,
            /// PPIB11 channel 0
            pub ppib11_ch0: Peri<'d, peripherals::PPIB11_CH0>,
            /// PPIB21 channel 0
            pub ppib21_ch0: Peri<'d, peripherals::PPIB21_CH0>,
        }

        impl<'d> Peripherals<'d> {
            /// Creates a new `Peripherals` instance.
            #[allow(clippy::too_many_arguments)]
            pub fn new(
                grtc_ch7: Peri<'d, peripherals::GRTC_CH7>,
                grtc_ch8: Peri<'d, peripherals::GRTC_CH8>,
                grtc_ch9: Peri<'d, peripherals::GRTC_CH9>,
                grtc_ch10: Peri<'d, peripherals::GRTC_CH10>,
                grtc_ch11: Peri<'d, peripherals::GRTC_CH11>,
                timer10: Peri<'d, peripherals::TIMER10>,
                timer20: Peri<'d, peripherals::TIMER20>,
                temp: Peri<'d, peripherals::TEMP>,
                ppi10_ch0: Peri<'d, peripherals::PPI10_CH0>,
                ppi20_ch1: Peri<'d, peripherals::PPI20_CH1>,
                ppib11_ch0: Peri<'d, peripherals::PPIB11_CH0>,
                ppib21_ch0: Peri<'d, peripherals::PPIB21_CH0>,
            ) -> Self {
                Peripherals {
                    grtc_ch7,
                    grtc_ch8,
                    grtc_ch9,
                    grtc_ch10,
                    grtc_ch11,
                    timer10,
                    timer20,
                    temp,
                    ppi10_ch0,
                    ppi20_ch1,
                    ppib11_ch0,
                    ppib21_ch0,
                }
            }
        }
    }
}

/// High priority interrupt handler.
pub struct HighPrioInterruptHandler;

impl Handler<TimerIrq> for HighPrioInterruptHandler {
    unsafe fn on_interrupt() {
        raw::MPSL_IRQ_TIMER0_Handler();
    }
}

impl Handler<RadioIrq> for HighPrioInterruptHandler {
    unsafe fn on_interrupt() {
        raw::MPSL_IRQ_RADIO_Handler();
    }
}

impl Handler<RtcIrq> for HighPrioInterruptHandler {
    unsafe fn on_interrupt() {
        raw::MPSL_IRQ_RTC0_Handler();
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

#[cfg(feature = "_nrf54l")]
#[no_mangle]
unsafe extern "C" fn mpsl_constlat_request_callback() {
    let p = embassy_nrf::pac::POWER;
    p.tasks_constlat().write_value(1);
}

#[cfg(feature = "_nrf54l")]
#[no_mangle]
unsafe extern "C" fn mpsl_lowpower_request_callback() {
    let p = embassy_nrf::pac::POWER;
    p.tasks_lowpwr().write_value(1);
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
            + Binding<RadioIrq, HighPrioInterruptHandler>
            + Binding<TimerIrq, HighPrioInterruptHandler>
            + Binding<RtcIrq, HighPrioInterruptHandler>
            + Binding<interrupt::typelevel::CLOCK_POWER, ClockInterruptHandler>,
    {
        // Peripherals are used by the MPSL library, so we merely take ownership and ignore them
        let _ = p;

        // GRTC must be started and configured
        #[cfg(feature = "_nrf54l")]
        {
            use embassy_nrf::pac;

            let freq = pac::OSCILLATORS.pll().currentfreq().read().currentfreq();
            assert!(freq == pac::oscillators::vals::Currentfreq::CK128M);

            let r = pac::GRTC;
            // Start syscounter if not started
            if !r.mode().read().syscounteren() {
                r.mode().write(|w| {
                    w.set_syscounteren(true);
                });
                r.tasks_clear().write_value(1);
                r.tasks_start().write_value(1);
            }
        }

        T::set_priority(Priority::P4);
        T::unpend();

        let ret = unsafe { raw::mpsl_init(&clock_cfg, raw::IRQn_Type::from(T::IRQ.number()), Some(assert_handler)) };
        RetVal::from(ret).to_result()?;

        RadioIrq::set_priority(Priority::P0);
        RtcIrq::set_priority(Priority::P0);
        TimerIrq::set_priority(Priority::P0);

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
            + Binding<RadioIrq, HighPrioInterruptHandler>
            + Binding<TimerIrq, HighPrioInterruptHandler>
            + Binding<RtcIrq, HighPrioInterruptHandler>
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
