//! An interface for the Nordic multiprotocol service layer (MPSL).
//!
//! # Example
//!
//! ```rust,no_run
//! #![no_std]
//! #![no_main]
//! #![feature(type_alias_impl_trait)]
//!
//! use embassy_executor::Spawner;
//! use embassy_nrf::bind_interrupts;
//! use nrf_mpsl::{MultiprotocolServiceLayer, Peripherals, raw};
//! use static_cell::StaticCell;
//!
//! // This is where we register the interrupt handlers for the MPSL
//! bind_interrupts!(struct Irqs {
//!     SWI0_EGU0 => nrf_mpsl::LowPrioInterruptHandler;
//!     POWER_CLOCK => nrf_mpsl::ClockInterruptHandler;
//!     RADIO => nrf_mpsl::HighPrioInterruptHandler;
//!     TIMER0 => nrf_mpsl::HighPrioInterruptHandler;
//!     RTC0 => nrf_mpsl::HighPrioInterruptHandler;
//! });
//!
//! #[embassy_executor::main]
//! async fn main(spawner: Spawner) -> ! {
//!     let p = embassy_nrf::init(Default::default());
//!
//!     // Create the clock configuration
//!     let lfclk_cfg = raw::mpsl_clock_lfclk_cfg_t {
//!         source: raw::MPSL_CLOCK_LF_SRC_RC as u8,
//!         rc_ctiv: 16,
//!         rc_temp_ctiv: 2,
//!         accuracy_ppm: raw::MPSL_CLOCK_LF_ACCURACY_500_PPM as u16,
//!     };
//!
//!     // On nrf52 chips, the peripherals needed by MPSL are:
//!     // RTC0, TIMER0, TEMP, PPI_CH19, PPI_CH30, PPI_CH31
//!     // The list of peripherals is different for other chips.
//!     let mpsl_p = Peripherals::new(
//!         p.RTC0,
//!         p.TIMER0,
//!         p.TEMP,
//!         p.PPI_CH19,
//!         p.PPI_CH30,
//!         p.PPI_CH31,
//!     );
//!
//!     // Initialize the MPSL
//!     static MPSL: StaticCell<MultiprotocolServiceLayer> = StaticCell::new();
//!     let mpsl = MPSL.init(MultiprotocolServiceLayer::new(mpsl_p, Irqs, lfclk_cfg).unwrap());
//!
//!     // Spawn the MPSL task
//!     spawner.must_spawn(mpsl_task(mpsl));
//!
//!     // Your application logic can go here.
//!     loop {
//!         // Do something
//!     }
//! }
//!
//! #[embassy_executor::task]
//! async fn mpsl_task(mpsl: &'static MultiprotocolServiceLayer) -> ! {
//!     mpsl.run().await
//! }
//!
//! #[panic_handler]
//! fn panic(_info: &core::panic::PanicInfo) -> ! {
//!     loop {}
//! }
//! ```
#![no_std]
#![deny(missing_docs)]

/// Unsafe low-level bindings for the Nordic multiprotocol service layer.
pub use nrf_mpsl_sys as raw;

// This mod MUST go first, so that the others see its macros.
pub(crate) mod fmt;

/// Error handling.
mod error;
#[cfg(any(feature = "nrf52", feature = "nrf54l-s"))]
/// Flash operations using the timeslot API.
mod flash;
/// High-frequency clock management.
mod hfclk;
/// Multiprotocol service layer.
mod mpsl;
/// Temperature sensor access.
mod temp;

#[cfg(feature = "critical-section-impl")]
mod critical_section_impl;

pub use error::*;
#[cfg(any(feature = "nrf52", feature = "nrf54l-s"))]
pub use flash::*;
pub use hfclk::*;
pub use mpsl::*;
pub use temp::*;
