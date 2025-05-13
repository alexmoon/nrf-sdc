#![no_std]

pub use nrf_mpsl_sys as raw;

// This mod MUST go first, so that the others see its macros.
pub(crate) mod fmt;

mod error;
#[cfg(feature = "nrf52")]
mod flash;
mod hfclk;
mod mpsl;
mod temp;

#[cfg(feature = "critical-section-impl")]
mod critical_section_impl;

pub use error::*;
#[cfg(feature = "nrf52")]
pub use flash::*;
pub use hfclk::*;
pub use mpsl::*;
pub use temp::*;
