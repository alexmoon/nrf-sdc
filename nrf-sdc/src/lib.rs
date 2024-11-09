#![no_std]

pub use mpsl::{Error, RetVal};
pub use {nrf_mpsl as mpsl, nrf_sdc_sys as raw};

// This mod MUST go first, so that the others see its macros.
pub(crate) mod fmt;

mod sdc;

pub use sdc::*;
