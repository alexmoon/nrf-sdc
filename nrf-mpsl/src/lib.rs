#![no_std]

#[cfg(not(any(
    feature = "nrf52805",
    feature = "nrf52810",
    feature = "nrf52811",
    feature = "nrf52820",
    feature = "nrf52832",
    feature = "nrf52833",
    feature = "nrf52840",
)))]
compile_error!("No chip feature activated. You must activate exactly one of the following features: nrf52810, nrf52811, nrf52832, nrf52833, nrf52840");

#[cfg(any(
    all(feature = "nrf52805", feature = "nrf52810"),
    all(feature = "nrf52805", feature = "nrf52811"),
    all(feature = "nrf52805", feature = "nrf52820"),
    all(feature = "nrf52805", feature = "nrf52832"),
    all(feature = "nrf52805", feature = "nrf52833"),
    all(feature = "nrf52805", feature = "nrf52840"),
    all(feature = "nrf52810", feature = "nrf52811"),
    all(feature = "nrf52810", feature = "nrf52820"),
    all(feature = "nrf52810", feature = "nrf52832"),
    all(feature = "nrf52810", feature = "nrf52833"),
    all(feature = "nrf52810", feature = "nrf52840"),
    all(feature = "nrf52811", feature = "nrf52820"),
    all(feature = "nrf52811", feature = "nrf52832"),
    all(feature = "nrf52811", feature = "nrf52833"),
    all(feature = "nrf52811", feature = "nrf52840"),
    all(feature = "nrf52820", feature = "nrf52832"),
    all(feature = "nrf52820", feature = "nrf52833"),
    all(feature = "nrf52820", feature = "nrf52840"),
    all(feature = "nrf52832", feature = "nrf52833"),
    all(feature = "nrf52832", feature = "nrf52840"),
    all(feature = "nrf52833", feature = "nrf52840"),
))]
compile_error!("Multiple chip features activated. You must activate exactly one of the following features: nrf52810, nrf52811, nrf52832, nrf52833, nrf52840");

#[cfg(feature = "nrf52805")]
pub use nrf52805_pac as pac;
#[cfg(feature = "nrf52810")]
pub use nrf52810_pac as pac;
#[cfg(feature = "nrf52811")]
pub use nrf52811_pac as pac;
#[cfg(feature = "nrf52820")]
pub use nrf52820_pac as pac;
#[cfg(feature = "nrf52832")]
pub use nrf52832_pac as pac;
#[cfg(feature = "nrf52833")]
pub use nrf52833_pac as pac;
#[cfg(feature = "nrf52840")]
pub use nrf52840_pac as pac;
pub use nrf_mpsl_sys as raw;

// This mod MUST go first, so that the others see its macros.
pub(crate) mod fmt;

mod error;
mod hfclk;
mod mpsl;
mod temp;

pub use error::*;
pub use hfclk::*;
pub use mpsl::*;
pub use temp::*;
