[![crates.io](https://img.shields.io/crates/v/nrf-mpsl.svg)](https://crates.io/crates/nrf-mpsl)
[![docs.rs](https://docs.rs/nrf-mpsl/badge.svg)](https://docs.rs/nrf-mpsl)

# nRF Multiprotocol Service Library

Rust bindings for the Nordic Semiconductor nRF series Multiprotocol Service Layer (MPSL).

The Multiprotocol Service Layer is a closed-source C library from Nordic Semiconductor that manages radio hardware access and timing for nRF52 and nRF53 series devices. It is a prerequisite for using a SoftDevice Controller.

This crate provides high-level, easy-to-use async Rust bindings for the MPSL.

## Example

The following example shows how to initialize and run the MPSL in an application.

```rust
#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]

use embassy_executor::Spawner;
use embassy_nrf::bind_interrupts;
use nrf_mpsl::{MultiprotocolServiceLayer, Peripherals, raw};
use static_cell::StaticCell;

// This is where we register the interrupt handlers for the MPSL
bind_interrupts!(struct Irqs {
    SWI0_EGU0 => nrf_mpsl::LowPrioInterruptHandler;
    POWER_CLOCK => nrf_mpsl::ClockInterruptHandler;
    RADIO => nrf_mpsl::HighPrioInterruptHandler;
    TIMER0 => nrf_mpsl::HighPrioInterruptHandler;
    RTC0 => nrf_mpsl::HighPrioInterruptHandler;
});

#[embassy_executor::main]
async fn main(spawner: Spawner) -> ! {
    let p = embassy_nrf::init(Default::default());

    // Create the clock configuration
    let lfclk_cfg = raw::mpsl_clock_lfclk_cfg_t {
        source: raw::MPSL_CLOCK_LF_SRC_RC as u8,
        rc_ctiv: 16,
        rc_temp_ctiv: 2,
        accuracy_ppm: raw::MPSL_CLOCK_LF_ACCURACY_500_PPM as u16,
    };

    // On nrf52 chips, the peripherals needed by MPSL are:
    // RTC0, TIMER0, TEMP, PPI_CH19, PPI_CH30, PPI_CH31
    // The list of peripherals is different for other chips.
    let mpsl_p = Peripherals::new(
        p.RTC0,
        p.TIMER0,
        p.TEMP,
        p.PPI_CH19,
        p.PPI_CH30,
        p.PPI_CH31,
    );

    // Initialize the MPSL
    static MPSL: StaticCell<MultiprotocolServiceLayer> = StaticCell::new();
    let mpsl = MPSL.init(MultiprotocolServiceLayer::new(mpsl_p, Irqs, lfclk_cfg).unwrap());

    // Spawn the MPSL task
    spawner.must_spawn(mpsl_task(mpsl));

    // Your application logic can go here.
    loop {
        // Do something
    }
}

#[embassy_executor::task]
async fn mpsl_task(mpsl: &'static MultiprotocolServiceLayer) -> ! {
    mpsl.run().await
}

#[panic_handler]
fn panic(_info: &core::panic::PanicInfo) -> ! {
    loop {}
}
```

## License

The Rust code in this crate is licensed under either of

- Apache License, Version 2.0 ([LICENSE-APACHE](./LICENSE-APACHE) or http://www.apache.org/licenses/LICENSE-2.0)
- MIT license ([LICENSE-MIT](./LICENSE-MIT) or http://opensource.org/licenses/MIT)

at your option.

This crate links against the pre-compiled Multiprotocol Service Layer library from Nordic Semiconductor, which is subject to its own license.

## Contribution

Unless you explicitly state otherwise, any contribution intentionally submitted for inclusion in the work by you, as defined in the Apache-2.0 license, shall be dual licensed as above, without any additional terms or conditions.
