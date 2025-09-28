[![crates.io](https://img.shields.io/crates/v/nrf-sdc.svg)](https://crates.io/crates/nrf-sdc)
[![docs.rs](https://docs.rs/nrf-sdc/badge.svg)](https://docs.rs/nrf-sdc)

# nRF 802.15.4. Radio Driver

Rust bindings for the Nordic Semiconductor nRF series 802.15.4 C radio driver

The 802.154.4 radio driver is an closed-source C library from Nordic Semiconductor that provides a 802.15.4 protocol stack.

This crate provides high-level, easy-to-use async Rust bindings for the radio driver, which are similar in spirit to the pure Rust ones in `embassy-nrf`.

Unlike the driver in `embassy-nrf`, the C driver provides the following extra functionalities:
- Integration with the NRF MPSL layer which allows the co-existence of the BLE and 802.15.4 stacks (i.e. the BLE and the 802.15.4 stacks can run simultaneously);
- Implementation of 802.15.4 MAC-level features:
 - Automatic send of ACKs for received frames;
 - Automatic receival of ACKs for sent frames;
 - MAC filtering;
 - Enh-ACKs;
 - ... and others. See [this page](https://docs.nordicsemi.com/bundle/ncs-latest/page/nrf/protocols/thread/overview/supported_features.html) for more information.

## Example - TBD

The following example shows how to initialize and run the 802.15.4 radio driver in an application.

```rust
#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]

use embassy_executor::Spawner;
use embassy_nrf as _;
use embassy_nrf::bind_interrupts;
use embassy_nrf::rng::Rng;
use nrf_mpsl::{MultiprotocolServiceLayer, Peripherals as MpslPeripherals, raw as mpsl_raw};
use nrf_sdc::{self as sdc, raw as sdc_raw, SoftdeviceController, Peripherals as SdcPeripherals};
use static_cell::StaticCell;

bind_interrupts!(struct Irqs {
    SWI0_EGU0 => nrf_mpsl::LowPrioInterruptHandler;
    POWER_CLOCK => nrf_mpsl::ClockInterruptHandler;
    RADIO => nrf_mpsl::HighPrioInterruptHandler;
    TIMER0 => nrf_mpsl::HighPrioInterruptHandler;
    RTC0 => nrf_mpsl::HighPrioInterruptHandler;
    RNG => embassy_nrf::rng::InterruptHandler<embassy_nrf::peripherals::RNG>;
});

#[embassy_executor::main]
async fn main(spawner: Spawner) -> ! {
    let p = embassy_nrf::init(Default::default());

    // Create the clock configuration
    let lfclk_cfg = mpsl_raw::mpsl_clock_lfclk_cfg_t {
        source: mpsl_raw::MPSL_CLOCK_LF_SRC_RC as u8,
        rc_ctiv: 16,
        rc_temp_ctiv: 2,
        accuracy_ppm: mpsl_raw::MPSL_CLOCK_LF_ACCURACY_500_PPM as u16,
    };

    // On nrf52 chips, the peripherals needed by MPSL are:
    // RTC0, TIMER0, TEMP, PPI_CH19, PPI_CH30, PPI_CH31
    // The list of peripherals is different for other chips.
    let mpsl_p = MpslPeripherals::new(
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

    // On nrf52 chips, the peripherals needed by SDC are:
    // PPI_CH17, PPI_CH18, PPI_CH20..=PPI_CH29
    // The list of peripherals is different for other chips.
    let sdc_p = SdcPeripherals::new(
        p.PPI_CH17,
        p.PPI_CH18,
        p.PPI_CH20,
        p.PPI_CH21,
        p.PPI_CH22,
        p.PPI_CH23,
        p.PPI_CH24,
        p.PPI_CH25,
        p.PPI_CH26,
        p.PPI_CH27,
        p.PPI_CH28,
        p.PPI_CH29,
    );

    static RNG: StaticCell<Rng<embassy_nrf::peripherals::RNG>> = StaticCell::new();
    let mut rng = RNG.init(Rng::new(p.RNG, Irqs));

    // The minimum memory required for the SoftDevice Controller to run.
    const SDC_MEM_SIZE: usize = sdc_raw::SDC_MEM_SIZE_MIN as usize;
    static SDC_MEM: StaticCell<sdc::Mem<SDC_MEM_SIZE>> = StaticCell::new();

    // Initialize the SoftDevice Controller
    let sdc = sdc::Builder::new()
        .unwrap()
        .support_adv()
        .unwrap()
        .support_peripheral()
        .unwrap()
        .build(sdc_p, &mut rng, mpsl, SDC_MEM.init(sdc::Mem::new()))
        .unwrap();

    // Spawn the MPSL and SDC tasks
    spawner.must_spawn(mpsl_task(mpsl));
    spawner.must_spawn(sdc_task(sdc));

    // Your application logic can go here.
    loop {
        // Do something
    }
}

#[embassy_executor::task]
async fn mpsl_task(mpsl: &'static MultiprotocolServiceLayer) -> ! {
    mpsl.run().await
}

#[embassy_executor::task]
async fn sdc_task(sdc: &'static SoftdeviceController) -> ! {
    loop {
        let mut evt_buf = [0; sdc_raw::HCI_MSG_BUFFER_MAX_SIZE as usize];
        match sdc.hci_get(&mut evt_buf).await {
            Ok(_event) => {
                // Handle Bluetooth events
            }
            Err(e) => {
                // Handle errors
                core::panic!("sdc_task error: {:?}", e)
            }
        }
    }
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

This crate links against the pre-compiled SoftDevice Controller library from Nordic Semiconductor, which is subject to its own license.

## Contribution

Unless you explicitly state otherwise, any contribution intentionally submitted for inclusion in the work by you, as defined in the Apache-2.0 license, shall be dual licensed as above, without any additional terms or conditions.
