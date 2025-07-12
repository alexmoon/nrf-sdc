# `nrf-sdc`

Rust bindings for the Nordic Semiconductor nRF series SoftDevice Controller.

The SoftDevice Controller is a closed source C binary written by Nordic for their microcontrollers that provides a
standard Bluetooth HCI controller interface. It is full featured and pre qualified for bluetooth controller
certification and thus makes a valuable bluetooth stack when bindgened to Rust.

## High-level bindings

The `nrf-sdc` crate contains high-level easy-to-use Rust async/await bindings for the SoftDevice Controller.

## License

This repo contains submodules with code and libraries provided by Nordic Semiconductor and ARM. Those are subject to
their own respective licenses.

The high level bindings [nrf-sdc](nrf-sdc) and [nrf-mpsl](nrf-mpsl) are licensed under either of

- Apache License, Version 2.0 ([LICENSE-APACHE](./nrf-sdc/LICENSE-APACHE) or http://www.apache.org/licenses/LICENSE-2.0)
- MIT license ([LICENSE-MIT](./nrf-sdc/LICENSE-MIT) or http://opensource.org/licenses/MIT)

at your option.
