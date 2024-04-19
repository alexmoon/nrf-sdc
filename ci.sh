#!/usr/bin/env bash

set -euxo pipefail

export RUSTFLAGS=-Dwarnings

cargo clippy -p nrf-sdc-examples

cargo clippy -p nrf-mpsl-sys
cargo clippy -p nrf-mpsl-sys --features fem-simple-gpio
cargo clippy -p nrf-mpsl-sys --features fem-nrf21540-gpio
cargo clippy -p nrf-mpsl-sys --features fem-nrf21540-gpio-spi

cargo clippy -p nrf-mpsl --features nrf52805
cargo clippy -p nrf-mpsl --features nrf52810
cargo clippy -p nrf-mpsl --features nrf52811
cargo clippy -p nrf-mpsl --features nrf52820
# cargo clippy -p nrf-mpsl --features nrf52832
cargo clippy -p nrf-mpsl --features nrf52833
cargo clippy -p nrf-mpsl --features nrf52840
cargo clippy -p nrf-mpsl --features nrf52840,defmt
cargo clippy -p nrf-mpsl --features nrf52840,log
cargo clippy -p nrf-mpsl --features nrf52840,critical-section-impl
cargo clippy -p nrf-mpsl --features nrf52840,fem-simple-gpio
cargo clippy -p nrf-mpsl --features nrf52840,fem-nrf21540-gpio
cargo clippy -p nrf-mpsl --features nrf52840,fem-nrf21540-gpio-spi

cargo clippy -p nrf-sdc-sys --features peripheral
cargo clippy -p nrf-sdc-sys --features central
cargo clippy -p nrf-sdc-sys --features peripheral,central

cargo clippy -p nrf-sdc --features nrf52805,peripheral,central
cargo clippy -p nrf-sdc --features nrf52810,peripheral,central
cargo clippy -p nrf-sdc --features nrf52811,peripheral,central
cargo clippy -p nrf-sdc --features nrf52820,peripheral,central
# cargo clippy -p nrf-sdc --features nrf52832,peripheral,central
cargo clippy -p nrf-sdc --features nrf52833,peripheral,central
cargo clippy -p nrf-sdc --features nrf52840,peripheral
cargo clippy -p nrf-sdc --features nrf52840,central
cargo clippy -p nrf-sdc --features nrf52840,peripheral,central
cargo clippy -p nrf-sdc --features nrf52840,peripheral,central,defmt
cargo clippy -p nrf-sdc --features nrf52840,peripheral,central,log
