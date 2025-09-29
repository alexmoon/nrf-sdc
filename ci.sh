#!/usr/bin/env bash

set -euxo pipefail

export RUSTFLAGS=-Dwarnings

cargo clippy -p nrf-sdc-examples --features nrf52832
cargo clippy -p nrf-sdc-examples --features nrf52840

cargo clippy -p nrf-mpsl-sys --features nrf52
cargo clippy -p nrf-mpsl-sys --features nrf52,fem-simple-gpio
cargo clippy -p nrf-mpsl-sys --features nrf52,fem-nrf21540-gpio
cargo clippy -p nrf-mpsl-sys --features nrf52,fem-nrf21540-gpio-spi
cargo clippy -p nrf-mpsl-sys --features nrf53 --target thumbv8m.main-none-eabi
cargo clippy -p nrf-mpsl-sys --features nrf53,fem-simple-gpio --target thumbv8m.main-none-eabi
cargo clippy -p nrf-mpsl-sys --features nrf53,fem-nrf21540-gpio --target thumbv8m.main-none-eabi
cargo clippy -p nrf-mpsl-sys --features nrf53,fem-nrf21540-gpio-spi --target thumbv8m.main-none-eabi
cargo clippy -p nrf-mpsl-sys --features nrf54l-ns --target thumbv8m.main-none-eabihf
cargo clippy -p nrf-mpsl-sys --features nrf54l-ns,fem-simple-gpio --target thumbv8m.main-none-eabihf
cargo clippy -p nrf-mpsl-sys --features nrf54l-ns,fem-nrf21540-gpio --target thumbv8m.main-none-eabihf
cargo clippy -p nrf-mpsl-sys --features nrf54l-ns,fem-nrf21540-gpio-spi --target thumbv8m.main-none-eabihf
cargo clippy -p nrf-mpsl-sys --features nrf54l-s --target thumbv8m.main-none-eabihf
cargo clippy -p nrf-mpsl-sys --features nrf54l-s,fem-simple-gpio --target thumbv8m.main-none-eabihf
cargo clippy -p nrf-mpsl-sys --features nrf54l-s,fem-nrf21540-gpio --target thumbv8m.main-none-eabihf
cargo clippy -p nrf-mpsl-sys --features nrf54l-s,fem-nrf21540-gpio-spi --target thumbv8m.main-none-eabihf
cargo clippy -p nrf-mpsl-sys --features nrf54h --target thumbv8m.main-none-eabihf
cargo clippy -p nrf-mpsl-sys --features nrf54h,fem-simple-gpio --target thumbv8m.main-none-eabihf
cargo clippy -p nrf-mpsl-sys --features nrf54h,fem-nrf21540-gpio --target thumbv8m.main-none-eabihf
cargo clippy -p nrf-mpsl-sys --features nrf54h,fem-nrf21540-gpio-spi --target thumbv8m.main-none-eabihf

cargo clippy -p nrf-mpsl --features nrf52805
cargo clippy -p nrf-mpsl --features nrf52810
cargo clippy -p nrf-mpsl --features nrf52811
cargo clippy -p nrf-mpsl --features nrf52820
cargo clippy -p nrf-mpsl --features nrf52832
cargo clippy -p nrf-mpsl --features nrf52833
cargo clippy -p nrf-mpsl --features nrf52840
cargo clippy -p nrf-mpsl --features nrf5340-net --target thumbv8m.main-none-eabi
cargo clippy -p nrf-mpsl --features nrf52840,defmt
cargo clippy -p nrf-mpsl --features nrf52840,log
cargo clippy -p nrf-mpsl --features nrf52840,critical-section-impl
cargo clippy -p nrf-mpsl --features nrf52840,fem-simple-gpio
cargo clippy -p nrf-mpsl --features nrf52840,fem-nrf21540-gpio
cargo clippy -p nrf-mpsl --features nrf52840,fem-nrf21540-gpio-spi

cargo clippy -p nrf-sdc-sys --features nrf52,peripheral
cargo clippy -p nrf-sdc-sys --features nrf52,central
cargo clippy -p nrf-sdc-sys --features nrf52,peripheral,central
cargo clippy -p nrf-sdc-sys --features nrf53,peripheral --target thumbv8m.main-none-eabi
cargo clippy -p nrf-sdc-sys --features nrf53,central --target thumbv8m.main-none-eabi
cargo clippy -p nrf-sdc-sys --features nrf53,peripheral,central --target thumbv8m.main-none-eabi
cargo clippy -p nrf-sdc-sys --features nrf54l-ns,peripheral --target thumbv8m.main-none-eabihf
cargo clippy -p nrf-sdc-sys --features nrf54l-ns,central --target thumbv8m.main-none-eabihf
cargo clippy -p nrf-sdc-sys --features nrf54l-ns,peripheral,central --target thumbv8m.main-none-eabihf
cargo clippy -p nrf-sdc-sys --features nrf54l-s,peripheral --target thumbv8m.main-none-eabihf
cargo clippy -p nrf-sdc-sys --features nrf54l-s,central --target thumbv8m.main-none-eabihf
cargo clippy -p nrf-sdc-sys --features nrf54l-s,peripheral,central --target thumbv8m.main-none-eabihf
cargo clippy -p nrf-sdc-sys --features nrf54h,peripheral --target thumbv8m.main-none-eabihf
cargo clippy -p nrf-sdc-sys --features nrf54h,central --target thumbv8m.main-none-eabihf
cargo clippy -p nrf-sdc-sys --features nrf54h,peripheral,central --target thumbv8m.main-none-eabihf

cargo clippy -p nrf-sdc --features nrf52805,peripheral,central
cargo clippy -p nrf-sdc --features nrf52810,peripheral,central
cargo clippy -p nrf-sdc --features nrf52811,peripheral,central
cargo clippy -p nrf-sdc --features nrf52820,peripheral,central
cargo clippy -p nrf-sdc --features nrf52832,peripheral,central
cargo clippy -p nrf-sdc --features nrf52833,peripheral,central
cargo clippy -p nrf-sdc --features nrf52840,peripheral,central
cargo clippy -p nrf-sdc --features nrf5340-net,peripheral,central --target thumbv8m.main-none-eabi
cargo clippy -p nrf-sdc --features nrf52840,peripheral
cargo clippy -p nrf-sdc --features nrf52840,central
cargo clippy -p nrf-sdc --features nrf52840,peripheral,central,defmt
cargo clippy -p nrf-sdc --features nrf52840,peripheral,central,log

cargo clippy -p nrf-802154-sys --features nrf52
cargo clippy -p nrf-802154-sys --features nrf53 --target thumbv8m.main-none-eabi
cargo clippy -p nrf-802154-sys --features nrf54l-ns --target thumbv8m.main-none-eabihf
cargo clippy -p nrf-802154-sys --features nrf54l-s --target thumbv8m.main-none-eabihf
cargo clippy -p nrf-802154-sys --features nrf54h --target thumbv8m.main-none-eabihf

cargo clippy -p nrf-802154 --features nrf52805
cargo clippy -p nrf-802154 --features nrf52810
cargo clippy -p nrf-802154 --features nrf52811
cargo clippy -p nrf-802154 --features nrf52820
cargo clippy -p nrf-802154 --features nrf52832
cargo clippy -p nrf-802154 --features nrf52833
cargo clippy -p nrf-802154 --features nrf52840
cargo clippy -p nrf-802154 --features nrf5340-net --target thumbv8m.main-none-eabi
cargo clippy -p nrf-802154 --features nrf52840,defmt
