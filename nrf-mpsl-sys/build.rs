//! Build Script for nrf-mpsl-sys
//!
//! Calls out to bindgen to generate a Rust crate from the Nordic header
//! files.

use std::env;
use std::path::{Path, PathBuf};

use bindgen::callbacks::ParseCallbacks;

#[derive(Debug)]
struct Callback;

impl ParseCallbacks for Callback {
    fn process_comment(&self, comment: &str) -> Option<String> {
        Some(doxygen_rs::transform(
            &comment
                .replace('[', "\\[")
                .replace("@sa @ref", "@ref")
                .replace("mpsl_fem_event_type_t::", ""),
        ))
    }
}

struct Target {
    target: String,
    cpu: &'static str,
    float_abi: &'static str,
    chip: &'static str,
}

impl Target {
    fn new(target: String) -> Self {
        let (cpu, float_abi, chip) = match target.as_str() {
            "thumbv7em-none-eabihf" => ("cortex-m4", "hard", "NRF52840_XXAA"),
            "thumbv7em-none-eabi" => ("cortex-m4", "soft", "NRF52840_XXAA"),
            "thumbv8m.main-none-eabi" => ("cortex-m33", "soft", "NRF5340_XXAA"),
            _ => panic!("Unsupported target: {:?}", target),
        };

        Self {
            target,
            cpu,
            float_abi,
            chip,
        }
    }
}

fn bindgen(target: &Target) -> bindgen::Builder {
    bindgen::Builder::default()
        .use_core()
        .size_t_is_usize(true)
        .clang_arg(format!("--target={}", target.target))
        .clang_arg(format!("-mcpu={}", target.cpu))
        .clang_arg(format!("-mfloat-abi={}", target.float_abi))
        .clang_arg("-mthumb")
        .clang_arg("-I./include")
        .clang_arg("-I./third_party/arm/CMSIS_5/CMSIS/Core/Include")
        .clang_arg("-I./third_party/nordic/nrfx/mdk")
        .clang_arg("-I./third_party/nordic/nrfxlib/mpsl/include")
        .clang_arg("-I./third_party/nordic/nrfxlib/mpsl/fem/include")
        .clang_arg(format!("-D{}", target.chip))
        .allowlist_function("mpsl_.*")
        .allowlist_function("MPSL_.*")
        .allowlist_type("mpsl_.*")
        .allowlist_type("MPSL_.*")
        .allowlist_var("MPSL_.*")
        .allowlist_var("NRF_E.*")
        .allowlist_var("UINT8_MAX")
        .prepend_enum_name(false)
        .rustfmt_bindings(true)
        .parse_callbacks(Box::new(Callback))
}

fn main() {
    let target = Target::new(env::var("TARGET").unwrap());

    let (fem_lib, fem_includes): (Option<&str>, Option<&[&str]>) = match (
        env::var_os("CARGO_FEATURE_FEM_SIMPLE_GPIO"),
        env::var_os("CARGO_FEATURE_FEM_NRF21540_GPIO"),
        env::var_os("CARGO_FEATURE_FEM_NRF21540_GPIO_SPI"),
    ) {
        (None, None, None) => (None, None),
        (Some(_), None, None) => (
            Some("simple_gpio"),
            Some(&[
                "./third_party/nordic/nrfxlib/mpsl/fem/include/mpsl_fem_config_common.h",
                "./third_party/nordic/nrfxlib/mpsl/fem/include/mpsl_fem_init.h",
                "./third_party/nordic/nrfxlib/mpsl/fem/include/mpsl_fem_power_model.h",
                "./third_party/nordic/nrfxlib/mpsl/fem/include/mpsl_fem_types.h",
                "./third_party/nordic/nrfxlib/mpsl/fem/include/protocol/mpsl_fem_protocol_api.h",
                "./third_party/nordic/nrfxlib/mpsl/fem/simple_gpio/include/mpsl_fem_config_simple_gpio.h",
            ]),
        ),
        (None, Some(_), None) => (
            Some("nrf21540_gpio"),
            Some(&[
                "./third_party/nordic/nrfxlib/mpsl/fem/include/mpsl_fem_config_common.h",
                "./third_party/nordic/nrfxlib/mpsl/fem/include/mpsl_fem_init.h",
                "./third_party/nordic/nrfxlib/mpsl/fem/include/mpsl_fem_power_model.h",
                "./third_party/nordic/nrfxlib/mpsl/fem/include/mpsl_fem_types.h",
                "./third_party/nordic/nrfxlib/mpsl/fem/include/mpsl_fem_config_nrf21540_common.h",
                "./third_party/nordic/nrfxlib/mpsl/fem/include/protocol/mpsl_fem_protocol_api.h",
                "./third_party/nordic/nrfxlib/mpsl/fem/nrf21540_gpio/include/mpsl_fem_config_nrf21540_gpio.h",
            ]),
        ),
        (None, None, Some(_)) => (Some("nrf21540_gpio_spi"),
            Some(&[
                "./third_party/nordic/nrfxlib/mpsl/fem/include/mpsl_fem_config_common.h",
                "./third_party/nordic/nrfxlib/mpsl/fem/include/mpsl_fem_init.h",
                "./third_party/nordic/nrfxlib/mpsl/fem/include/mpsl_fem_power_model.h",
                "./third_party/nordic/nrfxlib/mpsl/fem/include/mpsl_fem_types.h",
                "./third_party/nordic/nrfxlib/mpsl/fem/include/mpsl_fem_config_nrf21540_common.h",
                "./third_party/nordic/nrfxlib/mpsl/fem/include/protocol/mpsl_fem_protocol_api.h",
                "./third_party/nordic/nrfxlib/mpsl/fem/nrf21540_gpio_spi/include/mpsl_fem_config_nrf21540_gpio_spi.h",
                "./third_party/nordic/nrfxlib/mpsl/fem/nrf21540_gpio_spi/include/mpsl_fem_nrf21540_power_model_builtin.h",
            ]),
        ),
        _ => panic!("Only one front-end module feature may be enabled"),
    };

    let mut builder = bindgen(&target)
        .header("./include/stdlib.h")
        .header("./third_party/nordic/nrfxlib/mpsl/include/mpsl.h")
        .header("./third_party/nordic/nrfxlib/mpsl/include/mpsl_clock.h")
        .header("./third_party/nordic/nrfxlib/mpsl/include/mpsl_coex.h")
        .header("./third_party/nordic/nrfxlib/mpsl/include/mpsl_cx_abstract_interface.h")
        .header("./third_party/nordic/nrfxlib/mpsl/include/mpsl_radio_notification.h")
        .header("./third_party/nordic/nrfxlib/mpsl/include/mpsl_temp.h")
        .header("./third_party/nordic/nrfxlib/mpsl/include/mpsl_timeslot.h")
        .header("./third_party/nordic/nrfxlib/mpsl/include/mpsl_tx_power.h")
        .header("./third_party/nordic/nrfxlib/mpsl/include/nrf_errno.h")
        .header("./third_party/nordic/nrfxlib/mpsl/include/protocol/mpsl_cx_protocol_api.h")
        .header("./third_party/nordic/nrfxlib/mpsl/include/protocol/mpsl_dppi_protocol_api.h");

    if let Some(fem_includes) = fem_includes {
        for include in fem_includes.iter() {
            builder = builder.header(*include);
        }
    }

    builder
        .generate()
        .expect("Unable to generate bindings")
        .write_to_file(PathBuf::from(env::var_os("OUT_DIR").unwrap()).join("bindings.rs"))
        .expect("Couldn't write bindgen output");

    fn lib_path<P: AsRef<Path>>(dir: P, target: &Target) -> PathBuf {
        let mut path = PathBuf::from(env::var_os("CARGO_MANIFEST_DIR").unwrap());
        path.push("third_party/nordic/nrfxlib/mpsl");
        path.push(dir);
        path.push("lib");
        path.push(target.cpu);
        path.push(format!("{}-float", target.float_abi));
        path
    }

    let mpsl_lib_path = lib_path(".", &target);
    let fem_common_lib_path = lib_path("fem/common", &target);

    println!("cargo:rustc-link-search={}", mpsl_lib_path.to_str().unwrap());
    println!("cargo:rustc-link-search={}", fem_common_lib_path.to_str().unwrap());
    println!("cargo:rustc-link-lib=static=mpsl");
    println!("cargo:rustc-link-lib=static=mpsl_fem_common");

    if let Some(fem) = fem_lib {
        let fem_lib_path = lib_path(format!("fem/{fem}"), &target);
        println!("cargo:rustc-link-search={}", fem_lib_path.to_str().unwrap());
        println!("cargo:rustc-link-lib=static=mpsl_fem_{}", fem);
    }
}
