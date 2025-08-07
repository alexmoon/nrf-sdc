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

    fn enum_variant_behavior(
        &self,
        _enum_name: Option<&str>,
        original_variant_name: &str,
        _variant_value: bindgen::callbacks::EnumVariantValue,
    ) -> Option<bindgen::callbacks::EnumVariantCustomBehavior> {
        // IRQn values differ between chips, so hide them
        if original_variant_name.ends_with("_IRQn") {
            Some(bindgen::callbacks::EnumVariantCustomBehavior::Hide)
        } else {
            None
        }
    }
}

enum Series {
    Nrf52,
    Nrf53,
    Nrf54l,
    Nrf54lNs,
    Nrf54h,
}

impl Series {
    fn get() -> Self {
        let nrf52 = cfg!(feature = "nrf52");
        let nrf53 = cfg!(feature = "nrf53");
        let nrf54l_s = cfg!(feature = "nrf54l-s");
        let nrf54l_ns = cfg!(feature = "nrf54l-ns");
        let nrf54h = cfg!(feature = "nrf54h");
        match (nrf52, nrf53, nrf54l_s, nrf54l_ns, nrf54h) {
            (true, false, false, false, false) => Series::Nrf52,
            (false, true, false, false, false) => Series::Nrf53,
            (false, false, true, false, false) => Series::Nrf54l,
            (false, false, false, true, false) => Series::Nrf54lNs,
            (false, false, false, false, true) => Series::Nrf54h,
            _ => panic!("Exactly one architecture feature must be enabled for nrf_mpsl_sys"),
        }
    }
}

#[derive(Debug)]
struct Target {
    target: String,
    cpu: &'static str,
    float_abi: &'static str,
    chip_family: &'static str,
    chip: &'static str,
    core: Option<&'static str>,
}

impl Target {
    fn new(series: Series, target: String) -> Self {
        let (cpu, float_abi, chip_family, chip, core) = match (series, target.as_str()) {
            (Series::Nrf52, "thumbv7em-none-eabihf") => ("cortex-m4", "hard", "nrf52", "NRF52840_XXAA", None),
            (Series::Nrf52, "thumbv7em-none-eabi") => ("cortex-m4", "soft", "nrf52", "NRF52840_XXAA", None),
            (Series::Nrf53, "thumbv8m.main-none-eabi") => {
                ("cortex-m33+nodsp", "soft", "nrf53", "NRF5340_XXAA", Some("NRF_NETWORK"))
            }
            (Series::Nrf54l, "thumbv8m.main-none-eabihf") => {
                ("cortex-m33", "hard", "nrf54l", "NRF54L15_XXAA", Some("NRF_APPLICATION"))
            }
            (Series::Nrf54l, "thumbv8m.main-none-eabi") => {
                ("cortex-m33", "soft", "nrf54l", "NRF54L15_XXAA", Some("NRF_APPLICATION"))
            }
            (Series::Nrf54lNs, "thumbv8m.main-none-eabihf") => (
                "cortex-m33",
                "hard",
                "nrf54l_ns",
                "NRF54L15_XXAA",
                Some("NRF_APPLICATION"),
            ),
            (Series::Nrf54lNs, "thumbv8m.main-none-eabi") => (
                "cortex-m33",
                "soft",
                "nrf54l_ns",
                "NRF54L15_XXAA",
                Some("NRF_APPLICATION"),
            ),
            (Series::Nrf54h, "thumbv8m.main-none-eabihf") => {
                ("cortex-m33", "hard", "nrf54h", "NRF54H20_XXAA", Some("NRF_RADIOCORE"))
            }
            (Series::Nrf54h, "thumbv8m.main-none-eabi") => {
                ("cortex-m33", "soft", "nrf54h", "NRF54H20_XXAA", Some("NRF_RADIOCORE"))
            }
            _ => panic!("Unsupported target: {:?}", target),
        };

        Self {
            target,
            cpu,
            float_abi,
            chip_family,
            chip,
            core,
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
        .clang_args(target.core.map(|x| format!("-D{}", x)))
        .allowlist_function("mpsl_.*")
        .allowlist_function("MPSL_.*")
        .allowlist_type("mpsl_.*")
        .allowlist_type("MPSL_.*")
        .allowlist_var("MPSL_.*")
        .allowlist_var("NRF_E.*")
        .allowlist_var("UINT8_MAX")
        .blocklist_var("NRF_.*_BASE")
        .prepend_enum_name(false)
        .parse_callbacks(Box::new(Callback))
}

fn main() {
    let target = Target::new(Series::get(), env::var("TARGET").unwrap());

    let mut builder = bindgen(&target)
        .header("./include/stdlib.h")
        .header("./third_party/nordic/nrfxlib/mpsl/include/mpsl.h")
        .header("./third_party/nordic/nrfxlib/mpsl/include/mpsl_clock.h")
        .header("./third_party/nordic/nrfxlib/mpsl/include/mpsl_ecb.h")
        .header("./third_party/nordic/nrfxlib/mpsl/include/mpsl_cx_abstract_interface.h")
        .header("./third_party/nordic/nrfxlib/mpsl/include/mpsl_pm.h")
        .header("./third_party/nordic/nrfxlib/mpsl/include/mpsl_pm_config.h")
        .header("./third_party/nordic/nrfxlib/mpsl/include/mpsl_temp.h")
        .header("./third_party/nordic/nrfxlib/mpsl/include/mpsl_timeslot.h")
        .header("./third_party/nordic/nrfxlib/mpsl/include/mpsl_tx_power.h")
        .header("./third_party/nordic/nrfxlib/mpsl/include/nrf_errno.h")
        .header("./third_party/nordic/nrfxlib/mpsl/include/protocol/mpsl_cx_protocol_api.h")
        .header("./third_party/nordic/nrfxlib/mpsl/include/protocol/mpsl_dppi_protocol_api.h");

    if cfg!(feature = "fem") {
        builder = builder
            .header("./third_party/nordic/nrfxlib/mpsl/fem/include/mpsl_fem_config_common.h")
            .header("./third_party/nordic/nrfxlib/mpsl/fem/include/mpsl_fem_init.h")
            .header("./third_party/nordic/nrfxlib/mpsl/fem/include/mpsl_fem_power_model.h")
            .header("./third_party/nordic/nrfxlib/mpsl/fem/include/mpsl_fem_types.h");
    }

    let (fem_lib, fem_includes): (Option<&str>, Option<&[&str]>) = match (
        cfg!(feature = "fem-simple-gpio"),
        cfg!(feature = "fem-nrf21540-gpio"),
        cfg!(feature = "fem-nrf21540-gpio-spi"),
    ) {
        (false, false, false) => (None, None),
        (true, false, false) => (
            Some("simple_gpio"),
            Some(&[
                "./third_party/nordic/nrfxlib/mpsl/fem/include/protocol/mpsl_fem_protocol_api.h",
                "./third_party/nordic/nrfxlib/mpsl/fem/simple_gpio/include/mpsl_fem_config_simple_gpio.h",
            ]),
        ),
        (false, true, false) => (
            Some("nrf21540_gpio"),
            Some(&[
                "./third_party/nordic/nrfxlib/mpsl/fem/include/mpsl_fem_config_nrf21540_common.h",
                "./third_party/nordic/nrfxlib/mpsl/fem/include/protocol/mpsl_fem_protocol_api.h",
                "./third_party/nordic/nrfxlib/mpsl/fem/nrf21540_gpio/include/mpsl_fem_config_nrf21540_gpio.h",
            ]),
        ),
        (false, false, true) => (Some("nrf21540_gpio_spi"),
            Some(&[
                "./third_party/nordic/nrfxlib/mpsl/fem/include/mpsl_fem_config_nrf21540_common.h",
                "./third_party/nordic/nrfxlib/mpsl/fem/include/protocol/mpsl_fem_protocol_api.h",
                "./third_party/nordic/nrfxlib/mpsl/fem/nrf21540_gpio_spi/include/mpsl_fem_config_nrf21540_gpio_spi.h",
                "./third_party/nordic/nrfxlib/mpsl/fem/nrf21540_gpio_spi/include/mpsl_fem_nrf21540_power_model_builtin.h",
            ]),
        ),
        _ => panic!("Only one front-end module feature may be enabled"),
    };

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
        path.push(target.chip_family);
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

    // Save the third party repo path to the env
    let manifest_dir = PathBuf::from(env::var("CARGO_MANIFEST_DIR").unwrap());
    let third_party_repo_path = manifest_dir.join("third_party");
    let third_party_repo_path_str = third_party_repo_path.to_string_lossy();
    println!("cargo:THIRD_PARTY_REPO_PATH={}", third_party_repo_path_str);
}
