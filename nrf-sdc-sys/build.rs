//! Build Script for nrf-mpsl-sys
//!
//! Calls out to bindgen to generate a Rust crate from the Nordic header
//! files.

use std::env;
use std::path::PathBuf;
use std::str::FromStr;

use bindgen::callbacks::ParseCallbacks;

#[derive(Debug)]
struct Callback;

impl ParseCallbacks for Callback {
    fn process_comment(&self, comment: &str) -> Option<String> {
        Some(
            doxygen_rs::transform(
                &comment
                    .replace('[', "\\[")
                    .replace("@sa @ref", "@ref")
                    .replace("SDC_SOC_FLASH_CMD_STATUS.", "sdc_soc_flash_cmd_status."),
            )
            .replace(
                "http://csrc.nist.gov/publications/fips/ fips197/fips-197.pdf",
                "<http://csrc.nist.gov/publications/fips/fips197/fips-197.pdf>",
            ),
        )
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
        .clang_arg("-I./third_party/arm/CMSIS_5/CMSIS/Core/Include")
        .clang_arg("-I./third_party/nordic/nrfx/mdk")
        .clang_arg("-I./third_party/nordic/nrfxlib/softdevice_controller/include")
        .clang_arg(format!("-D{}", target.chip))
        .allowlist_function("sdc_.*")
        .allowlist_function("SDC_.*")
        .allowlist_type("sdc_.*")
        .allowlist_type("SDC_.*")
        .allowlist_var("SDC_.*")
        .allowlist_var("HCI_.*")
        .prepend_enum_name(false)
        .rustfmt_bindings(true)
        .parse_callbacks(Box::new(Callback))
}

fn main() {
    let target = Target::new(env::var("TARGET").unwrap());

    let role = match (cfg!(feature = "peripheral"), cfg!(feature = "central")) {
        (true, true) => "multirole",
        (true, false) => "peripheral",
        (false, true) => "central",
        (false, false) => panic!("At least one of the \"peripheral\" and/or \"central\" features must be enabled!"),
    };

    bindgen(&target)
        .header("./third_party/nordic/nrfxlib/softdevice_controller/include/sdc.h")
        .header("./third_party/nordic/nrfxlib/softdevice_controller/include/sdc_hci.h")
        .header("./third_party/nordic/nrfxlib/softdevice_controller/include/sdc_hci_cmd_controller_baseband.h")
        .header("./third_party/nordic/nrfxlib/softdevice_controller/include/sdc_hci_cmd_info_params.h")
        .header("./third_party/nordic/nrfxlib/softdevice_controller/include/sdc_hci_cmd_le.h")
        .header("./third_party/nordic/nrfxlib/softdevice_controller/include/sdc_hci_cmd_link_control.h")
        .header("./third_party/nordic/nrfxlib/softdevice_controller/include/sdc_hci_cmd_status_params.h")
        .header("./third_party/nordic/nrfxlib/softdevice_controller/include/sdc_hci_vs.h")
        .header("./third_party/nordic/nrfxlib/softdevice_controller/include/sdc_soc.h")
        .generate()
        .expect("Unable to generate bindings")
        .write_to_file(PathBuf::from(env::var("OUT_DIR").unwrap()).join("bindings.rs"))
        .expect("Couldn't write bindgen output");

    let lib_path = PathBuf::from_str(&env::var("CARGO_MANIFEST_DIR").unwrap())
        .unwrap()
        .join(format!(
            "./third_party/nordic/nrfxlib/softdevice_controller/lib/{}/{}-float",
            target.cpu, target.float_abi
        ));

    println!("cargo:rustc-link-search={}", lib_path.to_str().unwrap());
    println!("cargo:rustc-link-lib=static=softdevice_controller_{}", role);
}
