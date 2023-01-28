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
        Some(doxygen_rs::transform(
            &comment.replace('[', "\\[").replace("@sa @ref", "@ref"),
        ))
    }
}

fn main() {
    let target = env::var("TARGET").unwrap();
    let (cpu, float_abi, part) = match target.as_str() {
        "thumbv7em-none-eabihf" => ("cortex-m4", "hard", "NRF52840_XXAA"),
        "thumbv7em-none-eabi" => ("cortex-m4", "soft", "NRF52840_XXAA"),
        "thumbv8m.main-none-eabi" => ("cortex-m33", "soft", "NRF5340_XXAA"),
        _ => panic!("Unsupported target: {:?}", target),
    };

    bindgen::Builder::default()
        .use_core()
        .header("./include/wrapper.h")
        .size_t_is_usize(true)
        .clang_arg(format!("--target={}", target))
        .clang_arg(format!("-mcpu={}", cpu))
        .clang_arg(format!("-mfloat-abi={}", float_abi))
        .clang_arg("-mthumb")
        .clang_arg("-I./include")
        .clang_arg("-I./third_party/arm/CMSIS_5/CMSIS/Core/Include")
        .clang_arg("-I./third_party/nordic/nrfx/mdk")
        .clang_arg("-I./third_party/nordic/nrfxlib/mpsl/include")
        .clang_arg(format!("-D{}", part))
        .allowlist_function("mpsl_.*")
        .allowlist_function("MPSL_.*")
        .allowlist_type("mpsl_.*")
        .allowlist_type("MPSL_.*")
        .allowlist_var("MPSL_.*")
        .allowlist_var("NRF_E.*")
        .default_enum_style(bindgen::EnumVariation::NewType {
            is_bitfield: false,
            is_global: false,
        })
        .rustfmt_bindings(true)
        .parse_callbacks(Box::new(Callback))
        .generate()
        .expect("Unable to generate bindings")
        .write_to_file(PathBuf::from(env::var("OUT_DIR").unwrap()).join("bindings.rs"))
        .expect("Couldn't write bindgen output");

    let lib_path = PathBuf::from_str(&env::var("CARGO_MANIFEST_DIR").unwrap())
        .unwrap()
        .join(format!(
            "./third_party/nordic/nrfxlib/mpsl/lib/{}/{}-float",
            cpu, float_abi
        ));

    println!("cargo:rustc-link-search={}", lib_path.to_str().unwrap());
    println!("cargo:rustc-link-lib=static=mpsl");
}
