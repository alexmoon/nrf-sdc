//! Build Script for nrf-mpsl-sys
//!
//! Calls out to bindgen to generate a Rust crate from the Nordic header
//! files.

use std::cell::RefCell;
use std::env;
use std::fs::OpenOptions;
use std::io::Write;
use std::path::PathBuf;
use std::rc::Rc;
use std::str::FromStr;

use bindgen::callbacks::ParseCallbacks;

#[derive(Debug)]
struct Callback {
    mem_fns: Rc<RefCell<Vec<u8>>>,
}

impl ParseCallbacks for Callback {
    fn func_macro(&self, name: &str, value: &[&[u8]]) {
        if name.starts_with("SDC_MEM_") || name.starts_with("__MEM_") {
            let i = name.find('(').unwrap();
            let args = name[(i + 1)..(name.len() - 1)]
                .split(',')
                .map(|x| format!("{}: u32", x))
                .collect::<Vec<_>>()
                .join(", ");
            let name = &name[..i];

            fn stringify(value: &[&[u8]]) -> String {
                value
                    .iter()
                    .map(|x| core::str::from_utf8(x).unwrap())
                    .collect::<String>()
            }

            let body = if let Some(i) = value.iter().position(|x| x == b"?") {
                // Translate a simple ternary expression to an if/else statement
                assert_eq!(value.first().unwrap(), b"(");
                assert_eq!(value.last().unwrap(), b")");

                let j = value
                    .iter()
                    .position(|x| x == b":")
                    .expect("Incomplete ternary expression in SDC_MEM_* macro");

                let condition = stringify(&value[1..i]);
                let consequent = stringify(&value[(i + 1)..j]);
                let alternative = stringify(&value[(j + 1)..(value.len() - 1)]);

                format!(
                    "if {} {{\n    {}\n  }} else {{\n    {}\n  }}",
                    condition, consequent, alternative
                )
            } else {
                stringify(value)
            };

            write!(
                self.mem_fns.borrow_mut(),
                "const fn {}({}) -> u32 {{\n  {}\n}}\n",
                name,
                args,
                body
            )
            .unwrap();
        }
    }

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

fn bindgen(target: &Target, mem_fns: Rc<RefCell<Vec<u8>>>) -> bindgen::Builder {
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
        .allowlist_var("__MEM_.*")
        .prepend_enum_name(false)
        .rustfmt_bindings(true)
        .parse_callbacks(Box::new(Callback { mem_fns }))
}

fn main() {
    let target = Target::new(env::var("TARGET").unwrap());

    let role = match (cfg!(feature = "peripheral"), cfg!(feature = "central")) {
        (true, true) => "multirole",
        (true, false) => "peripheral",
        (false, true) => "central",
        (false, false) => panic!("At least one of the \"peripheral\" and/or \"central\" features must be enabled!"),
    };

    let mem_fns = Rc::new(RefCell::new(Vec::new()));
    let bindings = bindgen(&target, mem_fns.clone())
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
        .expect("Unable to generate bindings");

    let mut file = OpenOptions::new()
        .write(true)
        .truncate(true)
        .create(true)
        .open(PathBuf::from(env::var("OUT_DIR").unwrap()).join("bindings.rs"))
        .unwrap();
    bindings.write(Box::new(&file)).expect("Couldn't write bindgen output");
    file.write_all(&mem_fns.borrow())
        .expect("Couldn't write SDC_MEM_* functions");

    let lib_path = PathBuf::from_str(&env::var("CARGO_MANIFEST_DIR").unwrap())
        .unwrap()
        .join(format!(
            "./third_party/nordic/nrfxlib/softdevice_controller/lib/{}/{}-float",
            target.cpu, target.float_abi
        ));

    println!("cargo:rustc-link-search={}", lib_path.to_str().unwrap());
    println!("cargo:rustc-link-lib=static=softdevice_controller_{}", role);
}
