//! Build Script for nrf-sdc-sys
//!
//! Calls out to bindgen to generate a Rust crate from the Nordic header
//! files.

use std::cell::RefCell;
use std::env;
use std::fs::OpenOptions;
use std::io::Write;
use std::path::PathBuf;
use std::rc::Rc;

use bindgen::callbacks::ParseCallbacks;
use winnow::Parser;

#[derive(Debug)]
struct Callback {
    mem_fns: Rc<RefCell<Vec<u8>>>,
}

mod parser {
    use winnow::ascii::digit1;
    use winnow::combinator::{alt, cut_err, delimited, repeat, separated, separated_pair};
    use winnow::prelude::*;
    use winnow::stream::AsChar;
    use winnow::token::{one_of, take_while};

    pub fn uint<'s>(i: &mut &'s str) -> ModalResult<&'s str> {
        digit1.parse_next(i)
    }

    pub fn ident<'s>(i: &mut &'s str) -> ModalResult<&'s str> {
        (
            one_of(|c: char| c.is_alpha() || c == '_'),
            take_while(0.., |c: char| c.is_alphanum() || c == '_'),
        )
            .take()
            .parse_next(i)
    }

    pub fn ternary(i: &mut &str) -> ModalResult<String> {
        separated_pair(expr, '?', cut_err(separated_pair(expr, ':', expr)))
            .context(winnow::error::StrContext::Label("ternary expression"))
            .map(|(condition, (if_true, if_false))| format!("if {condition} {{ {if_true} }} else {{ {if_false} }}"))
            .parse_next(i)
    }

    pub fn func(i: &mut &str) -> ModalResult<String> {
        let func = ident.parse_next(i)?;

        let args = delimited(
            '(',
            cut_err(separated(0.., expr, ',').map(|x: Vec<String>| x.join(","))),
            ')',
        )
        .context(winnow::error::StrContext::Label("function-like macro call"))
        .parse_next(i)?;

        Ok(format!("{func}({args})"))
    }

    pub fn expr(i: &mut &str) -> ModalResult<String> {
        let init = term.parse_next(i)?;

        repeat(
            0..,
            (alt(("+", "-", "*", "/", ">", "<", "&&")), cut_err(term))
                .context(winnow::error::StrContext::Label("expression")),
        )
        .fold(
            move || init.clone(),
            |mut acc: String, (op, val): (&str, String)| {
                acc.push_str(op);
                acc.push_str(&val);
                acc
            },
        )
        .parse_next(i)
    }

    pub fn term(i: &mut &str) -> ModalResult<String> {
        alt((func, ident.map(str::to_owned), uint.map(str::to_owned), parens))
            .context(winnow::error::StrContext::Label("term"))
            .parse_next(i)
    }

    pub fn parens(i: &mut &str) -> ModalResult<String> {
        delimited('(', cut_err(alt((ternary, expr))), ')')
            .context(winnow::error::StrContext::Label("parenthesized expression"))
            .map(|x| format!("({x})"))
            .parse_next(i)
    }

    /// A quick and dirty parser to translate the function-like `SDC_MEM_` macros from
    /// C to Rust.
    ///
    /// This handles only the very small subset of C expressions used in those
    /// macros. The primary purpose of the translation is to turn C-style ternary
    /// expressions into Rust if/else expressions.
    pub fn sdc_mem_macro(i: &mut &str) -> ModalResult<String> {
        parens.parse_next(i)
    }
}

impl ParseCallbacks for Callback {
    fn func_macro(&self, name: &str, value: &[&[u8]]) {
        if name.starts_with("SDC_MEM_") || name.starts_with("__MEM_") {
            let i = name.find('(').unwrap();
            let args = name[(i + 1)..(name.len() - 1)]
                .split(',')
                .map(|x| {
                    if x.ends_with("_enabled") || x.ends_with("step_mode3_supported") {
                        format!("{x}: bool")
                    } else {
                        format!("{x}: u32")
                    }
                })
                .collect::<Vec<_>>()
                .join(", ");
            let name = &name[..i];

            fn stringify(value: &[&[u8]]) -> String {
                value
                    .iter()
                    .map(|x| core::str::from_utf8(x).unwrap())
                    .collect::<String>()
            }

            let body = parser::sdc_mem_macro.parse(&stringify(value)).unwrap();

            write!(
                self.mem_fns.borrow_mut(),
                "#[allow(clippy::identity_op)]\nconst fn {name}({args}) -> u32 {{\n  {body}\n}}\n",
            )
            .unwrap();
        }
    }

    fn process_comment(&self, comment: &str) -> Option<String> {
        Some(doxygen_rs::transform(&comment.replace('[', "\\[")).replace(
            "http://csrc.nist.gov/publications/fips/ fips197/fips-197.pdf",
            "<http://csrc.nist.gov/publications/fips/fips197/fips-197.pdf>",
        ))
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
            _ => panic!("Exactly one architecture feature must be enabled for nrf_sdc_sys"),
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

fn bindgen(target: &Target, mem_fns: Rc<RefCell<Vec<u8>>>) -> bindgen::Builder {
    bindgen::Builder::default()
        .use_core()
        .size_t_is_usize(true)
        .clang_arg(format!("--target={}", target.target))
        .clang_arg(format!("-mcpu={}", target.cpu))
        .clang_arg(format!("-mfloat-abi={}", target.float_abi))
        .clang_arg("-mthumb")
        .clang_arg("-I./third_party/arm/CMSIS_5/CMSIS/Core/Include")
        .clang_arg("-I./third_party/nordic/nrfx")
        .clang_arg("-I./third_party/nordic/nrfx/mdk")
        .clang_arg("-I./third_party/nordic/nrfxlib/softdevice_controller/include")
        .clang_arg(format!("-D{}", target.chip))
        .clang_args(target.core.map(|x| format!("-D{}", x)))
        .allowlist_function("sdc_.*")
        .allowlist_function("SDC_.*")
        .allowlist_type("sdc_.*")
        .allowlist_type("SDC_.*")
        .allowlist_var("SDC_.*")
        .allowlist_var("HCI_.*")
        .allowlist_var("__MEM_.*")
        .prepend_enum_name(false)
        .parse_callbacks(Box::new(Callback { mem_fns }))
}

fn main() {
    let target = Target::new(Series::get(), env::var("TARGET").unwrap());

    // Only nrf52 series have different binaries depending on the role.
    let role = if cfg!(feature = "nrf52") {
        match (cfg!(feature = "peripheral"), cfg!(feature = "central")) {
            (true, true) => "multirole",
            (true, false) => "peripheral",
            (false, true) => "central",
            (false, false) => panic!("At least one of the \"peripheral\" and/or \"central\" features must be enabled!"),
        }
    } else {
        "multirole"
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
        .open(PathBuf::from(env::var_os("OUT_DIR").unwrap()).join("bindings.rs"))
        .unwrap();
    bindings.write(Box::new(&file)).expect("Couldn't write bindgen output");
    file.write_all(&mem_fns.borrow())
        .expect("Couldn't write SDC_MEM_* functions");

    let mut lib_path = PathBuf::from(env::var_os("CARGO_MANIFEST_DIR").unwrap());
    lib_path.push("third_party/nordic/nrfxlib/softdevice_controller/lib");
    lib_path.push(target.chip_family);
    lib_path.push(format!("{}-float", target.float_abi));

    println!("cargo:rustc-link-search={}", lib_path.to_str().unwrap());
    println!("cargo:rustc-link-lib=static=softdevice_controller_{}", role);
}
