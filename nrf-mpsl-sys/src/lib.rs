#![no_std]
#![allow(non_camel_case_types)]
#![allow(non_upper_case_globals)]
#![allow(non_snake_case)]
#![allow(dead_code)]

mod bindings {
    #![allow(
        clippy::fn_to_numeric_cast,
        clippy::missing_safety_doc,
        clippy::redundant_static_lifetimes,
        clippy::useless_transmute
    )]

    include!(concat!(env!("OUT_DIR"), "/bindings.rs"));
}

pub use bindings::*;
