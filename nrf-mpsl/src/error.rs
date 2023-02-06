use core::num::NonZeroI32;

use crate::raw;

#[derive(Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub struct RetVal(i32);

impl RetVal {
    pub const SUCCESS: RetVal = RetVal(0);

    pub const fn new(n: i32) -> Self {
        RetVal(n)
    }

    pub const fn to_result(self) -> Result<u32, Error> {
        if self.0 >= 0 {
            Ok(self.0 as u32)
        } else {
            Err(Error(unsafe { NonZeroI32::new_unchecked(self.0) }))
        }
    }
}

#[cfg(feature = "defmt")]
impl defmt::Format for RetVal {
    fn format(&self, fmt: defmt::Formatter) {
        defmt::Format::format(&self.to_result(), fmt)
    }
}

impl core::fmt::Debug for RetVal {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        core::fmt::Debug::fmt(&self.to_result(), f)
    }
}

impl From<i32> for RetVal {
    fn from(value: i32) -> Self {
        RetVal(value)
    }
}

impl From<RetVal> for i32 {
    fn from(value: RetVal) -> Self {
        value.0
    }
}

#[derive(Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub struct Error(NonZeroI32);

impl Error {
    const unsafe fn from_u32(err: u32) -> Error {
        Error(NonZeroI32::new_unchecked(-(err as i32)))
    }

    pub const fn to_retval(self) -> RetVal {
        RetVal(self.0.get())
    }
}

impl core::fmt::Display for Error {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        core::fmt::Debug::fmt(self, f)
    }
}

impl From<Error> for i32 {
    fn from(value: Error) -> Self {
        value.0.get()
    }
}

macro_rules! errnos {
    (
        $(
            $(#[$docs:meta])*
            ($konst:ident, $raw:ident);
        )+
    ) => {
        impl Error {
        $(
            $(#[$docs])*
            pub const $konst: Error = unsafe { Error::from_u32(raw::$raw) };
        )+
        }

        impl RetVal {
        $(
            $(#[$docs])*
            pub const $konst: RetVal = Error::$konst.to_retval();
        )+
        }

        #[cfg(feature = "defmt")]
        impl defmt::Format for Error {
            fn format(&self, fmt: defmt::Formatter) {
                match *self {
                    $(
                    Self::$konst => defmt::write!(fmt, stringify!($konst)),
                    )+
                    _ => defmt::write!(fmt, "Unknown errno: {}", self.0),
                }
            }
        }

        impl core::fmt::Debug for Error {
            fn fmt(&self, fmt: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
                match *self {
                    $(
                    Self::$konst => core::write!(fmt, stringify!($konst)),
                    )+
                    _ => core::write!(fmt, "Unknown errno: {}", self.0),
                }
            }
        }
    }
}

errnos! {
    (EPERM, NRF_EPERM);
    (ENOENT, NRF_ENOENT);
    (EIO, NRF_EIO);
    (ENOMEM, NRF_ENOMEM);
    (EACCES, NRF_EACCES);
    (EFAULT, NRF_EFAULT);
    (EINVAL, NRF_EINVAL);
    (EAGAIN, NRF_EAGAIN);
    (EPROTOTYPE, NRF_EPROTOTYPE);
    (ENOPROTOOPT, NRF_ENOPROTOOPT);
    (EPROTONOSUPPORT, NRF_EPROTONOSUPPORT);
    (ESOCKTNOSUPPORT, NRF_ESOCKTNOSUPPORT);
    (EOPNOTSUPP, NRF_EOPNOTSUPP);
    (EAFNOSUPPORT, NRF_EAFNOSUPPORT);
    (EADDRINUSE, NRF_EADDRINUSE);
    (ENETDOWN, NRF_ENETDOWN);
    (ENETUNREACH, NRF_ENETUNREACH);
    (ECONNRESET, NRF_ECONNRESET);
    (EISCONN, NRF_EISCONN);
    (ENOTCONN, NRF_ENOTCONN);
    (ETIMEDOUT, NRF_ETIMEDOUT);
    (ENOBUFS, NRF_ENOBUFS);
    (EHOSTDOWN, NRF_EHOSTDOWN);
    (EINPROGRESS, NRF_EINPROGRESS);
    (ECANCELED, NRF_ECANCELED);
    (ENOKEY, NRF_ENOKEY);
    (EKEYEXPIRED, NRF_EKEYEXPIRED);
    (EKEYREVOKED, NRF_EKEYREVOKED);
    (EKEYREJECTED, NRF_EKEYREJECTED);
    (ECB_BASE, NRF_ECB_BASE);
    (EGU0_BASE, NRF_EGU0_BASE);
    (EGU1_BASE, NRF_EGU1_BASE);
    (EGU2_BASE, NRF_EGU2_BASE);
    (EGU3_BASE, NRF_EGU3_BASE);
    (EGU4_BASE, NRF_EGU4_BASE);
    (EGU5_BASE, NRF_EGU5_BASE);
}
