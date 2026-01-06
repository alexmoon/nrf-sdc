//! Error types for the MPSL crate.
use core::num::NonZeroI32;

use crate::raw;

/// A return value from a raw C function.
///
/// Can be converted to a `Result` to check for success or an error.
#[derive(Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub struct RetVal(i32);

impl RetVal {
    /// A successful return value.
    pub const SUCCESS: RetVal = RetVal(0);

    /// Create a new `RetVal` from an integer.
    pub const fn new(n: i32) -> Self {
        RetVal(n)
    }

    /// Convert the `RetVal` to a `Result`.
    ///
    /// Non-negative values are considered success, and are returned as `Ok(value)`.
    /// Negative values are considered errors, and are returned as `Err(Error)`.
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

/// An error returned from the raw API.
#[derive(Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub struct Error(NonZeroI32);

impl Error {
    const unsafe fn from_u32(err: u32) -> Error {
        Error(NonZeroI32::new_unchecked(-(err as i32)))
    }

    /// Convert an `Error` to a `RetVal`.
    pub const fn to_retval(self) -> RetVal {
        RetVal(self.0.get())
    }
}

impl core::fmt::Display for Error {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        core::fmt::Debug::fmt(self, f)
    }
}

impl core::error::Error for Error {}

impl embedded_io::Error for Error {
    fn kind(&self) -> embedded_io::ErrorKind {
        embedded_io::ErrorKind::Other
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
            ($konst:ident, $name:expr, $raw:ident);
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
                    Self::$konst => defmt::write!(fmt, $name),
                    )+
                    _ => defmt::write!(fmt, "Unknown errno: {}", self.0),
                }
            }
        }

        impl core::fmt::Debug for Error {
            fn fmt(&self, fmt: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
                match *self {
                    $(
                    Self::$konst => core::write!(fmt, $name),
                    )+
                    _ => core::write!(fmt, "Unknown errno: {}", self.0),
                }
            }
        }
    }
}

errnos! {
    /// Operation not permitted.
    (EPERM, "EPERM", NRF_EPERM);
    /// No such file or directory.
    (ENOENT, "ENOENT", NRF_ENOENT);
    /// I/O error.
    (EIO, "EIO", NRF_EIO);
    /// Out of memory.
    (ENOMEM, "ENOMEM", NRF_ENOMEM);
    /// Permission denied.
    (EACCES, "EACCES", NRF_EACCES);
    /// Bad address.
    (EFAULT, "EFAULT", NRF_EFAULT);
    /// Invalid argument.
    (EINVAL, "EINVAL", NRF_EINVAL);
    /// Try again.
    (EAGAIN, "EAGAIN", NRF_EAGAIN);
    /// Protocol wrong type for socket.
    (EPROTOTYPE, "EPROTOTYPE", NRF_EPROTOTYPE);
    /// Protocol not available.
    (ENOPROTOOPT, "ENOPROTOOPT", NRF_ENOPROTOOPT);
    /// Protocol not supported.
    (EPROTONOSUPPORT, "EPROTONOSUPPORT", NRF_EPROTONOSUPPORT);
    /// Socket type not supported.
    (ESOCKTNOSUPPORT, "ESOCKTNOSUPPORT", NRF_ESOCKTNOSUPPORT);
    /// Operation not supported on transport endpoint.
    (EOPNOTSUPP, "EOPNOTSUPP", NRF_EOPNOTSUPP);
    /// Address family not supported by protocol.
    (EAFNOSUPPORT, "EAFNOSUPPORT", NRF_EAFNOSUPPORT);
    /// Address already in use.
    (EADDRINUSE, "EADDRINUSE", NRF_EADDRINUSE);
    /// Network is down.
    (ENETDOWN, "ENETDOWN", NRF_ENETDOWN);
    /// Network is unreachable.
    (ENETUNREACH, "ENETUNREACH", NRF_ENETUNREACH);
    /// Connection reset by peer.
    (ECONNRESET, "ECONNRESET", NRF_ECONNRESET);
    /// Transport endpoint is already connected.
    (EISCONN, "EISCONN", NRF_EISCONN);
    /// Transport endpoint is not connected.
    (ENOTCONN, "ENOTCONN", NRF_ENOTCONN);
    /// Connection timed out.
    (ETIMEDOUT, "ETIMEDOUT", NRF_ETIMEDOUT);
    /// No buffer space available.
    (ENOBUFS, "ENOBUFS", NRF_ENOBUFS);
    /// Host is down.
    (EHOSTDOWN, "EHOSTDOWN", NRF_EHOSTDOWN);
    /// Operation now in progress.
    (EINPROGRESS, "EINPROGRESS", NRF_EINPROGRESS);
    /// Operation Canceled.
    (ECANCELED, "ECANCELED", NRF_ECANCELED);
    /// Required key not available.
    (ENOKEY, "ENOKEY", NRF_ENOKEY);
    /// Key has expired.
    (EKEYEXPIRED, "EKEYEXPIRED", NRF_EKEYEXPIRED);
    /// Key has been revoked.
    (EKEYREVOKED, "EKEYREVOKED", NRF_EKEYREVOKED);
    /// Key was rejected by service.
    (EKEYREJECTED, "EKEYREJECTED", NRF_EKEYREJECTED);
}
