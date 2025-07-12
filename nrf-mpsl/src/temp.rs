//! Temperature sensor access.

#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
/// A temperature measurement.
pub struct Temperature(pub(crate) i32);

impl Temperature {
    /// The integer part of the temperature in degrees Celsius.
    pub fn degrees(self) -> i32 {
        self.0 / 4
    }

    /// The fractional part of the temperature in milli-degrees Celsius.
    pub fn millidegrees(self) -> i32 {
        (self.0 & 0x03) * 250
    }

    /// The raw temperature value, in units of 0.25 degrees Celsius.
    pub fn raw(self) -> i32 {
        self.0
    }
}
