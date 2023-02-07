#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct Temperature(pub(crate) i32);

impl Temperature {
    // The integer part of the temperature in ˚C
    pub fn degrees(self) -> i32 {
        self.0 / 4
    }

    // The fractional part of the temperature in ˚C
    pub fn millidegrees(self) -> i32 {
        (self.0 & 0x03) * 250
    }

    // The raw temperature value in ¼˚C units
    pub fn raw(self) -> i32 {
        self.0
    }
}
