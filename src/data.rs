//! Compensated sensor data and field metadata.

use crate::{FixedMeasurement, RawMeasurement};

/// Status bits attached to a `BME68x` measurement field.
#[derive(Clone, Copy, Debug, Default, Eq, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct MeasurementStatus(u8);

impl MeasurementStatus {
    /// Create status flags from a complete status byte.
    ///
    /// The three documented Bosch flags occupy bits 7, 5, and 4. Remaining
    /// bits are retained so callers can detect future or unexpected status
    /// flags instead of silently discarding them.
    #[must_use]
    pub const fn from_bits(bits: u8) -> Self {
        Self(bits)
    }

    /// Return the complete status byte supplied by the sensor/decoder.
    #[must_use]
    pub const fn bits(self) -> u8 {
        self.0
    }

    /// Return only the three status bits documented by Bosch.
    #[must_use]
    pub const fn documented_bits(self) -> u8 {
        self.0 & 0xb0
    }

    /// Return status bits not currently documented by Bosch.
    #[must_use]
    pub const fn unknown_bits(self) -> u8 {
        self.0 & !0xb0
    }

    /// Whether this field contains data not previously read.
    #[must_use]
    pub const fn is_new(self) -> bool {
        self.0 & 0x80 != 0
    }

    /// Whether the gas conversion is valid.
    #[must_use]
    pub const fn gas_valid(self) -> bool {
        self.0 & 0x20 != 0
    }

    /// Whether the gas heater reached its target.
    #[must_use]
    pub const fn heater_stable(self) -> bool {
        self.0 & 0x10 != 0
    }
}

/// One compensated measurement plus Bosch field and heater metadata.
#[derive(Clone, Copy, Debug, Default, Eq, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct Measurement {
    /// Bosch's combined data-ready, gas-valid, and heater-stable status byte.
    pub status: MeasurementStatus,
    /// Exact field-status/index register byte (`FIELDx[0]`).
    ///
    /// The gas index is the low nibble; the remaining bits are retained
    /// verbatim for diagnostics.
    pub raw_field_status: u8,
    /// Exact gas-ADC status/range byte selected for the detected variant.
    ///
    /// This is `FIELDx[14]` for Gas Low or `FIELDx[16]` for Gas High.
    pub raw_gas_status: u8,
    /// Heater-profile index used for this conversion.
    pub gas_index: u8,
    /// Wrapping sub-measurement index used to order fields.
    pub measurement_index: u8,
    /// Raw heater-resistance register value used for this conversion.
    pub heater_resistance: u8,
    /// Raw heater current-DAC register value used for this conversion.
    pub heater_current: u8,
    /// Raw gas-wait register value used for this conversion.
    pub gas_wait: u8,
    /// Uncompensated ADC values decoded from this sensor field.
    ///
    /// This is also the input for the optional Bosch-compatible floating-point
    /// compensation path in [`crate::float`] when the `float` feature is on.
    pub raw: RawMeasurement,
    /// Temperature, pressure, humidity, and gas resistance.
    pub values: FixedMeasurement,
}

/// Up to three data fields returned by the `BME68x` FIFO-style field registers.
#[derive(Clone, Copy, Debug, Default, Eq, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct Measurements {
    data: [Measurement; 3],
    len: u8,
}

impl Measurements {
    #[cfg(any(feature = "blocking", feature = "async"))]
    pub(crate) const fn new(data: [Measurement; 3], len: u8) -> Self {
        Self { data, len }
    }

    /// Number of newly available fields.
    #[must_use]
    pub const fn len(&self) -> usize {
        self.len as usize
    }

    /// Whether no newly available fields were found.
    #[must_use]
    pub const fn is_empty(&self) -> bool {
        self.len == 0
    }

    /// Newly available fields, ordered from oldest to newest.
    #[must_use]
    pub fn as_slice(&self) -> &[Measurement] {
        &self.data[..self.len()]
    }

    /// Iterate over newly available fields from oldest to newest.
    pub fn iter(&self) -> core::slice::Iter<'_, Measurement> {
        self.as_slice().iter()
    }
}

impl<'a> IntoIterator for &'a Measurements {
    type Item = &'a Measurement;
    type IntoIter = core::slice::Iter<'a, Measurement>;

    fn into_iter(self) -> Self::IntoIter {
        self.iter()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn status_preserves_unknown_bits_while_decoding_documented_flags() {
        let status = MeasurementStatus::from_bits(0xf5);
        assert_eq!(status.bits(), 0xf5);
        assert_eq!(status.documented_bits(), 0xb0);
        assert_eq!(status.unknown_bits(), 0x45);
        assert!(status.is_new());
        assert!(status.gas_valid());
        assert!(status.heater_stable());
    }
}
