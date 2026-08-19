// Portions of this file are derived from Bosch Sensortec's
// BME68x SensorAPI v4.4.8.
// Copyright (c) 2023 Bosch Sensortec GmbH. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause

//! Platform-independent data types used by the `BME68x` driver.

// The floating-point accessors are convenience views over an exact integer
// representation. As with Bosch's FPU API, very large u32 values can round.
#![allow(clippy::cast_precision_loss)]

/// Number of bytes in the three `BME68x` calibration register blocks.
pub const CALIBRATION_DATA_LEN: usize = 42;

/// Length of the calibration block starting at register `0x8a`.
pub const CALIBRATION_BLOCK_1_LEN: usize = 23;

/// Length of the calibration block starting at register `0xe1`.
pub const CALIBRATION_BLOCK_2_LEN: usize = 14;

/// Length of the calibration block starting at register `0x00` on memory page 1.
pub const CALIBRATION_BLOCK_3_LEN: usize = 5;

const fn signed_byte(value: u8) -> i8 {
    i8::from_ne_bytes([value])
}

/// Gas-resistance calculation variant reported by the sensor.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[repr(u8)]
pub enum Variant {
    /// Original low-gas-resistance calculation used by BME680 devices.
    GasLow = 0,
    /// High-gas-resistance calculation used by newer BME688 devices.
    GasHigh = 1,
}

impl Variant {
    /// Converts the variant-ID register value into a typed variant.
    #[must_use]
    pub const fn from_register(value: u8) -> Option<Self> {
        match value {
            0 => Some(Self::GasLow),
            1 => Some(Self::GasHigh),
            _ => None,
        }
    }

    /// Returns the value stored in the variant-ID register.
    #[must_use]
    pub const fn register_value(self) -> u8 {
        self as u8
    }
}

/// Sensor operating mode.
#[derive(Clone, Copy, Debug, Default, Eq, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[repr(u8)]
pub enum OperationMode {
    /// No measurement is running.
    #[default]
    Sleep = 0,
    /// Perform one measurement, then return to sleep.
    Forced = 1,
    /// Cycle through a heater profile with shared heater duration.
    Parallel = 2,
    /// Cycle through a heater profile sequentially.
    Sequential = 3,
}

impl OperationMode {
    /// Converts the mode bits into a typed mode.
    #[must_use]
    pub const fn from_register(value: u8) -> Option<Self> {
        match value {
            0 => Some(Self::Sleep),
            1 => Some(Self::Forced),
            2 => Some(Self::Parallel),
            3 => Some(Self::Sequential),
            _ => None,
        }
    }

    /// Returns the mode bits written to the sensor.
    #[must_use]
    pub const fn register_value(self) -> u8 {
        self as u8
    }
}

/// Temperature, pressure, or humidity oversampling setting.
#[derive(Clone, Copy, Debug, Default, Eq, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[repr(u8)]
pub enum Oversampling {
    /// Skip this measurement.
    #[default]
    None = 0,
    /// One sample.
    X1 = 1,
    /// Two samples.
    X2 = 2,
    /// Four samples.
    X4 = 3,
    /// Eight samples.
    X8 = 4,
    /// Sixteen samples.
    X16 = 5,
}

impl Oversampling {
    /// Converts an oversampling register field into a typed value.
    #[must_use]
    pub const fn from_register(value: u8) -> Option<Self> {
        match value {
            0 => Some(Self::None),
            1 => Some(Self::X1),
            2 => Some(Self::X2),
            3 => Some(Self::X4),
            4 => Some(Self::X8),
            5 => Some(Self::X16),
            _ => None,
        }
    }

    /// Returns the oversampling register field.
    #[must_use]
    pub const fn register_value(self) -> u8 {
        self as u8
    }

    /// Returns the number of ADC measurement cycles.
    #[must_use]
    pub const fn measurement_cycles(self) -> u32 {
        match self {
            Self::None => 0,
            Self::X1 => 1,
            Self::X2 => 2,
            Self::X4 => 4,
            Self::X8 => 8,
            Self::X16 => 16,
        }
    }
}

/// IIR filter setting.
///
/// Variant names follow Bosch's `FILTER_SIZE_*` names. The corresponding
/// filter coefficients are 2, 4, 8, 16, 32, 64, and 128.
#[derive(Clone, Copy, Debug, Default, Eq, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[repr(u8)]
pub enum Filter {
    /// IIR filtering disabled.
    #[default]
    Off = 0,
    /// Bosch filter-size setting 1 (coefficient 2).
    Size1 = 1,
    /// Bosch filter-size setting 3 (coefficient 4).
    Size3 = 2,
    /// Bosch filter-size setting 7 (coefficient 8).
    Size7 = 3,
    /// Bosch filter-size setting 15 (coefficient 16).
    Size15 = 4,
    /// Bosch filter-size setting 31 (coefficient 32).
    Size31 = 5,
    /// Bosch filter-size setting 63 (coefficient 64).
    Size63 = 6,
    /// Bosch filter-size setting 127 (coefficient 128).
    Size127 = 7,
}

impl Filter {
    /// Converts the filter register field into a typed value.
    #[must_use]
    pub const fn from_register(value: u8) -> Option<Self> {
        match value {
            0 => Some(Self::Off),
            1 => Some(Self::Size1),
            2 => Some(Self::Size3),
            3 => Some(Self::Size7),
            4 => Some(Self::Size15),
            5 => Some(Self::Size31),
            6 => Some(Self::Size63),
            7 => Some(Self::Size127),
            _ => None,
        }
    }

    /// Returns the filter register field.
    #[must_use]
    pub const fn register_value(self) -> u8 {
        self as u8
    }
}

/// Standby time used between sequential-mode measurements.
#[derive(Clone, Copy, Debug, Default, Eq, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[repr(u8)]
pub enum StandbyTime {
    /// 0.59 ms.
    Millis0_59 = 0,
    /// 62.5 ms.
    Millis62_5 = 1,
    /// 125 ms.
    Millis125 = 2,
    /// 250 ms.
    Millis250 = 3,
    /// 500 ms.
    Millis500 = 4,
    /// 1,000 ms.
    Millis1000 = 5,
    /// 10 ms.
    Millis10 = 6,
    /// 20 ms.
    Millis20 = 7,
    /// No standby time.
    #[default]
    None = 8,
}

impl StandbyTime {
    /// Converts Bosch's logical ODR value into a typed standby time.
    #[must_use]
    pub const fn from_register(value: u8) -> Option<Self> {
        match value {
            0 => Some(Self::Millis0_59),
            1 => Some(Self::Millis62_5),
            2 => Some(Self::Millis125),
            3 => Some(Self::Millis250),
            4 => Some(Self::Millis500),
            5 => Some(Self::Millis1000),
            6 => Some(Self::Millis10),
            7 => Some(Self::Millis20),
            8 => Some(Self::None),
            _ => None,
        }
    }

    /// Returns Bosch's logical ODR value.
    #[must_use]
    pub const fn register_value(self) -> u8 {
        self as u8
    }
}

/// Temperature, pressure, humidity, filter, and standby configuration.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct Configuration {
    /// Humidity oversampling.
    pub humidity_oversampling: Oversampling,
    /// Temperature oversampling.
    pub temperature_oversampling: Oversampling,
    /// Pressure oversampling.
    pub pressure_oversampling: Oversampling,
    /// IIR filter setting.
    pub filter: Filter,
    /// Sequential-mode standby time.
    pub standby_time: StandbyTime,
}

impl Default for Configuration {
    fn default() -> Self {
        Self {
            humidity_oversampling: Oversampling::None,
            temperature_oversampling: Oversampling::None,
            pressure_oversampling: Oversampling::None,
            filter: Filter::Off,
            standby_time: StandbyTime::None,
        }
    }
}

/// Raw ADC values extracted from one sensor data field.
#[derive(Clone, Copy, Debug, Default, Eq, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct RawMeasurement {
    /// 20-bit uncompensated temperature ADC value.
    pub temperature_adc: u32,
    /// 20-bit uncompensated pressure ADC value.
    pub pressure_adc: u32,
    /// 16-bit uncompensated humidity ADC value.
    pub humidity_adc: u16,
    /// 10-bit uncompensated gas-resistance ADC value.
    pub gas_resistance_adc: u16,
    /// Four-bit gas-resistance range value.
    pub gas_range: u8,
}

/// One fully compensated measurement in Bosch's exact fixed-point units.
#[derive(Clone, Copy, Debug, Default, Eq, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct FixedMeasurement {
    /// Temperature in hundredths of a degree Celsius.
    pub temperature: i16,
    /// Pressure in pascals.
    pub pressure: u32,
    /// Relative humidity in thousandths of a percent RH.
    pub humidity: u32,
    /// Gas resistance in ohms.
    pub gas_resistance: u32,
}

impl FixedMeasurement {
    /// Temperature in degrees Celsius.
    #[must_use]
    pub fn temperature_celsius(self) -> f32 {
        f32::from(self.temperature) / 100.0
    }

    /// Pressure in pascals as a floating-point value.
    #[must_use]
    pub fn pressure_pascals(self) -> f32 {
        self.pressure as f32
    }

    /// Pressure in hectopascals.
    #[must_use]
    pub fn pressure_hectopascals(self) -> f32 {
        self.pressure as f32 / 100.0
    }

    /// Relative humidity in percent RH.
    #[must_use]
    pub fn humidity_percent(self) -> f32 {
        self.humidity as f32 / 1_000.0
    }

    /// Gas resistance in ohms as a floating-point value.
    #[must_use]
    pub fn gas_resistance_ohms(self) -> f32 {
        self.gas_resistance as f32
    }
}

/// Factory calibration coefficients read from the sensor.
///
/// The fields intentionally mirror Bosch's `bme68x_calib_data` names so that
/// values and differential-test vectors can be compared without translation.
#[derive(Clone, Copy, Debug, Default, Eq, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct CalibrationData {
    /// Humidity calibration coefficient H1.
    pub par_h1: u16,
    /// Humidity calibration coefficient H2.
    pub par_h2: u16,
    /// Humidity calibration coefficient H3.
    pub par_h3: i8,
    /// Humidity calibration coefficient H4.
    pub par_h4: i8,
    /// Humidity calibration coefficient H5.
    pub par_h5: i8,
    /// Humidity calibration coefficient H6.
    pub par_h6: u8,
    /// Humidity calibration coefficient H7.
    pub par_h7: i8,
    /// Gas-heater calibration coefficient GH1.
    pub par_gh1: i8,
    /// Gas-heater calibration coefficient GH2.
    pub par_gh2: i16,
    /// Gas-heater calibration coefficient GH3.
    pub par_gh3: i8,
    /// Temperature calibration coefficient T1.
    pub par_t1: u16,
    /// Temperature calibration coefficient T2.
    pub par_t2: i16,
    /// Temperature calibration coefficient T3.
    pub par_t3: i8,
    /// Pressure calibration coefficient P1.
    pub par_p1: u16,
    /// Pressure calibration coefficient P2.
    pub par_p2: i16,
    /// Pressure calibration coefficient P3.
    pub par_p3: i8,
    /// Pressure calibration coefficient P4.
    pub par_p4: i16,
    /// Pressure calibration coefficient P5.
    pub par_p5: i16,
    /// Pressure calibration coefficient P6.
    pub par_p6: i8,
    /// Pressure calibration coefficient P7.
    pub par_p7: i8,
    /// Pressure calibration coefficient P8.
    pub par_p8: i16,
    /// Pressure calibration coefficient P9.
    pub par_p9: i16,
    /// Pressure calibration coefficient P10.
    pub par_p10: u8,
    /// Temperature fine-resolution intermediate used by pressure and humidity.
    pub(crate) t_fine: i32,
    /// Heater resistance range coefficient.
    pub res_heat_range: u8,
    /// Heater resistance value coefficient.
    pub res_heat_val: i8,
    /// Gas range switching-error coefficient.
    pub range_sw_err: i8,
}

impl CalibrationData {
    /// Parses the concatenated 42 bytes from Bosch's three calibration blocks.
    #[must_use]
    pub fn from_register_bytes(bytes: &[u8; CALIBRATION_DATA_LEN]) -> Self {
        const H1_DATA_MASK: u8 = 0x0f;
        const RES_HEAT_RANGE_MASK: u8 = 0x30;
        const RANGE_SWITCH_ERROR_MASK: u8 = 0xf0;

        Self {
            par_h1: (u16::from(bytes[25]) << 4) | u16::from(bytes[24] & H1_DATA_MASK),
            par_h2: (u16::from(bytes[23]) << 4) | u16::from(bytes[24] >> 4),
            par_h3: signed_byte(bytes[26]),
            par_h4: signed_byte(bytes[27]),
            par_h5: signed_byte(bytes[28]),
            par_h6: bytes[29],
            par_h7: signed_byte(bytes[30]),
            par_gh1: signed_byte(bytes[35]),
            par_gh2: i16::from_le_bytes([bytes[33], bytes[34]]),
            par_gh3: signed_byte(bytes[36]),
            par_t1: u16::from_le_bytes([bytes[31], bytes[32]]),
            par_t2: i16::from_le_bytes([bytes[0], bytes[1]]),
            par_t3: signed_byte(bytes[2]),
            par_p1: u16::from_le_bytes([bytes[4], bytes[5]]),
            par_p2: i16::from_le_bytes([bytes[6], bytes[7]]),
            par_p3: signed_byte(bytes[8]),
            par_p4: i16::from_le_bytes([bytes[10], bytes[11]]),
            par_p5: i16::from_le_bytes([bytes[12], bytes[13]]),
            par_p6: signed_byte(bytes[15]),
            par_p7: signed_byte(bytes[14]),
            par_p8: i16::from_le_bytes([bytes[18], bytes[19]]),
            par_p9: i16::from_le_bytes([bytes[20], bytes[21]]),
            par_p10: bytes[22],
            t_fine: 0,
            res_heat_range: (bytes[39] & RES_HEAT_RANGE_MASK) / 16,
            res_heat_val: signed_byte(bytes[37]),
            range_sw_err: signed_byte(bytes[41] & RANGE_SWITCH_ERROR_MASK) / 16,
        }
    }

    /// Parses the three discontiguous calibration register blocks.
    #[must_use]
    pub fn from_register_blocks(
        block_1: &[u8; CALIBRATION_BLOCK_1_LEN],
        block_2: &[u8; CALIBRATION_BLOCK_2_LEN],
        block_3: &[u8; CALIBRATION_BLOCK_3_LEN],
    ) -> Self {
        let mut bytes = [0_u8; CALIBRATION_DATA_LEN];
        bytes[..CALIBRATION_BLOCK_1_LEN].copy_from_slice(block_1);
        bytes[CALIBRATION_BLOCK_1_LEN..CALIBRATION_BLOCK_1_LEN + CALIBRATION_BLOCK_2_LEN]
            .copy_from_slice(block_2);
        bytes[CALIBRATION_BLOCK_1_LEN + CALIBRATION_BLOCK_2_LEN..].copy_from_slice(block_3);
        Self::from_register_bytes(&bytes)
    }

    /// Returns the current Bosch `t_fine` compensation intermediate.
    #[must_use]
    pub const fn temperature_fine(&self) -> i32 {
        self.t_fine
    }

    /// Encode the immutable factory coefficients in Bosch's canonical
    /// 42-byte register-block layout.
    ///
    /// Reserved bytes and unused bits are zeroed. The mutable `t_fine`
    /// compensation intermediate is deliberately excluded, so the result is
    /// stable before and after measurements and can identify sensor changes.
    #[must_use]
    pub fn canonical_coefficient_bytes(&self) -> [u8; CALIBRATION_DATA_LEN] {
        let mut bytes = [0_u8; CALIBRATION_DATA_LEN];
        bytes[0..2].copy_from_slice(&self.par_t2.to_le_bytes());
        bytes[2] = self.par_t3.to_ne_bytes()[0];
        bytes[4..6].copy_from_slice(&self.par_p1.to_le_bytes());
        bytes[6..8].copy_from_slice(&self.par_p2.to_le_bytes());
        bytes[8] = self.par_p3.to_ne_bytes()[0];
        bytes[10..12].copy_from_slice(&self.par_p4.to_le_bytes());
        bytes[12..14].copy_from_slice(&self.par_p5.to_le_bytes());
        bytes[14] = self.par_p7.to_ne_bytes()[0];
        bytes[15] = self.par_p6.to_ne_bytes()[0];
        bytes[18..20].copy_from_slice(&self.par_p8.to_le_bytes());
        bytes[20..22].copy_from_slice(&self.par_p9.to_le_bytes());
        bytes[22] = self.par_p10;
        bytes[23] = (self.par_h2 >> 4).to_le_bytes()[0];
        bytes[24] =
            ((self.par_h2 & 0x0f).to_le_bytes()[0] << 4) | (self.par_h1 & 0x0f).to_le_bytes()[0];
        bytes[25] = (self.par_h1 >> 4).to_le_bytes()[0];
        bytes[26] = self.par_h3.to_ne_bytes()[0];
        bytes[27] = self.par_h4.to_ne_bytes()[0];
        bytes[28] = self.par_h5.to_ne_bytes()[0];
        bytes[29] = self.par_h6;
        bytes[30] = self.par_h7.to_ne_bytes()[0];
        bytes[31..33].copy_from_slice(&self.par_t1.to_le_bytes());
        bytes[33..35].copy_from_slice(&self.par_gh2.to_le_bytes());
        bytes[35] = self.par_gh1.to_ne_bytes()[0];
        bytes[36] = self.par_gh3.to_ne_bytes()[0];
        bytes[37] = self.res_heat_val.to_ne_bytes()[0];
        bytes[39] = (self.res_heat_range & 0x03) << 4;
        bytes[41] = self.range_sw_err.to_ne_bytes()[0] << 4;
        bytes
    }

    /// Stable FNV-1a 64-bit fingerprint of the normalized coefficients.
    ///
    /// Prefer the driver's `calibration_fingerprint()` method when the exact
    /// raw calibration register image is available. This normalized helper is
    /// useful for coefficient comparisons, but intentionally omits reserved
    /// register bits and bytes.
    #[must_use]
    pub fn coefficient_fingerprint(&self) -> u64 {
        calibration_register_fingerprint(&self.canonical_coefficient_bytes())
    }
}

/// Calculate an FNV-1a 64-bit fingerprint over exact calibration bytes.
///
/// This deterministic checksum is intended to detect a changed sensor. It is
/// not cryptographic and is not a globally unique sensor serial number.
#[must_use]
pub fn calibration_register_fingerprint(bytes: &[u8; CALIBRATION_DATA_LEN]) -> u64 {
    const OFFSET_BASIS: u64 = 0xcbf2_9ce4_8422_2325;
    const PRIME: u64 = 0x0000_0100_0000_01b3;

    let mut hash = OFFSET_BASIS;
    for &byte in bytes {
        hash ^= u64::from(byte);
        hash = hash.wrapping_mul(PRIME);
    }
    hash
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn calibration_parsing_matches_bosch_byte_layout() {
        let bytes = core::array::from_fn::<_, CALIBRATION_DATA_LEN, _>(|index| {
            u8::try_from(index).unwrap_or(0)
        });
        let calibration = CalibrationData::from_register_bytes(&bytes);

        assert_eq!(calibration.par_t2, 0x0100);
        assert_eq!(calibration.par_t3, 2);
        assert_eq!(calibration.par_p1, 0x0504);
        assert_eq!(calibration.par_p2, 0x0706);
        assert_eq!(calibration.par_p7, 14);
        assert_eq!(calibration.par_p6, 15);
        assert_eq!(calibration.par_p10, 22);
        assert_eq!(calibration.par_h1, 0x0198);
        assert_eq!(calibration.par_h2, 0x0171);
        assert_eq!(calibration.par_t1, 0x201f);
        assert_eq!(calibration.par_gh2, 0x2221);
        assert_eq!(calibration.res_heat_val, 37);
        assert_eq!(calibration.res_heat_range, 2);
        assert_eq!(calibration.range_sw_err, 2);
        assert_eq!(calibration.temperature_fine(), 0);
    }

    #[test]
    fn signed_calibration_fields_preserve_twos_complement() {
        let mut bytes = [0_u8; CALIBRATION_DATA_LEN];
        bytes[0] = 0xfe;
        bytes[1] = 0xff;
        bytes[2] = 0x80;
        bytes[33] = 0x34;
        bytes[34] = 0x80;
        bytes[37] = 0xff;
        bytes[41] = 0xf0;

        let calibration = CalibrationData::from_register_bytes(&bytes);
        assert_eq!(calibration.par_t2, -2);
        assert_eq!(calibration.par_t3, -128);
        assert_eq!(calibration.par_gh2, -32_716);
        assert_eq!(calibration.res_heat_val, -1);
        assert_eq!(calibration.range_sw_err, -1);
    }

    #[test]
    fn canonical_calibration_bytes_round_trip_and_ignore_temperature_fine() {
        let mut bytes = core::array::from_fn::<_, CALIBRATION_DATA_LEN, _>(|index| {
            u8::try_from(index).unwrap_or(0)
        });
        let mut calibration = CalibrationData::from_register_bytes(&bytes);
        let canonical = calibration.canonical_coefficient_bytes();

        assert_eq!(
            CalibrationData::from_register_bytes(&canonical),
            calibration
        );
        bytes[3] ^= 0xff;
        bytes[9] ^= 0xff;
        assert_eq!(
            CalibrationData::from_register_bytes(&bytes).canonical_coefficient_bytes(),
            canonical
        );

        let fingerprint = calibration.coefficient_fingerprint();
        calibration.t_fine = 123_456;
        assert_eq!(calibration.canonical_coefficient_bytes(), canonical);
        assert_eq!(calibration.coefficient_fingerprint(), fingerprint);
        assert_eq!(fingerprint, 0x5a5b_240a_eb30_385b);
    }

    #[test]
    fn raw_calibration_fingerprint_includes_reserved_bytes() {
        let mut first = [0_u8; CALIBRATION_DATA_LEN];
        let mut second = first;
        second[3] = 0xa5;

        assert_ne!(
            calibration_register_fingerprint(&first),
            calibration_register_fingerprint(&second)
        );
        first[3] = 0xa5;
        assert_eq!(
            calibration_register_fingerprint(&first),
            calibration_register_fingerprint(&second)
        );
    }

    #[test]
    fn fixed_measurement_float_accessors_only_scale_units() {
        let data = FixedMeasurement {
            temperature: 2_534,
            pressure: 101_325,
            humidity: 45_678,
            gas_resistance: 123_456,
        };

        assert!((data.temperature_celsius() - 25.34).abs() < f32::EPSILON);
        assert!((data.pressure_pascals() - 101_325.0).abs() < f32::EPSILON);
        assert!((data.pressure_hectopascals() - 1_013.25).abs() < f32::EPSILON);
        assert!((data.humidity_percent() - 45.678).abs() < f32::EPSILON);
        assert!((data.gas_resistance_ohms() - 123_456.0).abs() < f32::EPSILON);
    }

    #[test]
    fn typed_register_values_round_trip() {
        for value in 0..=5 {
            let setting = Oversampling::from_register(value).unwrap();
            assert_eq!(setting.register_value(), value);
        }
        assert_eq!(Oversampling::from_register(6), None);

        for value in 0..=8 {
            let setting = StandbyTime::from_register(value).unwrap();
            assert_eq!(setting.register_value(), value);
        }
        assert_eq!(StandbyTime::from_register(9), None);
    }
}
