// This floating-point calculation core is derived from Bosch Sensortec's
// BME68x SensorAPI v4.4.8.
// Copyright (c) 2023 Bosch Sensortec GmbH. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause

//! Bosch-compatible single-precision compensation calculations.
//!
//! This module reproduces the `BME68X_USE_FPU` calculation path from Bosch's
//! `BME68x` `SensorAPI` v4.4.8. It is separate from the crate's default exact
//! fixed-point path and is available with the `float` Cargo feature.
//!
//! Pressure and humidity depend on the temperature-fine intermediate. Call
//! [`compensate_temperature`] first for every sensor field, or use
//! [`compensate`], which performs the calculations in the required order.

// These casts deliberately reproduce conversions in Bosch's C reference.
// Raw ADC and gas-range inputs are constrained by the sensor register widths.
#![allow(
    clippy::cast_possible_truncation,
    clippy::cast_precision_loss,
    clippy::cast_sign_loss
)]

use crate::{
    CalibrationData, RawMeasurement, Variant, CALIBRATION_BLOCK_1_LEN, CALIBRATION_BLOCK_2_LEN,
    CALIBRATION_BLOCK_3_LEN, CALIBRATION_DATA_LEN,
};

const LOW_GAS_RANGE_CORRECTION_1: [f32; 16] = [
    0.0, 0.0, 0.0, 0.0, 0.0, -1.0, 0.0, -0.8, 0.0, 0.0, -0.2, -0.5, 0.0, -1.0, 0.0, 0.0,
];

const LOW_GAS_RANGE_CORRECTION_2: [f32; 16] = [
    0.0, 0.0, 0.0, 0.0, 0.1, 0.7, 0.0, -0.8, -0.1, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
];

/// Factory coefficients plus the floating-point temperature intermediate.
///
/// The coefficient payload is shared with [`CalibrationData`], while
/// `temperature_fine` has the `float` type used by Bosch's FPU build. Keeping
/// this state separate prevents floating-point compensation from changing the
/// default fixed-point driver's state or results.
#[derive(Clone, Copy, Debug, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct FloatCalibrationData {
    coefficients: CalibrationData,
    temperature_fine: f32,
}

impl FloatCalibrationData {
    /// Create floating-point compensation state from parsed coefficients.
    #[must_use]
    pub const fn new(coefficients: CalibrationData) -> Self {
        Self {
            coefficients,
            temperature_fine: 0.0,
        }
    }

    /// Create state with an explicit Bosch `t_fine` intermediate.
    ///
    /// Most applications should use [`Self::new`] and let
    /// [`compensate_temperature`] update this value for every measurement.
    #[must_use]
    pub const fn from_parts(coefficients: CalibrationData, temperature_fine: f32) -> Self {
        Self {
            coefficients,
            temperature_fine,
        }
    }

    /// Parse the concatenated 42-byte Bosch calibration register image.
    #[must_use]
    pub fn from_register_bytes(bytes: &[u8; CALIBRATION_DATA_LEN]) -> Self {
        Self::new(CalibrationData::from_register_bytes(bytes))
    }

    /// Parse the three discontiguous Bosch calibration register blocks.
    #[must_use]
    pub fn from_register_blocks(
        block_1: &[u8; CALIBRATION_BLOCK_1_LEN],
        block_2: &[u8; CALIBRATION_BLOCK_2_LEN],
        block_3: &[u8; CALIBRATION_BLOCK_3_LEN],
    ) -> Self {
        Self::new(CalibrationData::from_register_blocks(
            block_1, block_2, block_3,
        ))
    }

    /// Return the factory coefficients used by this calculation state.
    #[must_use]
    pub const fn coefficients(&self) -> &CalibrationData {
        &self.coefficients
    }

    /// Mutably access the factory coefficients used by this calculation state.
    ///
    /// Changing coefficients invalidates any previously calculated
    /// temperature-fine value; call [`compensate_temperature`] again before
    /// calculating pressure or humidity.
    pub fn coefficients_mut(&mut self) -> &mut CalibrationData {
        &mut self.coefficients
    }

    /// Return Bosch's current floating-point `t_fine` intermediate.
    #[must_use]
    pub const fn temperature_fine(&self) -> f32 {
        self.temperature_fine
    }

    /// Replace Bosch's floating-point `t_fine` intermediate.
    ///
    /// This is primarily useful for differential testing and state transfer.
    /// Normal measurement code should call [`compensate_temperature`].
    pub fn set_temperature_fine(&mut self, temperature_fine: f32) {
        self.temperature_fine = temperature_fine;
    }

    /// Consume the floating-point state and return its factory coefficients.
    #[must_use]
    pub const fn into_coefficients(self) -> CalibrationData {
        self.coefficients
    }
}

impl Default for FloatCalibrationData {
    fn default() -> Self {
        Self::new(CalibrationData::default())
    }
}

impl From<CalibrationData> for FloatCalibrationData {
    fn from(coefficients: CalibrationData) -> Self {
        Self::new(coefficients)
    }
}

impl From<&CalibrationData> for FloatCalibrationData {
    fn from(coefficients: &CalibrationData) -> Self {
        Self::new(*coefficients)
    }
}

/// One measurement in Bosch's `BME68X_USE_FPU` output units.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct FloatMeasurement {
    /// Temperature in degrees Celsius.
    pub temperature: f32,
    /// Pressure in pascals.
    pub pressure: f32,
    /// Relative humidity in percent RH.
    pub humidity: f32,
    /// Gas resistance in ohms.
    pub gas_resistance: f32,
}

/// Compensate a raw temperature sample in degrees Celsius.
///
/// This updates the Bosch `t_fine` value in `calibration`. It must precede
/// pressure and humidity compensation for the same measurement field.
#[must_use]
pub fn compensate_temperature(temperature_adc: u32, calibration: &mut FloatCalibrationData) -> f32 {
    let coefficients = &calibration.coefficients;
    let var1 = ((temperature_adc as f32 / 16_384.0) - (f32::from(coefficients.par_t1) / 1_024.0))
        * f32::from(coefficients.par_t2);
    let normalized_adc =
        (temperature_adc as f32 / 131_072.0) - (f32::from(coefficients.par_t1) / 8_192.0);
    let var2 = normalized_adc * normalized_adc * (f32::from(coefficients.par_t3) * 16.0);

    calibration.temperature_fine = var1 + var2;
    calibration.temperature_fine / 5_120.0
}

/// Compensate a raw pressure sample in pascals.
///
/// [`compensate_temperature`] must have updated `calibration` from the same
/// measurement field first. As in Bosch's reference, a pressure divisor whose
/// truncation to an integer is zero produces `0.0` rather than dividing.
#[must_use]
pub fn compensate_pressure(pressure_adc: u32, calibration: &FloatCalibrationData) -> f32 {
    let coefficients = &calibration.coefficients;
    let mut var1 = calibration.temperature_fine / 2.0 - 64_000.0;
    let mut var2 = var1 * var1 * (f32::from(coefficients.par_p6) / 131_072.0);
    var2 += var1 * f32::from(coefficients.par_p5) * 2.0;
    var2 = var2 / 4.0 + f32::from(coefficients.par_p4) * 65_536.0;
    var1 = ((f32::from(coefficients.par_p3) * var1 * var1 / 16_384.0)
        + f32::from(coefficients.par_p2) * var1)
        / 524_288.0;
    var1 = (1.0 + var1 / 32_768.0) * f32::from(coefficients.par_p1);

    let mut compensated_pressure = 1_048_576.0 - pressure_adc as f32;
    if var1 as i32 != 0 {
        compensated_pressure = ((compensated_pressure - var2 / 4_096.0) * 6_250.0) / var1;
        var1 = f32::from(coefficients.par_p9) * compensated_pressure * compensated_pressure
            / 2_147_483_648.0;
        var2 = compensated_pressure * (f32::from(coefficients.par_p8) / 32_768.0);
        let pressure_scaled = compensated_pressure / 256.0;
        let var3 = pressure_scaled
            * pressure_scaled
            * pressure_scaled
            * (f32::from(coefficients.par_p10) / 131_072.0);
        compensated_pressure +=
            (var1 + var2 + var3 + f32::from(coefficients.par_p7) * 128.0) / 16.0;
    } else {
        compensated_pressure = 0.0;
    }

    compensated_pressure
}

/// Compensate a raw humidity sample in percent relative humidity.
///
/// [`compensate_temperature`] must have updated `calibration` from the same
/// measurement field first. The result is capped to Bosch's `0.0..=100.0`
/// range.
#[must_use]
pub fn compensate_humidity(humidity_adc: u16, calibration: &FloatCalibrationData) -> f32 {
    let coefficients = &calibration.coefficients;
    let compensated_temperature = calibration.temperature_fine / 5_120.0;
    let var1 = f32::from(humidity_adc)
        - (f32::from(coefficients.par_h1) * 16.0
            + (f32::from(coefficients.par_h3) / 2.0) * compensated_temperature);
    let var2 = var1
        * ((f32::from(coefficients.par_h2) / 262_144.0)
            * (1.0
                + (f32::from(coefficients.par_h4) / 16_384.0) * compensated_temperature
                + (f32::from(coefficients.par_h5) / 1_048_576.0)
                    * compensated_temperature
                    * compensated_temperature));
    let var3 = f32::from(coefficients.par_h6) / 16_384.0;
    let var4 = f32::from(coefficients.par_h7) / 2_097_152.0;
    let compensated_humidity = var2 + (var3 + var4 * compensated_temperature) * var2 * var2;

    compensated_humidity.clamp(0.0, 100.0)
}

/// Calculate gas resistance for Bosch's low-gas variant, in ohms.
///
/// `gas_range` must be the four-bit sensor field (`0..=15`).
#[must_use]
pub fn compensate_gas_low(
    gas_resistance_adc: u16,
    gas_range: u8,
    calibration: &FloatCalibrationData,
) -> f32 {
    let index = usize::from(gas_range);
    let gas_range_factor = (1_u32 << gas_range) as f32;
    let var1 = 1_340.0 + 5.0 * f32::from(calibration.coefficients.range_sw_err);
    let var2 = var1 * (1.0 + LOW_GAS_RANGE_CORRECTION_1[index] / 100.0);
    let var3 = 1.0 + LOW_GAS_RANGE_CORRECTION_2[index] / 100.0;

    1.0 / (var3
        * 0.000_000_125
        * gas_range_factor
        * ((f32::from(gas_resistance_adc) - 512.0) / var2 + 1.0))
}

/// Calculate gas resistance for Bosch's high-gas variant, in ohms.
///
/// `gas_range` must be the four-bit sensor field (`0..=15`).
#[must_use]
pub fn compensate_gas_high(gas_resistance_adc: u16, gas_range: u8) -> f32 {
    let var1 = 262_144_u32 >> gas_range;
    let mut var2 = i32::from(gas_resistance_adc) - 512;
    var2 *= 3;
    var2 += 4_096;

    1_000_000.0 * var1 as f32 / var2 as f32
}

/// Calculate the heater-resistance register using Bosch's FPU formula.
///
/// `target_temperature` is in degrees Celsius and is capped to 400 °C.
/// `ambient_temperature` is in degrees Celsius. The valid factory-calibration
/// domain keeps the final C-compatible float-to-`u8` conversion in range.
#[must_use]
pub fn calculate_heater_resistance(
    target_temperature: u16,
    ambient_temperature: i8,
    calibration: &FloatCalibrationData,
) -> u8 {
    let coefficients = &calibration.coefficients;
    let target_temperature = target_temperature.min(400);
    let var1 = f32::from(coefficients.par_gh1) / 16.0 + 49.0;
    let var2 = (f32::from(coefficients.par_gh2) / 32_768.0) * 0.0005 + 0.00235;
    let var3 = f32::from(coefficients.par_gh3) / 1_024.0;
    let var4 = var1 * (1.0 + var2 * f32::from(target_temperature));
    let var5 = var4 + var3 * f32::from(ambient_temperature);
    let heater_resistance = 3.4
        * (var5
            * (4.0 / (4.0 + f32::from(coefficients.res_heat_range)))
            * (1.0 / (1.0 + f32::from(coefficients.res_heat_val) * 0.002))
            - 25.0);

    heater_resistance as u8
}

/// Compensate all channels from one raw sensor field using Bosch's FPU path.
///
/// Temperature is deliberately evaluated first so pressure and humidity use
/// the correct `t_fine` state.
#[must_use]
pub fn compensate(
    raw: RawMeasurement,
    variant: Variant,
    calibration: &mut FloatCalibrationData,
) -> FloatMeasurement {
    let temperature = compensate_temperature(raw.temperature_adc, calibration);
    let pressure = compensate_pressure(raw.pressure_adc, calibration);
    let humidity = compensate_humidity(raw.humidity_adc, calibration);
    let gas_resistance = match variant {
        Variant::GasLow => compensate_gas_low(raw.gas_resistance_adc, raw.gas_range, calibration),
        Variant::GasHigh => compensate_gas_high(raw.gas_resistance_adc, raw.gas_range),
    };

    FloatMeasurement {
        temperature,
        pressure,
        humidity,
        gas_resistance,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn reference_calibration() -> FloatCalibrationData {
        FloatCalibrationData::new(CalibrationData {
            par_h1: 824,
            par_h2: 1_019,
            par_h3: 0,
            par_h4: 45,
            par_h5: 20,
            par_h6: 120,
            par_h7: -100,
            par_gh1: -30,
            par_gh2: -2_500,
            par_gh3: 4,
            par_t1: 27_504,
            par_t2: 26_435,
            par_t3: 3,
            par_p1: 36_477,
            par_p2: -10_685,
            par_p3: 88,
            par_p4: 10_485,
            par_p5: -53,
            par_p6: 30,
            par_p7: 7,
            par_p8: -14_600,
            par_p9: 6_000,
            par_p10: 30,
            res_heat_range: 1,
            res_heat_val: 30,
            range_sw_err: -2,
            ..CalibrationData::default()
        })
    }

    #[test]
    fn temperature_updates_state_before_pressure_and_humidity() {
        let mut calibration = reference_calibration();
        assert_eq!(calibration.temperature_fine().to_bits(), 0);

        let temperature = compensate_temperature(519_888, &mut calibration);
        assert_ne!(calibration.temperature_fine().to_bits(), 0);
        assert_eq!(
            temperature.to_bits(),
            (calibration.temperature_fine() / 5_120.0).to_bits()
        );
        assert!(compensate_pressure(415_148, &calibration).is_finite());
        assert!((0.0..=100.0).contains(&compensate_humidity(32_257, &calibration)));
    }

    #[test]
    fn aggregate_compensation_uses_bosch_units_and_variant() {
        let mut calibration = reference_calibration();
        let measurement = compensate(
            RawMeasurement {
                temperature_adc: 519_888,
                pressure_adc: 415_148,
                humidity_adc: 32_257,
                gas_resistance_adc: 700,
                gas_range: 5,
            },
            Variant::GasHigh,
            &mut calibration,
        );

        assert!((-40.0..=85.0).contains(&measurement.temperature));
        assert!(measurement.pressure > 0.0);
        assert!((0.0..=100.0).contains(&measurement.humidity));
        assert!(measurement.gas_resistance > 0.0);
    }

    #[test]
    fn humidity_is_capped_to_reference_range() {
        let mut calibration = reference_calibration();
        let _ = compensate_temperature(519_888, &mut calibration);
        assert_eq!(
            compensate_humidity(0, &calibration).to_bits(),
            0.0_f32.to_bits()
        );
        assert_eq!(
            compensate_humidity(u16::MAX, &calibration).to_bits(),
            100.0_f32.to_bits()
        );
    }

    #[test]
    fn high_gas_formula_retains_single_precision_result() {
        assert_eq!(compensate_gas_high(700, 5).to_bits(), 1_238_800_287);
    }

    #[test]
    fn heater_target_is_capped_at_400_celsius() {
        let calibration = reference_calibration();
        assert_eq!(
            calculate_heater_resistance(400, 25, &calibration),
            calculate_heater_resistance(u16::MAX, 25, &calibration)
        );
    }
}
