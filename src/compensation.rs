// This fixed-point calculation core is derived from Bosch Sensortec's
// BME68x SensorAPI v4.4.8.
// Copyright (c) 2023 Bosch Sensortec GmbH. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause

//! Bosch-compatible fixed-point compensation and timing calculations.
//!
//! Every function in this module uses the same integer units and truncation
//! points as the non-FPU build of Bosch's `BME68x` `SensorAPI` v4.4.8. These
//! low-level functions deliberately do not clamp raw ADC values or calibration
//! coefficients beyond the places where the Bosch implementation does so.

// These conversions deliberately reproduce the fixed-width casts in Bosch's
// C reference. Their valid sensor domains are documented on each API.
#![allow(
    clippy::cast_possible_truncation,
    clippy::cast_possible_wrap,
    clippy::cast_sign_loss
)]

use crate::types::{
    CalibrationData, Configuration, FixedMeasurement, OperationMode, RawMeasurement, Variant,
};

const GAS_RANGE_LOOKUP_1: [u32; 16] = [
    2_147_483_647,
    2_147_483_647,
    2_147_483_647,
    2_147_483_647,
    2_147_483_647,
    2_126_008_810,
    2_147_483_647,
    2_130_303_777,
    2_147_483_647,
    2_147_483_647,
    2_143_188_679,
    2_136_746_228,
    2_147_483_647,
    2_126_008_810,
    2_147_483_647,
    2_147_483_647,
];

const GAS_RANGE_LOOKUP_2: [u32; 16] = [
    4_096_000_000,
    2_048_000_000,
    1_024_000_000,
    512_000_000,
    255_744_255,
    127_110_228,
    64_000_000,
    32_258_064,
    16_016_016,
    8_000_000,
    4_000_000,
    2_000_000,
    1_000_000,
    500_000,
    250_000,
    125_000,
];

/// Compensates a raw temperature sample.
///
/// The returned unit is one hundredth of a degree Celsius. This operation
/// updates `t_fine` inside `calibration`; call it before
/// [`compensate_pressure`] and [`compensate_humidity`] for the same
/// measurement field.
#[must_use]
pub fn compensate_temperature(temperature_adc: u32, calibration: &mut CalibrationData) -> i16 {
    // The ADC is 20-bit, so Bosch's initial u32-to-i32 conversion is exact for
    // every value the sensor can produce.
    let var1 = i64::from(temperature_adc) / 8 - (i64::from(calibration.par_t1) * 2);
    let var2 = (var1 * i64::from(calibration.par_t2)) >> 11;
    let half_var1 = var1 >> 1;
    let var3 = (((half_var1 * half_var1) >> 12) * (i64::from(calibration.par_t3) * 16)) >> 14;

    calibration.t_fine = (var2 + var3) as i32;
    ((calibration.t_fine.wrapping_mul(5).wrapping_add(128)) >> 8) as i16
}

/// Compensates a raw pressure sample using the most recent temperature.
///
/// The return value is in pascals. [`compensate_temperature`] must have
/// updated the calibration's `t_fine` value for the same sensor field first.
#[must_use]
pub fn compensate_pressure(pressure_adc: u32, calibration: &CalibrationData) -> u32 {
    const PRESSURE_OVERFLOW_CHECK: i32 = 0x4000_0000;

    let mut var1 = (calibration.t_fine >> 1).wrapping_sub(64_000);
    let quarter_var1 = var1 >> 2;
    let mut var2 = quarter_var1
        .wrapping_mul(quarter_var1)
        .wrapping_shr(11)
        .wrapping_mul(i32::from(calibration.par_p6))
        .wrapping_shr(2);
    var2 = var2.wrapping_add(
        var1.wrapping_mul(i32::from(calibration.par_p5))
            .wrapping_shl(1),
    );
    var2 = (var2 >> 2).wrapping_add(i32::from(calibration.par_p4).wrapping_shl(16));

    var1 = quarter_var1
        .wrapping_mul(quarter_var1)
        .wrapping_shr(13)
        .wrapping_mul(i32::from(calibration.par_p3).wrapping_shl(5))
        .wrapping_shr(3)
        .wrapping_add(
            i32::from(calibration.par_p2)
                .wrapping_mul(var1)
                .wrapping_shr(1),
        );
    var1 >>= 18;
    var1 = 32_768_i32
        .wrapping_add(var1)
        .wrapping_mul(i32::from(calibration.par_p1))
        >> 15;

    // The original C expression performs this subtraction and multiplication
    // as u32 before assigning the two's-complement result back to i32.
    let mut pressure_comp = 1_048_576_u32
        .wrapping_sub(pressure_adc)
        .wrapping_sub((var2 >> 12) as u32)
        .wrapping_mul(3_125) as i32;

    pressure_comp = if pressure_comp >= PRESSURE_OVERFLOW_CHECK {
        (pressure_comp / var1).wrapping_shl(1)
    } else {
        pressure_comp.wrapping_shl(1) / var1
    };

    var1 = i32::from(calibration.par_p9)
        .wrapping_mul(
            (pressure_comp >> 3)
                .wrapping_mul(pressure_comp >> 3)
                .wrapping_shr(13),
        )
        .wrapping_shr(12);
    var2 = (pressure_comp >> 2)
        .wrapping_mul(i32::from(calibration.par_p8))
        .wrapping_shr(13);
    let pressure_over_256 = pressure_comp >> 8;
    let var3 = pressure_over_256
        .wrapping_mul(pressure_over_256)
        .wrapping_mul(pressure_over_256)
        .wrapping_mul(i32::from(calibration.par_p10))
        .wrapping_shr(17);

    pressure_comp = pressure_comp.wrapping_add(
        var1.wrapping_add(var2)
            .wrapping_add(var3)
            .wrapping_add(i32::from(calibration.par_p7).wrapping_shl(7))
            >> 4,
    );

    pressure_comp as u32
}

/// Compensates a raw humidity sample using the most recent temperature.
///
/// The return value is relative humidity in thousandths of a percent RH and is
/// capped to the Bosch range `0..=100_000`. [`compensate_temperature`] must be
/// called for the same measurement field first.
#[must_use]
pub fn compensate_humidity(humidity_adc: u16, calibration: &CalibrationData) -> u32 {
    let temperature_scaled = calibration.t_fine.wrapping_mul(5).wrapping_add(128) >> 8;
    let var1 = i32::from(humidity_adc)
        .wrapping_sub(i32::from(calibration.par_h1).wrapping_mul(16))
        .wrapping_sub((temperature_scaled.wrapping_mul(i32::from(calibration.par_h3)) / 100) >> 1);

    let linear_temperature = temperature_scaled.wrapping_mul(i32::from(calibration.par_h4)) / 100;
    let quadratic_temperature = temperature_scaled
        .wrapping_mul(temperature_scaled.wrapping_mul(i32::from(calibration.par_h5)) / 100)
        .wrapping_shr(6)
        / 100;
    let var2 = i32::from(calibration.par_h2).wrapping_mul(
        linear_temperature
            .wrapping_add(quadratic_temperature)
            .wrapping_add(1 << 14),
    ) >> 10;
    let var3 = var1.wrapping_mul(var2);
    let var4 = i32::from(calibration.par_h6)
        .wrapping_shl(7)
        .wrapping_add(temperature_scaled.wrapping_mul(i32::from(calibration.par_h7)) / 100)
        >> 4;
    let var3_scaled = var3 >> 14;
    let var5 = var3_scaled.wrapping_mul(var3_scaled) >> 10;
    let var6 = var4.wrapping_mul(var5) >> 1;
    let calculated = (var3.wrapping_add(var6) >> 10).wrapping_mul(1_000) >> 12;

    calculated.clamp(0, 100_000) as u32
}

/// Calculates gas resistance for the Bosch low-gas variant.
///
/// The return value is in ohms. `gas_range` must be the sensor's four-bit gas
/// range field (`0..=15`).
#[must_use]
pub fn compensate_gas_low(
    gas_resistance_adc: u16,
    gas_range: u8,
    calibration: &CalibrationData,
) -> u32 {
    let index = usize::from(gas_range);
    let var1 = ((1_340_i64 + 5 * i64::from(calibration.range_sw_err))
        * i64::from(GAS_RANGE_LOOKUP_1[index]))
        >> 16;
    let var2 = ((i64::from(gas_resistance_adc) << 15) - 16_777_216) + var1;
    let var3 = (i64::from(GAS_RANGE_LOOKUP_2[index]) * var1) >> 9;

    ((var3 + (var2 >> 1)) / var2) as u32
}

/// Calculates gas resistance for the Bosch high-gas variant.
///
/// The return value is in ohms. `gas_range` must be the sensor's four-bit gas
/// range field (`0..=15`).
#[must_use]
pub fn compensate_gas_high(gas_resistance_adc: u16, gas_range: u8) -> u32 {
    let var1 = 262_144_u32 >> gas_range;
    let var2 = i32::from(gas_resistance_adc)
        .wrapping_sub(512)
        .wrapping_mul(3)
        .wrapping_add(4_096) as u32;

    (10_000_u32.wrapping_mul(var1) / var2).wrapping_mul(100)
}

/// Calculates the heater-resistance register value.
///
/// `target_temperature` is in degrees Celsius and is capped to 400 °C exactly
/// as in Bosch's integer implementation. `ambient_temperature` is in degrees
/// Celsius.
#[must_use]
pub fn calculate_heater_resistance(
    target_temperature: u16,
    ambient_temperature: i8,
    calibration: &CalibrationData,
) -> u8 {
    let target_temperature = i32::from(target_temperature.min(400));
    let var1 = (i32::from(ambient_temperature).wrapping_mul(i32::from(calibration.par_gh3))
        / 1_000)
        .wrapping_mul(256);
    let heater_temperature_term = (i32::from(calibration.par_gh2)
        .wrapping_add(154_009)
        .wrapping_mul(target_temperature)
        .wrapping_mul(5)
        / 100)
        .wrapping_add(3_276_800)
        / 10;
    let var2 = i32::from(calibration.par_gh1)
        .wrapping_add(784)
        .wrapping_mul(heater_temperature_term);
    let var3 = var1.wrapping_add(var2 / 2);
    let var4 = var3 / i32::from(calibration.res_heat_range + 4);
    let var5 = 131_i32
        .wrapping_mul(i32::from(calibration.res_heat_val))
        .wrapping_add(65_536);
    let heater_resistance_x100 = (var4 / var5).wrapping_sub(250).wrapping_mul(34);

    ((heater_resistance_x100.wrapping_add(50)) / 100) as u8
}

/// Encodes a heater step duration in milliseconds for a gas-wait register.
#[must_use]
pub const fn encode_gas_wait(mut duration_ms: u16) -> u8 {
    if duration_ms >= 0x0fc0 {
        return 0xff;
    }

    let mut factor = 0_u8;
    while duration_ms > 0x3f {
        duration_ms /= 4;
        factor += 1;
    }

    duration_ms as u8 + factor * 64
}

/// Decode the exact represented gas-wait duration in milliseconds.
///
/// Encoding rounds down, so this can be lower than the duration originally
/// passed to [`encode_gas_wait`].
#[must_use]
pub fn decode_gas_wait_ms(register: u8) -> u16 {
    let mantissa = u16::from(register & 0x3f);
    let factor = u32::from(register >> 6) * 2;
    mantissa << factor
}

/// Encodes the parallel-mode shared heater duration in milliseconds.
///
/// This uses the sensor's 0.477 ms step size before applying the same
/// exponent/mantissa encoding used by [`encode_gas_wait`].
#[must_use]
pub const fn encode_shared_heater_duration(duration_ms: u16) -> u8 {
    if duration_ms >= 0x0783 {
        return 0xff;
    }

    let mut duration_steps = (duration_ms as u32 * 1_000 / 477) as u16;
    let mut factor = 0_u8;
    while duration_steps > 0x3f {
        duration_steps >>= 2;
        factor += 1;
    }

    duration_steps as u8 + factor * 64
}

/// Decode the exact represented parallel shared wait in microseconds.
///
/// The register quantum is 477 microseconds and encoding rounds down. Use the
/// read-back register value, rather than the requested millisecond duration,
/// when calculating exact profile timing.
#[must_use]
pub fn decode_shared_heater_duration_us(register: u8) -> u32 {
    let mantissa = u32::from(register & 0x3f);
    let factor = u32::from(register >> 6) * 2;
    (mantissa << factor) * 477
}

/// Exact duration represented by one parallel-mode profile step.
///
/// A nonzero `repetition_multiplier` repeats the shared-wait-plus-TPHG period.
/// Bosch defines zero as a special case that skips the shared wait and performs
/// exactly one TPHG conversion.
#[must_use]
pub fn parallel_step_duration_us(
    repetition_multiplier: u8,
    shared_duration_register: u8,
    tphg_duration_us: u32,
) -> u32 {
    if repetition_multiplier == 0 {
        tphg_duration_us
    } else {
        u32::from(repetition_multiplier).saturating_mul(
            decode_shared_heater_duration_us(shared_duration_register)
                .saturating_add(tphg_duration_us),
        )
    }
}

/// Returns Bosch's measurement duration in microseconds.
///
/// Heater duration is intentionally not included: this is the duration of the
/// T/P/H conversion, sensor switching, gas conversion, and (except in parallel
/// mode) the 1 ms wake-up interval.
#[must_use]
pub const fn measurement_duration_us(mode: OperationMode, configuration: &Configuration) -> u32 {
    let measurement_cycles = configuration.temperature_oversampling.measurement_cycles()
        + configuration.pressure_oversampling.measurement_cycles()
        + configuration.humidity_oversampling.measurement_cycles();
    let duration = measurement_cycles * 1_963 + 477 * 4 + 477 * 5;

    if matches!(mode, OperationMode::Parallel) {
        duration
    } else {
        duration + 1_000
    }
}

/// Compensates all channels from one raw measurement field.
///
/// Temperature is evaluated first so pressure and humidity use the correct
/// `t_fine` intermediate. Gas resistance is selected by the sensor variant.
#[must_use]
pub fn compensate(
    raw: RawMeasurement,
    variant: Variant,
    calibration: &mut CalibrationData,
) -> FixedMeasurement {
    let compensated_temperature = compensate_temperature(raw.temperature_adc, calibration);
    let compensated_pressure = compensate_pressure(raw.pressure_adc, calibration);
    let compensated_humidity = compensate_humidity(raw.humidity_adc, calibration);
    let compensated_gas_resistance = match variant {
        Variant::GasLow => compensate_gas_low(raw.gas_resistance_adc, raw.gas_range, calibration),
        Variant::GasHigh => compensate_gas_high(raw.gas_resistance_adc, raw.gas_range),
    };

    FixedMeasurement {
        temperature: compensated_temperature,
        pressure: compensated_pressure,
        humidity: compensated_humidity,
        gas_resistance: compensated_gas_resistance,
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::types::{Filter, Oversampling, StandbyTime};

    fn reference_calibration() -> CalibrationData {
        CalibrationData {
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
            par_t1: 26_000,
            par_t2: 26_470,
            par_t3: 3,
            par_p1: 36_400,
            par_p2: -10_685,
            par_p3: 88,
            par_p4: 10_000,
            par_p5: -200,
            par_p6: 30,
            par_p7: -50,
            par_p8: -7_000,
            par_p9: 6_000,
            par_p10: 30,
            t_fine: 0,
            res_heat_range: 1,
            res_heat_val: 40,
            range_sw_err: -2,
        }
    }

    #[test]
    fn gas_wait_matches_bosch_boundaries() {
        assert_eq!(encode_gas_wait(0), 0x00);
        assert_eq!(encode_gas_wait(63), 0x3f);
        assert_eq!(encode_gas_wait(64), 0x50);
        assert_eq!(encode_gas_wait(252), 0x7f);
        assert_eq!(encode_gas_wait(253), 0x7f);
        assert_eq!(encode_gas_wait(1_008), 0xbf);
        assert_eq!(encode_gas_wait(4_031), 0xfe);
        assert_eq!(encode_gas_wait(4_032), 0xff);
        assert_eq!(encode_gas_wait(u16::MAX), 0xff);
        assert_eq!(decode_gas_wait_ms(0x00), 0);
        assert_eq!(decode_gas_wait_ms(0x3f), 63);
        assert_eq!(decode_gas_wait_ms(0x50), 64);
        assert_eq!(decode_gas_wait_ms(0xff), 4_032);
    }

    #[test]
    fn shared_heater_duration_matches_bosch_boundaries() {
        assert_eq!(encode_shared_heater_duration(0), 0x00);
        assert_eq!(encode_shared_heater_duration(1), 0x02);
        assert_eq!(encode_shared_heater_duration(30), 0x3e);
        assert_eq!(encode_shared_heater_duration(31), 0x50);
        assert_eq!(encode_shared_heater_duration(1_922), 0xfe);
        assert_eq!(encode_shared_heater_duration(1_923), 0xff);
        assert_eq!(encode_shared_heater_duration(u16::MAX), 0xff);
        assert_eq!(decode_shared_heater_duration_us(0x00), 0);
        assert_eq!(decode_shared_heater_duration_us(0x73), 97_308);
        assert_eq!(decode_shared_heater_duration_us(0xff), 1_923_264);
        assert_eq!(
            parallel_step_duration_us(5, encode_shared_heater_duration(99), 41_590),
            694_490
        );
        assert_eq!(
            parallel_step_duration_us(0, encode_shared_heater_duration(99), 41_590),
            41_590
        );
    }

    #[test]
    fn duration_matches_bosch_integer_equation() {
        let configuration = Configuration {
            humidity_oversampling: Oversampling::X1,
            temperature_oversampling: Oversampling::X2,
            pressure_oversampling: Oversampling::X16,
            filter: Filter::Size7,
            standby_time: StandbyTime::Millis20,
        };

        assert_eq!(
            measurement_duration_us(OperationMode::Parallel, &configuration),
            41_590
        );
        assert_eq!(
            measurement_duration_us(OperationMode::Forced, &configuration),
            42_590
        );
        assert_eq!(
            measurement_duration_us(OperationMode::Sequential, &configuration),
            42_590
        );
        assert_eq!(
            measurement_duration_us(OperationMode::Sleep, &configuration),
            42_590
        );
    }

    #[test]
    fn humidity_is_capped_to_physical_range() {
        let mut calibration = reference_calibration();
        calibration.t_fine = 128_000;
        assert!(compensate_humidity(0, &calibration) <= 100_000);
        assert_eq!(compensate_humidity(u16::MAX, &calibration), 100_000);
    }

    #[test]
    fn heater_target_is_capped_at_400_celsius() {
        let calibration = reference_calibration();
        assert_eq!(
            calculate_heater_resistance(400, 25, &calibration),
            calculate_heater_resistance(u16::MAX, 25, &calibration)
        );
    }

    #[test]
    fn compensation_pipeline_updates_temperature_fine() {
        let mut calibration = reference_calibration();
        let raw = RawMeasurement {
            temperature_adc: 519_888,
            pressure_adc: 364_576,
            humidity_adc: 30_000,
            gas_resistance_adc: 700,
            gas_range: 8,
        };

        let measurement = compensate(raw, Variant::GasLow, &mut calibration);
        assert_eq!(measurement.temperature, 3_279);
        assert_eq!(calibration.temperature_fine(), 167_871);
        assert_eq!(measurement.pressure, 91_655);
        assert_eq!(measurement.humidity, 100_000);
        assert_eq!(measurement.gas_resistance, 27_407);
        assert_eq!(compensate_gas_high(700, 8), 219_700);
        assert_eq!(calculate_heater_resistance(320, 25, &calibration), 121);
    }
}
