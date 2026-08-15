//! Differential tests against Bosch v4.4.8's floating-point C calculations.

use bme68x::float::{self, FloatCalibrationData};
use bme68x::{CalibrationData, RawMeasurement, Variant, CALIBRATION_DATA_LEN};

#[repr(C)]
#[derive(Clone, Copy, Debug, Default)]
struct OracleFloatCalibration {
    par_h1: u16,
    par_h2: u16,
    par_h3: i8,
    par_h4: i8,
    par_h5: i8,
    par_h6: u8,
    par_h7: i8,
    par_gh1: i8,
    par_gh2: i16,
    par_gh3: i8,
    par_t1: u16,
    par_t2: i16,
    par_t3: i8,
    par_p1: u16,
    par_p2: i16,
    par_p3: i8,
    par_p4: i16,
    par_p5: i16,
    par_p6: i8,
    par_p7: i8,
    par_p8: i16,
    par_p9: i16,
    par_p10: u8,
    temperature_fine: f32,
    res_heat_range: u8,
    res_heat_val: i8,
    range_sw_err: i8,
}

extern "C" {
    fn oracle_float_parse_calibration(
        bytes: *const u8,
        calibration: *mut OracleFloatCalibration,
    ) -> i8;
    fn oracle_float_temperature(adc: u32, calibration: *mut OracleFloatCalibration) -> f32;
    fn oracle_float_pressure(adc: u32, calibration: *const OracleFloatCalibration) -> f32;
    fn oracle_float_humidity(adc: u16, calibration: *const OracleFloatCalibration) -> f32;
    fn oracle_float_gas_low(adc: u16, range: u8, calibration: *const OracleFloatCalibration)
        -> f32;
    fn oracle_float_gas_high(adc: u16, range: u8) -> f32;
    fn oracle_float_heater_resistance(
        temperature: u16,
        ambient_temperature: i8,
        calibration: *const OracleFloatCalibration,
    ) -> u8;
}

fn calibration_bytes(seed: u32) -> [u8; CALIBRATION_DATA_LEN] {
    let mut bytes = [0_u8; CALIBRATION_DATA_LEN];
    let mut state = seed;
    for byte in &mut bytes {
        state = state.wrapping_mul(1_664_525).wrapping_add(1_013_904_223);
        *byte = state.to_be_bytes()[0];
    }

    // Keep the pressure divisor in a realistic non-zero domain and the heater
    // range inside its documented two-bit register representation.
    bytes[4] = 0x30;
    bytes[5] = 0x8e;
    bytes[39] &= 0x30;
    bytes
}

fn parse_bytes(
    bytes: &[u8; CALIBRATION_DATA_LEN],
) -> (FloatCalibrationData, OracleFloatCalibration) {
    let rust = FloatCalibrationData::from_register_bytes(bytes);
    let mut oracle = OracleFloatCalibration::default();
    // SAFETY: both pointers reference live values with the exact C layout and
    // remain valid for the duration of the call.
    let status = unsafe { oracle_float_parse_calibration(bytes.as_ptr(), &mut oracle) };
    assert_eq!(status, 0);
    (rust, oracle)
}

fn assert_coefficients_equal(rust: &CalibrationData, c: &OracleFloatCalibration) {
    assert_eq!(rust.par_h1, c.par_h1);
    assert_eq!(rust.par_h2, c.par_h2);
    assert_eq!(rust.par_h3, c.par_h3);
    assert_eq!(rust.par_h4, c.par_h4);
    assert_eq!(rust.par_h5, c.par_h5);
    assert_eq!(rust.par_h6, c.par_h6);
    assert_eq!(rust.par_h7, c.par_h7);
    assert_eq!(rust.par_gh1, c.par_gh1);
    assert_eq!(rust.par_gh2, c.par_gh2);
    assert_eq!(rust.par_gh3, c.par_gh3);
    assert_eq!(rust.par_t1, c.par_t1);
    assert_eq!(rust.par_t2, c.par_t2);
    assert_eq!(rust.par_t3, c.par_t3);
    assert_eq!(rust.par_p1, c.par_p1);
    assert_eq!(rust.par_p2, c.par_p2);
    assert_eq!(rust.par_p3, c.par_p3);
    assert_eq!(rust.par_p4, c.par_p4);
    assert_eq!(rust.par_p5, c.par_p5);
    assert_eq!(rust.par_p6, c.par_p6);
    assert_eq!(rust.par_p7, c.par_p7);
    assert_eq!(rust.par_p8, c.par_p8);
    assert_eq!(rust.par_p9, c.par_p9);
    assert_eq!(rust.par_p10, c.par_p10);
    assert_eq!(rust.res_heat_range, c.res_heat_range);
    assert_eq!(rust.res_heat_val, c.res_heat_val);
    assert_eq!(rust.range_sw_err, c.range_sw_err);
}

fn assert_float_bits(rust: f32, c: f32, context: &str) {
    assert_eq!(
        rust.to_bits(),
        c.to_bits(),
        "{context}: Rust {rust:?}, C {c:?}"
    );
}

fn main() {
    let (mut rust_calibration, mut c_calibration) = parse_bytes(&calibration_bytes(0x68e6_8842));
    assert_coefficients_equal(rust_calibration.coefficients(), &c_calibration);
    assert_eq!(
        rust_calibration.temperature_fine().to_bits(),
        c_calibration.temperature_fine.to_bits()
    );

    for temperature_adc in (0_u32..=0x0f_ffff).step_by(2_057) {
        let rust = float::compensate_temperature(temperature_adc, &mut rust_calibration);
        // SAFETY: the mutable pointer is valid and uniquely borrowed.
        let c = unsafe { oracle_float_temperature(temperature_adc, &mut c_calibration) };
        assert_float_bits(rust, c, &format!("temperature ADC {temperature_adc}"));
        assert_float_bits(
            rust_calibration.temperature_fine(),
            c_calibration.temperature_fine,
            &format!("temperature fine ADC {temperature_adc}"),
        );

        for pressure_adc in [0, 131_071, 364_576, 524_288, 786_431, 0x0f_ffff] {
            let rust = float::compensate_pressure(pressure_adc, &rust_calibration);
            // SAFETY: the immutable pointer is valid for the call.
            let c = unsafe { oracle_float_pressure(pressure_adc, &c_calibration) };
            assert_float_bits(rust, c, &format!("pressure ADC {pressure_adc}"));
        }

        for humidity_adc in [0, 1, 10_000, 30_000, 50_000, u16::MAX] {
            let rust = float::compensate_humidity(humidity_adc, &rust_calibration);
            // SAFETY: the immutable pointer is valid for the call.
            let c = unsafe { oracle_float_humidity(humidity_adc, &c_calibration) };
            assert_float_bits(rust, c, &format!("humidity ADC {humidity_adc}"));
        }
    }

    for range in 0_u8..16 {
        for adc in 0_u16..=1_023 {
            let rust_low = float::compensate_gas_low(adc, range, &rust_calibration);
            // SAFETY: range is within the C lookup-table domain and the pointer is valid.
            let c_low = unsafe { oracle_float_gas_low(adc, range, &c_calibration) };
            assert_float_bits(
                rust_low,
                c_low,
                &format!("low gas ADC {adc}, range {range}"),
            );

            let rust_high = float::compensate_gas_high(adc, range);
            // SAFETY: range is within the documented four-bit domain.
            let c_high = unsafe { oracle_float_gas_high(adc, range) };
            assert_float_bits(
                rust_high,
                c_high,
                &format!("high gas ADC {adc}, range {range}"),
            );
        }
    }

    for temperature in 0_u16..=u16::MAX {
        for ambient in [-40_i8, 0, 25, 60, 127] {
            let rust = float::calculate_heater_resistance(temperature, ambient, &rust_calibration);
            // SAFETY: the pointer is valid and this is the documented scalar domain.
            let c = unsafe { oracle_float_heater_resistance(temperature, ambient, &c_calibration) };
            assert_eq!(rust, c, "heater {temperature} °C, ambient {ambient} °C");
        }
    }

    // Exercise the aggregate API and both gas-variant branches after the
    // per-function differential loops established exact operation parity.
    for variant in [Variant::GasLow, Variant::GasHigh] {
        let raw = RawMeasurement {
            temperature_adc: 519_888,
            pressure_adc: 415_148,
            humidity_adc: 32_257,
            gas_resistance_adc: 700,
            gas_range: 5,
        };
        let measurement = float::compensate(raw, variant, &mut rust_calibration);
        assert!(measurement.temperature.is_finite());
        assert!(measurement.pressure.is_finite());
        assert!((0.0..=100.0).contains(&measurement.humidity));
        assert!(measurement.gas_resistance.is_finite());
    }

    println!("Bosch v4.4.8 floating-point differential vectors passed");
}
