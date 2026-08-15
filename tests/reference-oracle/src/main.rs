//! Differential tests against the pinned Bosch v4.4.8 fixed-point C source.

use bme68x::compensation;
use bme68x::{
    CalibrationData, Configuration, Filter, OperationMode, Oversampling, StandbyTime,
    CALIBRATION_DATA_LEN,
};

#[repr(C)]
#[derive(Clone, Copy, Debug, Default)]
struct OracleCalibration {
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
    t_fine: i32,
    res_heat_range: u8,
    res_heat_val: i8,
    range_sw_err: i8,
}

#[repr(C)]
#[derive(Clone, Copy)]
struct OracleConfiguration {
    humidity_oversampling: u8,
    temperature_oversampling: u8,
    pressure_oversampling: u8,
    filter: u8,
    standby_time: u8,
}

extern "C" {
    fn oracle_parse_calibration(bytes: *const u8, calibration: *mut OracleCalibration) -> i8;
    fn oracle_temperature(adc: u32, calibration: *mut OracleCalibration) -> i16;
    fn oracle_pressure(adc: u32, calibration: *const OracleCalibration) -> u32;
    fn oracle_humidity(adc: u16, calibration: *const OracleCalibration) -> u32;
    fn oracle_gas_low(adc: u16, range: u8, calibration: *const OracleCalibration) -> u32;
    fn oracle_gas_high(adc: u16, range: u8) -> u32;
    fn oracle_heater_resistance(
        temperature: u16,
        ambient_temperature: i8,
        calibration: *const OracleCalibration,
    ) -> u8;
    fn oracle_gas_wait(duration: u16) -> u8;
    fn oracle_shared_heater_duration(duration: u16) -> u8;
    fn oracle_measurement_duration(mode: u8, configuration: *const OracleConfiguration) -> u32;
}

fn calibration_bytes() -> [u8; CALIBRATION_DATA_LEN] {
    // A deterministic factory-valid register image. P1 is deliberately
    // non-zero so all pressure vectors stay inside Bosch's valid domain.
    let mut bytes = [0_u8; CALIBRATION_DATA_LEN];
    let mut state = 0x68e6_8842_u32;
    for byte in &mut bytes {
        state = state.wrapping_mul(1_664_525).wrapping_add(1_013_904_223);
        *byte = state.to_be_bytes()[0];
    }
    bytes[4] = 0x30;
    bytes[5] = 0x8e;
    bytes[39] &= 0x30;
    bytes
}

fn parse_bytes(bytes: &[u8; CALIBRATION_DATA_LEN]) -> (CalibrationData, OracleCalibration) {
    let rust = CalibrationData::from_register_bytes(bytes);
    let mut oracle = OracleCalibration::default();
    // SAFETY: both pointers reference live objects with the exact C layout and
    // remain valid for the duration of the call.
    let status = unsafe { oracle_parse_calibration(bytes.as_ptr(), &mut oracle) };
    assert_eq!(status, 0);
    (rust, oracle)
}

fn assert_calibration_equal(rust: &CalibrationData, c: &OracleCalibration) {
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
    assert_eq!(rust.temperature_fine(), c.t_fine);
    assert_eq!(rust.res_heat_range, c.res_heat_range);
    assert_eq!(rust.res_heat_val, c.res_heat_val);
    assert_eq!(rust.range_sw_err, c.range_sw_err);
}

fn main() {
    for byte_index in 0..CALIBRATION_DATA_LEN {
        for bit in 0..8 {
            let mut bytes = [0_u8; CALIBRATION_DATA_LEN];
            bytes[byte_index] = 1_u8 << bit;
            let (rust, c) = parse_bytes(&bytes);
            assert_calibration_equal(&rust, &c);
        }
    }

    let mut random_state = 0xb6e6_8001_u32;
    for _ in 0..10_000 {
        let mut bytes = [0_u8; CALIBRATION_DATA_LEN];
        for byte in &mut bytes {
            random_state = random_state
                .wrapping_mul(1_664_525)
                .wrapping_add(1_013_904_223);
            *byte = random_state.to_be_bytes()[0];
        }
        let (rust, c) = parse_bytes(&bytes);
        assert_calibration_equal(&rust, &c);
    }

    let (mut rust_calibration, mut c_calibration) = parse_bytes(&calibration_bytes());
    assert_calibration_equal(&rust_calibration, &c_calibration);

    for temperature_adc in (0_u32..=0x0f_ffff).step_by(2_057) {
        let rust = compensation::compensate_temperature(temperature_adc, &mut rust_calibration);
        // SAFETY: the mutable pointer is valid and uniquely borrowed.
        let c = unsafe { oracle_temperature(temperature_adc, &mut c_calibration) };
        assert_eq!(rust, c, "temperature ADC {temperature_adc}");
        assert_eq!(rust_calibration.temperature_fine(), c_calibration.t_fine);

        for pressure_adc in [0, 131_071, 364_576, 524_288, 786_431, 0x0f_ffff] {
            let rust = compensation::compensate_pressure(pressure_adc, &rust_calibration);
            // SAFETY: the immutable pointer is valid for the call.
            let c = unsafe { oracle_pressure(pressure_adc, &c_calibration) };
            assert_eq!(rust, c, "pressure ADC {pressure_adc}");
        }
        for humidity_adc in [0, 1, 10_000, 30_000, 50_000, u16::MAX] {
            let rust = compensation::compensate_humidity(humidity_adc, &rust_calibration);
            // SAFETY: the immutable pointer is valid for the call.
            let c = unsafe { oracle_humidity(humidity_adc, &c_calibration) };
            assert_eq!(rust, c, "humidity ADC {humidity_adc}");
        }
    }

    for range in 0_u8..16 {
        for adc in 0_u16..=1_023 {
            let rust_low = compensation::compensate_gas_low(adc, range, &rust_calibration);
            // SAFETY: range is within the C lookup-table domain and the pointer is valid.
            let c_low = unsafe { oracle_gas_low(adc, range, &c_calibration) };
            assert_eq!(rust_low, c_low, "low gas ADC {adc}, range {range}");

            let rust_high = compensation::compensate_gas_high(adc, range);
            // SAFETY: range is within the documented domain.
            let c_high = unsafe { oracle_gas_high(adc, range) };
            assert_eq!(rust_high, c_high, "high gas ADC {adc}, range {range}");
        }
    }

    for duration in 0_u16..=u16::MAX {
        // SAFETY: these C functions accept every u16 value.
        assert_eq!(compensation::encode_gas_wait(duration), unsafe {
            oracle_gas_wait(duration)
        });
        // SAFETY: these C functions accept every u16 value.
        assert_eq!(
            compensation::encode_shared_heater_duration(duration),
            unsafe { oracle_shared_heater_duration(duration) }
        );
    }

    for temperature in 0_u16..=u16::MAX {
        for ambient in [-40_i8, 0, 25, 60, 127] {
            let rust =
                compensation::calculate_heater_resistance(temperature, ambient, &rust_calibration);
            // SAFETY: the pointer is valid and scalar arguments cover the C domain.
            let c = unsafe { oracle_heater_resistance(temperature, ambient, &c_calibration) };
            assert_eq!(rust, c, "heater {temperature} °C, ambient {ambient} °C");
        }
    }

    let oversampling = [
        Oversampling::None,
        Oversampling::X1,
        Oversampling::X2,
        Oversampling::X4,
        Oversampling::X8,
        Oversampling::X16,
    ];
    let modes = [
        OperationMode::Sleep,
        OperationMode::Forced,
        OperationMode::Parallel,
        OperationMode::Sequential,
    ];
    for humidity in oversampling {
        for temperature in oversampling {
            for pressure in oversampling {
                let configuration = Configuration {
                    humidity_oversampling: humidity,
                    temperature_oversampling: temperature,
                    pressure_oversampling: pressure,
                    filter: Filter::Off,
                    standby_time: StandbyTime::None,
                };
                let oracle_configuration = OracleConfiguration {
                    humidity_oversampling: humidity.register_value(),
                    temperature_oversampling: temperature.register_value(),
                    pressure_oversampling: pressure.register_value(),
                    filter: 0,
                    standby_time: 8,
                };
                for mode in modes {
                    let rust = compensation::measurement_duration_us(mode, &configuration);
                    // SAFETY: both the enum value and configuration are valid C inputs.
                    let c = unsafe {
                        oracle_measurement_duration(mode.register_value(), &oracle_configuration)
                    };
                    assert_eq!(rust, c);
                }
            }
        }
    }

    println!("Bosch v4.4.8 fixed-point differential vectors passed");
}
