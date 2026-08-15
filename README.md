# bme68x

`bme68x` is a pure-Rust, `no_std`, allocation-free driver for Bosch BME680 and
BME688 environmental gas sensors. It uses `embedded-hal` 1.0 and
`embedded-hal-async` 1.0, so the same sensor code can run on STM32, ESP32, nRF,
RP2040, Linux adapters, and other supported platforms.

This is an unofficial, community-maintained project. It is not affiliated with
or endorsed by Bosch Sensortec.

The compatibility target is Bosch Sensortec's
[`BME68x_SensorAPI` v4.4.8](https://github.com/boschsensortec/BME68x_SensorAPI/tree/v4.4.8).

## Supported functionality

- I2C addresses `0x76` and `0x77`
- I2C and shared-bus-safe `SpiDevice` transports
- blocking and async APIs
- BME680 low-gas and BME688 high-gas variants
- forced, sequential, and parallel modes
- temperature, pressure, humidity, and gas-resistance compensation
- one-step and 1–10 step gas-heater profiles
- measurement status, profile index, measurement index, and heater metadata
- exact measurement/heater-duration encodings
- Bosch-style live physical self-test
- raw register access and release of owned bus/delay objects

The default `blocking` feature can be disabled. Enable `async` for the
`embedded-hal-async` frontend:

```toml
[dependencies]
bme68x = { version = "0.1", default-features = false, features = ["async"] }
```

Until the first crates.io release, use a Git or local path dependency.

Enable `float` to expose `bme68x::float`, which reproduces Bosch's
single-precision `BME68X_USE_FPU` compensation path. This feature is
independent of the blocking/async transport choice and does not add a math
library dependency.

## Blocking example

```rust,ignore
use bme68x::blocking::Bme68x;
use bme68x::interface::{I2cAddress, I2cInterface};
use bme68x::{
    Configuration, Filter, HeaterConfiguration, OperationMode, Oversampling,
    StandbyTime,
};

let interface = I2cInterface::new(i2c, I2cAddress::Low);
let mut sensor = Bme68x::new(interface, delay)?;
let configuration = Configuration {
    humidity_oversampling: Oversampling::X16,
    temperature_oversampling: Oversampling::X2,
    pressure_oversampling: Oversampling::X1,
    filter: Filter::Off,
    standby_time: StandbyTime::None,
};
let heater = HeaterConfiguration::Forced {
    enabled: true,
    temperature_celsius: 300,
    duration_ms: 100,
};

sensor.set_configuration(&configuration)?;
sensor.set_heater_configuration(&heater)?;
sensor.set_operation_mode(OperationMode::Forced)?;

let conversion_us = bme68x::compensation::measurement_duration_us(
    OperationMode::Forced,
    &configuration,
) + 100_000;
// Wait `conversion_us` with your platform timer, then read the field.
let fields = sensor.measurements(OperationMode::Forced)?;
if let Some(field) = fields.as_slice().first() {
    let temperature_c = field.values.temperature_celsius();
    let humidity_percent = field.values.humidity_percent();
    let gas_ohms = field.values.gas_resistance;
}
```

The async frontend has the same shape under `bme68x::asynch`; its bus calls and
delays are awaited.

## Output units

The primary data path preserves Bosch's fixed-point output exactly:

| Value | Type | Unit |
|---|---:|---|
| Temperature | `i16` | 0.01 °C |
| Pressure | `u32` | Pa |
| Relative humidity | `u32` | 0.001 %RH |
| Gas resistance | `u32` | Ω |

Convenience accessors convert these values to `f32` without changing the
canonical stored values. With the optional `float` feature,
`FloatCalibrationData` and `FloatMeasurement` provide Bosch's native FPU-path
units: °C, Pa, %RH, and Ω as `f32`. Every normal [`Measurement`](https://docs.rs/bme68x/latest/bme68x/struct.Measurement.html)
also exposes its decoded `raw` ADC field, so the same sample can be passed to
`bme68x::float::compensate` without another sensor read.

## Gas fingerprints and fire detection

Bosch's open SensorAPI returns compensated gas resistance and programmable
heater-scan data. It does **not** return an IAQ score, smoke label, or trained
gas fingerprint. Bosch BSEC and its trained classifiers are separate,
closed-source software.

An open wildfire classifier can be built above this crate by recording labeled
multi-temperature heater profiles and training/validating a separate model.
That model needs field data covering humidity, temperature, sensor aging,
contamination, seasons, and non-fire interferents. A BME688 alone must not be
treated as a certified life-safety detector; Bosch's datasheet excludes use in
safety-critical systems.

## Reference testing

Bosch v4.4.8 publishes no unit tests, fixtures, golden vectors, or CI suite.
This repository therefore pins the unmodified C release as a host-only
differential oracle. It is never linked into target firmware or the published
crate.

```bash
cargo test --all-features --all-targets
cargo clippy --all-features --all-targets -- -D warnings
cargo check --all-features --target thumbv7em-none-eabi
cargo run --release --manifest-path tests/reference-oracle/Cargo.toml --bin bme68x-reference-oracle
cargo run --release --manifest-path tests/reference-oracle/Cargo.toml --bin float_oracle
```

The fixed-point oracle covers calibration parsing, one-hot and seeded
calibration images, 20-bit T/P samples, 16-bit humidity samples, every gas ADC
value across all 16 ranges and both variants, every `u16` duration encoding,
every heater target, and every valid oversampling/mode timing combination.
The independent FPU oracle compares exact `f32` bit patterns for temperature,
pressure, humidity, and both gas variants, plus every heater target.

## MSRV and licensing

The minimum supported Rust version is 1.75 and the crate uses Rust edition
2021. The project is BSD-3-Clause. See `LICENSE` and `NOTICE.md` for Bosch
attribution and the pinned reference revision.
