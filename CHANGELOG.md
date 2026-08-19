# Changelog

All notable changes to this project will be documented here. The format is
based on [Keep a Changelog](https://keepachangelog.com/en/1.1.0/), and this
project follows [Semantic Versioning](https://semver.org/).

## [Unreleased]

## [0.2.0] - 2026-08-19

### Added

- Fixed-capacity profile collector that reassembles all 1–10 heater steps and
  reports missing, duplicate, overwritten, invalid, and rollover fields.
- Exact physical status bytes, full heater/configuration readback, exact raw
  42-byte calibration images, and a stable FNV-1a calibration fingerprint.

### Changed

- **Breaking:** parallel heater profiles now accept Bosch-defined
  `repetition_multipliers: &[u8]`; the former `durations_ms: &[u16]` name was
  semantically incorrect because parallel `GAS_WAITx` values are raw TPHG
  repetition multipliers. This change requires a `0.2.0` release.
- **Breaking:** `Measurement` now exposes the exact field-status and gas-status
  register bytes, and `HeaterRegisters` now includes current-DAC and shared-wait
  registers. Public struct literals must initialize these new fields.

## [0.1.0] - 2026-08-15

### Added

- Initial blocking and asynchronous `embedded-hal` 1.0 drivers.
- Bosch v4.4.8-compatible fixed-point compensation and optional native FPU path.
- I2C and SPI, all operating modes, heater profiles, metadata, and live self-test.
- Test-only fixed-point and floating-point differential oracles pinned to Bosch SensorAPI v4.4.8.

[Unreleased]: https://github.com/carlosgj94/bosch-bme68x-rust/compare/v0.2.0...HEAD
[0.2.0]: https://github.com/carlosgj94/bosch-bme68x-rust/compare/v0.1.0...v0.2.0
[0.1.0]: https://github.com/carlosgj94/bosch-bme68x-rust/releases/tag/v0.1.0
