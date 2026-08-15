// Portions of this file are derived from Bosch Sensortec's
// BME68x SensorAPI v4.4.8.
// Copyright (c) 2023 Bosch Sensortec GmbH. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause

//! Blocking `embedded-hal` BME680/BME688 driver.

use embedded_hal::delay::DelayNs;

use crate::compensation;
use crate::interface::RegisterInterface;
use crate::registers::{
    CHIP_ID, FIELD_COUNT, GAS_INDEX_MASK, GAS_RANGE_MASK, GAS_VALID_MASK, HEATER_STABLE_MASK,
    LEN_COEFF1, LEN_COEFF2, LEN_COEFF3, LEN_FIELD, MAX_PROFILE_LEN, MAX_REGISTER_WRITES, MODE_MASK,
    NEW_DATA_MASK, POLL_DELAY_US, REG_CHIP_ID, REG_COEFF1, REG_COEFF2, REG_COEFF3, REG_CONFIG,
    REG_CTRL_GAS_0, REG_CTRL_GAS_1, REG_CTRL_HUM, REG_CTRL_MEAS, REG_FIELD0, REG_GAS_WAIT0,
    REG_IDAC_HEAT0, REG_RES_HEAT0, REG_SHARED_HEATER_DURATION, REG_SOFT_RESET, REG_VARIANT_ID,
    RESET_DELAY_US, SOFT_RESET_COMMAND,
};
use crate::{
    CalibrationData, ConfigError, Configuration, Error, HeaterConfiguration, HeaterRegisters,
    Measurement, MeasurementStatus, Measurements, OperationMode, Oversampling, RawMeasurement,
    SelfTestFailure, StandbyTime, Variant,
};

const MODE_CHANGE_ATTEMPTS: usize = 100;
const FORCED_DATA_ATTEMPTS: usize = 5;

/// A blocking BME680/BME688 sensor instance.
#[derive(Debug)]
pub struct Bme68x<I, D> {
    interface: I,
    delay: D,
    chip_id: u8,
    variant: Variant,
    ambient_temperature: i8,
    calibration: CalibrationData,
}

impl<I, D> Bme68x<I, D>
where
    I: RegisterInterface,
    D: DelayNs,
{
    /// Reset and initialize a sensor using 25 °C as ambient heater temperature.
    ///
    /// # Errors
    ///
    /// Returns a bus, identity, or sensor-register error if initialization fails.
    pub fn new(interface: I, delay: D) -> Result<Self, Error<I::Error>> {
        Self::new_with_ambient_temperature(interface, delay, 25)
    }

    /// Reset and initialize a sensor with an explicit ambient temperature.
    ///
    /// # Errors
    ///
    /// Returns a bus, identity, or sensor-register error if initialization fails.
    pub fn new_with_ambient_temperature(
        interface: I,
        delay: D,
        ambient_temperature: i8,
    ) -> Result<Self, Error<I::Error>> {
        let mut sensor = Self {
            interface,
            delay,
            chip_id: 0,
            variant: Variant::GasLow,
            ambient_temperature,
            calibration: CalibrationData::default(),
        };
        sensor.initialize()?;
        Ok(sensor)
    }

    /// Re-run reset, identity, variant, and calibration discovery.
    ///
    /// # Errors
    ///
    /// Returns a bus, identity, or sensor-register error if discovery fails.
    pub fn initialize(&mut self) -> Result<(), Error<I::Error>> {
        self.soft_reset()?;

        let mut chip_id = 0;
        self.read_registers(REG_CHIP_ID, core::slice::from_mut(&mut chip_id))?;
        if chip_id != CHIP_ID {
            return Err(Error::UnexpectedChipId { found: chip_id });
        }
        self.chip_id = chip_id;

        let mut variant = 0;
        self.read_registers(REG_VARIANT_ID, core::slice::from_mut(&mut variant))?;
        self.variant = Variant::from_register(variant).ok_or(Error::InvalidRegisterValue {
            register: REG_VARIANT_ID,
            value: variant,
        })?;

        let mut block_1 = [0_u8; LEN_COEFF1];
        let mut block_2 = [0_u8; LEN_COEFF2];
        let mut block_3 = [0_u8; LEN_COEFF3];
        self.read_registers(REG_COEFF1, &mut block_1)?;
        self.read_registers(REG_COEFF2, &mut block_2)?;
        self.read_registers(REG_COEFF3, &mut block_3)?;
        self.calibration = CalibrationData::from_register_blocks(&block_1, &block_2, &block_3);
        Ok(())
    }

    /// Issue the documented `0xb6` soft-reset command and wait 10 ms.
    ///
    /// # Errors
    ///
    /// Returns the concrete bus error if the reset write fails.
    pub fn soft_reset(&mut self) -> Result<(), Error<I::Error>> {
        self.interface
            .write_pairs(&[REG_SOFT_RESET], &[SOFT_RESET_COMMAND])
            .map_err(Error::Bus)?;
        self.delay.delay_us(RESET_DELAY_US);
        Ok(())
    }

    /// Release the owned register interface and delay implementation.
    pub fn release(self) -> (I, D) {
        (self.interface, self.delay)
    }

    /// Return the chip identifier read during initialization.
    #[must_use]
    pub const fn chip_id(&self) -> u8 {
        self.chip_id
    }

    /// Return the detected gas-calculation variant.
    #[must_use]
    pub const fn variant(&self) -> Variant {
        self.variant
    }

    /// Return the factory calibration coefficients.
    #[must_use]
    pub const fn calibration(&self) -> &CalibrationData {
        &self.calibration
    }

    /// Return the ambient temperature used for heater calculations.
    #[must_use]
    pub const fn ambient_temperature(&self) -> i8 {
        self.ambient_temperature
    }

    /// Change the ambient temperature used for future heater calculations.
    pub fn set_ambient_temperature(&mut self, temperature_celsius: i8) {
        self.ambient_temperature = temperature_celsius;
    }

    /// Read consecutive logical sensor registers.
    ///
    /// # Errors
    ///
    /// Returns the concrete bus error if the read fails.
    pub fn read_registers(&mut self, register: u8, data: &mut [u8]) -> Result<(), Error<I::Error>> {
        self.interface.read(register, data).map_err(Error::Bus)
    }

    /// Write between one and ten logical register/value pairs.
    ///
    /// # Errors
    ///
    /// Returns a configuration error for invalid slices or the concrete bus
    /// error if the write fails.
    pub fn write_registers(
        &mut self,
        registers: &[u8],
        values: &[u8],
    ) -> Result<(), Error<I::Error>> {
        if registers.len() != values.len() {
            return Err(Error::InvalidConfiguration(
                ConfigError::RegisterWriteLengthMismatch {
                    registers: registers.len(),
                    values: values.len(),
                },
            ));
        }
        if registers.is_empty() || registers.len() > MAX_REGISTER_WRITES {
            return Err(Error::InvalidConfiguration(
                ConfigError::InvalidRegisterWriteLength {
                    length: registers.len(),
                },
            ));
        }
        self.interface
            .write_pairs(registers, values)
            .map_err(Error::Bus)
    }

    /// Read the current operation mode.
    ///
    /// # Errors
    ///
    /// Returns the concrete bus error if the mode register cannot be read.
    pub fn operation_mode(&mut self) -> Result<OperationMode, Error<I::Error>> {
        let mut value = 0;
        self.read_registers(REG_CTRL_MEAS, core::slice::from_mut(&mut value))?;
        // The two-bit mask means every possible value maps to a mode.
        Ok(OperationMode::from_register(value & MODE_MASK).unwrap_or(OperationMode::Sleep))
    }

    /// Put the sensor to sleep, then enter the requested mode.
    ///
    /// # Errors
    ///
    /// Returns a bus error or [`Error::Timeout`] if sleep is not reached.
    pub fn set_operation_mode(&mut self, mode: OperationMode) -> Result<(), Error<I::Error>> {
        let mut register_value = 0;
        let mut reached_sleep = false;
        for _ in 0..MODE_CHANGE_ATTEMPTS {
            self.read_registers(REG_CTRL_MEAS, core::slice::from_mut(&mut register_value))?;
            if register_value & MODE_MASK == OperationMode::Sleep.register_value() {
                reached_sleep = true;
                break;
            }
            register_value &= !MODE_MASK;
            self.write_registers(&[REG_CTRL_MEAS], &[register_value])?;
            self.delay.delay_us(POLL_DELAY_US);
        }
        if !reached_sleep {
            return Err(Error::Timeout);
        }

        if mode != OperationMode::Sleep {
            register_value = (register_value & !MODE_MASK) | mode.register_value();
            self.write_registers(&[REG_CTRL_MEAS], &[register_value])?;
        }
        Ok(())
    }

    /// Read the current oversampling, filter, and standby settings.
    ///
    /// # Errors
    ///
    /// Returns a bus error or [`Error::InvalidRegisterValue`] for reserved
    /// register encodings.
    pub fn configuration(&mut self) -> Result<Configuration, Error<I::Error>> {
        let mut registers = [0_u8; 5];
        self.read_registers(REG_CTRL_GAS_1, &mut registers)?;

        let humidity_raw = registers[1] & 0x07;
        let temperature_raw = (registers[3] & 0xe0) >> 5;
        let pressure_raw = (registers[3] & 0x1c) >> 2;
        let filter_raw = (registers[4] & 0x1c) >> 2;
        let standby_raw = if registers[0] & 0x80 != 0 {
            StandbyTime::None.register_value()
        } else {
            (registers[4] & 0xe0) >> 5
        };

        Ok(Configuration {
            humidity_oversampling: Self::parse_oversampling(REG_CTRL_HUM, humidity_raw)?,
            temperature_oversampling: Self::parse_oversampling(REG_CTRL_MEAS, temperature_raw)?,
            pressure_oversampling: Self::parse_oversampling(REG_CTRL_MEAS, pressure_raw)?,
            filter: crate::Filter::from_register(filter_raw).ok_or(
                Error::InvalidRegisterValue {
                    register: REG_CONFIG,
                    value: filter_raw,
                },
            )?,
            standby_time: StandbyTime::from_register(standby_raw).ok_or(
                Error::InvalidRegisterValue {
                    register: REG_CONFIG,
                    value: standby_raw,
                },
            )?,
        })
    }

    /// Apply oversampling, filter, and standby settings while preserving mode.
    ///
    /// # Errors
    ///
    /// Returns a bus error or timeout encountered while entering sleep.
    pub fn set_configuration(
        &mut self,
        configuration: &Configuration,
    ) -> Result<(), Error<I::Error>> {
        let previous_mode = self.operation_mode()?;
        self.set_operation_mode(OperationMode::Sleep)?;

        let addresses = [0x71, 0x72, 0x73, 0x74, 0x75];
        let mut values = [0_u8; 5];
        self.read_registers(REG_CTRL_GAS_1, &mut values)?;

        values[4] = (values[4] & !0x1c) | (configuration.filter.register_value() << 2);
        values[3] =
            (values[3] & !0xe0) | (configuration.temperature_oversampling.register_value() << 5);
        values[3] =
            (values[3] & !0x1c) | (configuration.pressure_oversampling.register_value() << 2);
        values[1] = (values[1] & !0x07) | configuration.humidity_oversampling.register_value();

        if configuration.standby_time == StandbyTime::None {
            values[0] |= 0x80;
            values[4] &= !0xe0;
        } else {
            values[0] &= !0x80;
            values[4] = (values[4] & !0xe0) | (configuration.standby_time.register_value() << 5);
        }
        self.write_registers(&addresses, &values)?;

        if previous_mode != OperationMode::Sleep {
            self.set_operation_mode(previous_mode)?;
        }
        Ok(())
    }

    /// Calculate T/P/H plus switching/wake time in microseconds.
    #[must_use]
    pub const fn measurement_duration(mode: OperationMode, configuration: &Configuration) -> u32 {
        compensation::measurement_duration_us(mode, configuration)
    }

    /// Configure the gas heater for forced, sequential, or parallel mode.
    ///
    /// # Errors
    ///
    /// Returns a profile-validation, bus, or mode-transition error.
    pub fn set_heater_configuration(
        &mut self,
        configuration: &HeaterConfiguration<'_>,
    ) -> Result<(), Error<I::Error>> {
        self.set_operation_mode(OperationMode::Sleep)?;

        let profile_len = match configuration {
            HeaterConfiguration::Forced {
                temperature_celsius,
                duration_ms,
                ..
            } => {
                let resistance = compensation::calculate_heater_resistance(
                    *temperature_celsius,
                    self.ambient_temperature,
                    &self.calibration,
                );
                let gas_wait = compensation::encode_gas_wait(*duration_ms);
                self.write_registers(&[REG_RES_HEAT0], &[resistance])?;
                self.write_registers(&[REG_GAS_WAIT0], &[gas_wait])?;
                0
            }
            HeaterConfiguration::Sequential {
                temperatures_celsius,
                durations_ms,
                ..
            } => {
                let len = Self::validate_profile(temperatures_celsius, durations_ms)?;
                let mut resistance = [0_u8; MAX_PROFILE_LEN];
                let mut gas_wait = [0_u8; MAX_PROFILE_LEN];
                let mut resistance_registers = [0_u8; MAX_PROFILE_LEN];
                let mut wait_registers = [0_u8; MAX_PROFILE_LEN];
                for index in 0..len {
                    resistance[index] = compensation::calculate_heater_resistance(
                        temperatures_celsius[index],
                        self.ambient_temperature,
                        &self.calibration,
                    );
                    gas_wait[index] = compensation::encode_gas_wait(durations_ms[index]);
                    let register_offset = u8::try_from(index).unwrap_or(0);
                    resistance_registers[index] = REG_RES_HEAT0 + register_offset;
                    wait_registers[index] = REG_GAS_WAIT0 + register_offset;
                }
                self.write_registers(&resistance_registers[..len], &resistance[..len])?;
                self.write_registers(&wait_registers[..len], &gas_wait[..len])?;
                u8::try_from(len).unwrap_or(0)
            }
            HeaterConfiguration::Parallel {
                temperatures_celsius,
                durations_ms,
                shared_duration_ms,
                ..
            } => {
                let len = Self::validate_profile(temperatures_celsius, durations_ms)?;
                if *shared_duration_ms == 0 {
                    return Err(Error::InvalidConfiguration(
                        ConfigError::MissingSharedHeaterDuration,
                    ));
                }
                let shared = compensation::encode_shared_heater_duration(*shared_duration_ms);
                self.write_registers(&[REG_SHARED_HEATER_DURATION], &[shared])?;

                let mut resistance = [0_u8; MAX_PROFILE_LEN];
                let mut gas_wait = [0_u8; MAX_PROFILE_LEN];
                let mut resistance_registers = [0_u8; MAX_PROFILE_LEN];
                let mut wait_registers = [0_u8; MAX_PROFILE_LEN];
                for index in 0..len {
                    resistance[index] = compensation::calculate_heater_resistance(
                        temperatures_celsius[index],
                        self.ambient_temperature,
                        &self.calibration,
                    );
                    gas_wait[index] = durations_ms[index].to_le_bytes()[0];
                    let register_offset = u8::try_from(index).unwrap_or(0);
                    resistance_registers[index] = REG_RES_HEAT0 + register_offset;
                    wait_registers[index] = REG_GAS_WAIT0 + register_offset;
                }
                self.write_registers(&resistance_registers[..len], &resistance[..len])?;
                self.write_registers(&wait_registers[..len], &gas_wait[..len])?;
                u8::try_from(len).unwrap_or(0)
            }
        };

        let mut control = [0_u8; 2];
        self.read_registers(REG_CTRL_GAS_0, &mut control)?;
        let (heater_control, run_gas) = if configuration.enabled() {
            let run_gas = match self.variant {
                Variant::GasLow => 1,
                Variant::GasHigh => 2,
            };
            (0, run_gas)
        } else {
            (1, 0)
        };
        control[0] = (control[0] & !0x08) | (heater_control << 3);
        control[1] = (control[1] & !0x0f) | (profile_len & 0x0f);
        control[1] = (control[1] & !0x30) | (run_gas << 4);
        self.write_registers(&[REG_CTRL_GAS_0, REG_CTRL_GAS_1], &control)
    }

    /// Read all ten raw heater-resistance and gas-wait profile registers.
    ///
    /// # Errors
    ///
    /// Returns the concrete bus error if either register read fails.
    pub fn heater_registers(&mut self) -> Result<HeaterRegisters, Error<I::Error>> {
        let mut registers = HeaterRegisters::default();
        self.read_registers(REG_RES_HEAT0, &mut registers.resistance)?;
        self.read_registers(REG_GAS_WAIT0, &mut registers.gas_wait)?;
        Ok(registers)
    }

    /// Read and compensate newly available data fields.
    ///
    /// Forced mode returns zero or one field. Sequential and parallel modes
    /// return up to three fields ordered from oldest to newest. No-new-data is
    /// represented by an empty successful result, matching Bosch's warning
    /// status without turning it into a Rust error.
    ///
    /// # Errors
    ///
    /// Returns a configuration, register-value, timeout, or concrete bus error.
    pub fn measurements(&mut self, mode: OperationMode) -> Result<Measurements, Error<I::Error>> {
        match mode {
            OperationMode::Forced => self.forced_measurement(),
            OperationMode::Parallel | OperationMode::Sequential => self.profile_measurements(),
            OperationMode::Sleep => Err(Error::InvalidConfiguration(
                ConfigError::UnsupportedDataMode,
            )),
        }
    }

    /// Run Bosch's live heater and environmental self-test sequence.
    ///
    /// This destructive sensor-state test resets the device, performs one
    /// 350 °C / 1 s gas measurement and six alternating 350/150 °C / 2 s
    /// measurements, and leaves the sensor configured for forced mode. It
    /// takes roughly 13 seconds and is not intended as a normal power-on check.
    ///
    /// # Errors
    ///
    /// Returns a communication/configuration error, timeout, or the precise
    /// [`SelfTestFailure`] detected by the Bosch sequence.
    pub fn self_test(&mut self) -> Result<(), Error<I::Error>> {
        self.initialize()?;
        self.ambient_temperature = 25;

        let configuration = Configuration {
            humidity_oversampling: Oversampling::X1,
            temperature_oversampling: Oversampling::X2,
            pressure_oversampling: Oversampling::X16,
            filter: crate::Filter::Off,
            standby_time: StandbyTime::None,
        };

        let initial_heater = HeaterConfiguration::Forced {
            enabled: true,
            temperature_celsius: 350,
            duration_ms: 1_000,
        };
        self.set_heater_configuration(&initial_heater)?;
        self.set_configuration(&configuration)?;
        self.set_operation_mode(OperationMode::Forced)?;
        self.delay.delay_us(1_000_000);
        let initial = self
            .measurements(OperationMode::Forced)?
            .as_slice()
            .first()
            .copied()
            .ok_or(Error::Timeout)?;
        if initial.heater_current == 0 || initial.heater_current == 0xff {
            return Err(Error::SelfTestFailed(SelfTestFailure::InvalidHeaterCurrent));
        }
        if !initial.status.gas_valid() {
            return Err(Error::SelfTestFailed(
                SelfTestFailure::InvalidGasMeasurement,
            ));
        }

        let mut data = [Measurement::default(); 6];
        for (index, measurement) in data.iter_mut().enumerate() {
            let heater = HeaterConfiguration::Forced {
                enabled: true,
                temperature_celsius: if index % 2 == 0 { 350 } else { 150 },
                duration_ms: 2_000,
            };
            self.set_heater_configuration(&heater)?;
            self.set_configuration(&configuration)?;
            self.set_operation_mode(OperationMode::Forced)?;
            self.delay.delay_us(2_000_000);
            *measurement = self
                .measurements(OperationMode::Forced)?
                .as_slice()
                .first()
                .copied()
                .ok_or(Error::Timeout)?;
        }

        let first = data[0].values;
        if !(0..=6_000).contains(&first.temperature)
            || !(90_000..=110_000).contains(&first.pressure)
            || !(20_000..=80_000).contains(&first.humidity)
        {
            return Err(Error::SelfTestFailed(
                SelfTestFailure::MeasurementOutOfRange,
            ));
        }
        if data.iter().any(|value| !value.status.gas_valid()) {
            return Err(Error::SelfTestFailed(
                SelfTestFailure::InvalidGasMeasurement,
            ));
        }

        let numerator = 5_u64
            * (u64::from(data[3].values.gas_resistance) + u64::from(data[5].values.gas_resistance));
        let denominator = 2_u64 * u64::from(data[4].values.gas_resistance);
        let response_percent = numerator.checked_div(denominator).unwrap_or(0);
        if response_percent < 6 {
            return Err(Error::SelfTestFailed(SelfTestFailure::GasResponseTooSmall));
        }

        Ok(())
    }

    fn parse_oversampling(register: u8, value: u8) -> Result<Oversampling, Error<I::Error>> {
        Oversampling::from_register(value).ok_or(Error::InvalidRegisterValue { register, value })
    }

    fn validate_profile(temperatures: &[u16], durations: &[u16]) -> Result<usize, Error<I::Error>> {
        if temperatures.len() != durations.len() {
            return Err(Error::InvalidConfiguration(
                ConfigError::ProfileLengthMismatch {
                    temperatures: temperatures.len(),
                    durations: durations.len(),
                },
            ));
        }
        if temperatures.is_empty() || temperatures.len() > MAX_PROFILE_LEN {
            return Err(Error::InvalidConfiguration(
                ConfigError::InvalidProfileLength {
                    length: temperatures.len(),
                },
            ));
        }
        Ok(temperatures.len())
    }

    fn forced_measurement(&mut self) -> Result<Measurements, Error<I::Error>> {
        for _ in 0..FORCED_DATA_ATTEMPTS {
            let mut field = [0_u8; LEN_FIELD];
            self.read_registers(REG_FIELD0, &mut field)?;
            let status = self.field_status(&field);
            if status.is_new() {
                let gas_index = field[0] & GAS_INDEX_MASK;
                if usize::from(gas_index) >= MAX_PROFILE_LEN {
                    return Err(Error::InvalidRegisterValue {
                        register: REG_FIELD0,
                        value: gas_index,
                    });
                }
                let mut heater_resistance = 0;
                let mut heater_current = 0;
                let mut gas_wait = 0;
                self.read_registers(
                    REG_RES_HEAT0 + gas_index,
                    core::slice::from_mut(&mut heater_resistance),
                )?;
                self.read_registers(
                    REG_IDAC_HEAT0 + gas_index,
                    core::slice::from_mut(&mut heater_current),
                )?;
                self.read_registers(
                    REG_GAS_WAIT0 + gas_index,
                    core::slice::from_mut(&mut gas_wait),
                )?;
                let measurement =
                    self.decode_field(&field, heater_current, heater_resistance, gas_wait);
                return Ok(Measurements::new(
                    [measurement, Measurement::default(), Measurement::default()],
                    1,
                ));
            }
            self.delay.delay_us(POLL_DELAY_US);
        }
        Ok(Measurements::default())
    }

    fn profile_measurements(&mut self) -> Result<Measurements, Error<I::Error>> {
        let mut fields = [0_u8; LEN_FIELD * FIELD_COUNT];
        let mut settings = [0_u8; MAX_PROFILE_LEN * 3];
        self.read_registers(REG_FIELD0, &mut fields)?;
        self.read_registers(REG_IDAC_HEAT0, &mut settings)?;

        let mut measurements = [Measurement::default(); FIELD_COUNT];
        let mut new_fields = 0_u8;
        for (index, measurement) in measurements.iter_mut().enumerate() {
            let offset = index * LEN_FIELD;
            let field: &[u8; LEN_FIELD] = (&fields[offset..offset + LEN_FIELD])
                .try_into()
                .expect("fixed field length");
            let gas_index_raw = field[0] & GAS_INDEX_MASK;
            let gas_index = usize::from(gas_index_raw);
            if gas_index >= MAX_PROFILE_LEN {
                let register_offset = u8::try_from(offset).unwrap_or(0);
                return Err(Error::InvalidRegisterValue {
                    register: REG_FIELD0 + register_offset,
                    value: gas_index_raw,
                });
            }
            *measurement = self.decode_field(
                field,
                settings[gas_index],
                settings[MAX_PROFILE_LEN + gas_index],
                settings[MAX_PROFILE_LEN * 2 + gas_index],
            );
            if measurement.status.is_new() {
                new_fields += 1;
            }
        }

        for low in 0..2 {
            for high in low + 1..FIELD_COUNT {
                if should_swap_fields(&measurements[low], &measurements[high]) {
                    measurements.swap(low, high);
                }
            }
        }
        Ok(Measurements::new(measurements, new_fields))
    }

    fn field_status(&self, field: &[u8; LEN_FIELD]) -> MeasurementStatus {
        let gas_status_index = match self.variant {
            Variant::GasLow => 14,
            Variant::GasHigh => 16,
        };
        MeasurementStatus::from_bits(
            (field[0] & NEW_DATA_MASK)
                | (field[gas_status_index] & GAS_VALID_MASK)
                | (field[gas_status_index] & HEATER_STABLE_MASK),
        )
    }

    fn decode_field(
        &mut self,
        field: &[u8; LEN_FIELD],
        heater_current: u8,
        heater_resistance: u8,
        gas_wait: u8,
    ) -> Measurement {
        let pressure_adc =
            (u32::from(field[2]) << 12) | (u32::from(field[3]) << 4) | u32::from(field[4] >> 4);
        let temperature_adc =
            (u32::from(field[5]) << 12) | (u32::from(field[6]) << 4) | u32::from(field[7] >> 4);
        let humidity_adc = u16::from_be_bytes([field[8], field[9]]);
        let (gas_resistance_adc, gas_range) = match self.variant {
            Variant::GasLow => (
                (u16::from(field[13]) << 2) | u16::from(field[14] >> 6),
                field[14] & GAS_RANGE_MASK,
            ),
            Variant::GasHigh => (
                (u16::from(field[15]) << 2) | u16::from(field[16] >> 6),
                field[16] & GAS_RANGE_MASK,
            ),
        };
        let raw = RawMeasurement {
            temperature_adc,
            pressure_adc,
            humidity_adc,
            gas_resistance_adc,
            gas_range,
        };
        let values = compensation::compensate(raw, self.variant, &mut self.calibration);

        Measurement {
            status: self.field_status(field),
            gas_index: field[0] & GAS_INDEX_MASK,
            measurement_index: field[1],
            heater_resistance,
            heater_current,
            gas_wait,
            raw,
            values,
        }
    }
}

fn should_swap_fields(low: &Measurement, high: &Measurement) -> bool {
    if low.status.is_new() && high.status.is_new() {
        let difference = i16::from(high.measurement_index) - i16::from(low.measurement_index);
        ((difference > -3) && (difference < 0)) || difference > 2
    } else {
        !low.status.is_new() && high.status.is_new()
    }
}

#[cfg(test)]
mod tests {
    // The fixture encoder intentionally narrows already-masked sensor fields.
    #![allow(clippy::cast_possible_truncation)]

    extern crate std;

    use super::*;
    use std::vec::Vec;

    #[derive(Clone, Copy, Debug, Eq, PartialEq)]
    enum TestBusError {
        Read { register: u8, occurrence: usize },
        Write { register: u8, occurrence: usize },
    }

    #[derive(Clone, Copy, Debug)]
    struct FailurePoint {
        register: u8,
        occurrence: usize,
    }

    #[derive(Debug)]
    struct TestInterface {
        bytes: [u8; 256],
        field_reads: Vec<[u8; LEN_FIELD]>,
        next_field: usize,
        read_counts: [usize; 256],
        write_counts: [usize; 256],
        reads: Vec<(u8, usize)>,
        writes: Vec<(u8, Vec<u8>)>,
        fail_read: Option<FailurePoint>,
        fail_write: Option<FailurePoint>,
        hold_mode_awake: bool,
    }

    impl TestInterface {
        fn sensor(variant: Variant) -> Self {
            let mut bytes = [0_u8; 256];
            bytes[usize::from(REG_CHIP_ID)] = CHIP_ID;
            bytes[usize::from(REG_VARIANT_ID)] = variant.register_value();
            load_calibration(&mut bytes, &reference_calibration_bytes());
            Self {
                bytes,
                field_reads: Vec::new(),
                next_field: 0,
                read_counts: [0; 256],
                write_counts: [0; 256],
                reads: Vec::new(),
                writes: Vec::new(),
                fail_read: None,
                fail_write: None,
                hold_mode_awake: false,
            }
        }

        fn fail_next_read(&mut self, register: u8) {
            self.fail_read = Some(FailurePoint {
                register,
                occurrence: self.read_counts[usize::from(register)] + 1,
            });
        }

        fn fail_next_write(&mut self, register: u8) {
            self.fail_write = Some(FailurePoint {
                register,
                occurrence: self.write_counts[usize::from(register)] + 1,
            });
        }

        fn set_profile_fields(&mut self, fields: &[[u8; LEN_FIELD]; FIELD_COUNT]) {
            let start = usize::from(REG_FIELD0);
            for (index, field) in fields.iter().enumerate() {
                let offset = start + index * LEN_FIELD;
                self.bytes[offset..offset + LEN_FIELD].copy_from_slice(field);
            }
        }

        fn read_count(&self, register: u8) -> usize {
            self.reads
                .iter()
                .filter(|(address, _)| *address == register)
                .count()
        }

        fn write_count(&self, register: u8) -> usize {
            self.writes
                .iter()
                .filter(|(address, _)| *address == register)
                .count()
        }
    }

    impl RegisterInterface for TestInterface {
        type Error = TestBusError;

        fn read(&mut self, register: u8, data: &mut [u8]) -> Result<(), Self::Error> {
            let count = &mut self.read_counts[usize::from(register)];
            *count += 1;
            let occurrence = *count;
            self.reads.push((register, data.len()));
            if self.fail_read.is_some_and(|failure| {
                failure.register == register && failure.occurrence == occurrence
            }) {
                return Err(TestBusError::Read {
                    register,
                    occurrence,
                });
            }

            if register == REG_FIELD0
                && data.len() == LEN_FIELD
                && self.next_field < self.field_reads.len()
            {
                data.copy_from_slice(&self.field_reads[self.next_field]);
                self.next_field += 1;
                return Ok(());
            }

            let start = usize::from(register);
            data.copy_from_slice(&self.bytes[start..start + data.len()]);
            Ok(())
        }

        fn write(&mut self, register: u8, data: &[u8]) -> Result<(), Self::Error> {
            let count = &mut self.write_counts[usize::from(register)];
            *count += 1;
            let occurrence = *count;
            self.writes.push((register, data.to_vec()));
            if self.fail_write.is_some_and(|failure| {
                failure.register == register && failure.occurrence == occurrence
            }) {
                return Err(TestBusError::Write {
                    register,
                    occurrence,
                });
            }

            if self.hold_mode_awake && register == REG_CTRL_MEAS {
                return Ok(());
            }

            let start = usize::from(register);
            self.bytes[start..start + data.len()].copy_from_slice(data);
            Ok(())
        }
    }

    #[derive(Debug, Default)]
    struct RecordingDelay {
        nanoseconds: Vec<u32>,
    }

    impl DelayNs for RecordingDelay {
        fn delay_ns(&mut self, nanoseconds: u32) {
            self.nanoseconds.push(nanoseconds);
        }
    }

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

    fn reference_calibration_bytes() -> [u8; crate::types::CALIBRATION_DATA_LEN] {
        let calibration = reference_calibration();
        let mut bytes = [0_u8; crate::types::CALIBRATION_DATA_LEN];
        bytes[0..2].copy_from_slice(&calibration.par_t2.to_le_bytes());
        bytes[2] = calibration.par_t3.to_ne_bytes()[0];
        bytes[4..6].copy_from_slice(&calibration.par_p1.to_le_bytes());
        bytes[6..8].copy_from_slice(&calibration.par_p2.to_le_bytes());
        bytes[8] = calibration.par_p3.to_ne_bytes()[0];
        bytes[10..12].copy_from_slice(&calibration.par_p4.to_le_bytes());
        bytes[12..14].copy_from_slice(&calibration.par_p5.to_le_bytes());
        bytes[14] = calibration.par_p7.to_ne_bytes()[0];
        bytes[15] = calibration.par_p6.to_ne_bytes()[0];
        bytes[18..20].copy_from_slice(&calibration.par_p8.to_le_bytes());
        bytes[20..22].copy_from_slice(&calibration.par_p9.to_le_bytes());
        bytes[22] = calibration.par_p10;
        bytes[23] = (calibration.par_h2 >> 4) as u8;
        bytes[24] = ((calibration.par_h2 & 0x0f) as u8) << 4 | (calibration.par_h1 & 0x0f) as u8;
        bytes[25] = (calibration.par_h1 >> 4) as u8;
        bytes[26] = calibration.par_h3.to_ne_bytes()[0];
        bytes[27] = calibration.par_h4.to_ne_bytes()[0];
        bytes[28] = calibration.par_h5.to_ne_bytes()[0];
        bytes[29] = calibration.par_h6;
        bytes[30] = calibration.par_h7.to_ne_bytes()[0];
        bytes[31..33].copy_from_slice(&calibration.par_t1.to_le_bytes());
        bytes[33..35].copy_from_slice(&calibration.par_gh2.to_le_bytes());
        bytes[35] = calibration.par_gh1.to_ne_bytes()[0];
        bytes[36] = calibration.par_gh3.to_ne_bytes()[0];
        bytes[37] = calibration.res_heat_val.to_ne_bytes()[0];
        bytes[39] = calibration.res_heat_range << 4;
        bytes[41] = (calibration.range_sw_err * 16).to_ne_bytes()[0];
        bytes
    }

    fn load_calibration(memory: &mut [u8; 256], bytes: &[u8; crate::types::CALIBRATION_DATA_LEN]) {
        memory[usize::from(REG_COEFF1)..usize::from(REG_COEFF1) + LEN_COEFF1]
            .copy_from_slice(&bytes[..LEN_COEFF1]);
        memory[usize::from(REG_COEFF2)..usize::from(REG_COEFF2) + LEN_COEFF2]
            .copy_from_slice(&bytes[LEN_COEFF1..LEN_COEFF1 + LEN_COEFF2]);
        memory[usize::from(REG_COEFF3)..usize::from(REG_COEFF3) + LEN_COEFF3]
            .copy_from_slice(&bytes[LEN_COEFF1 + LEN_COEFF2..]);
    }

    fn initialized_sensor(variant: Variant) -> Bme68x<TestInterface, RecordingDelay> {
        Bme68x::new(TestInterface::sensor(variant), RecordingDelay::default()).unwrap()
    }

    fn raw_sample() -> RawMeasurement {
        RawMeasurement {
            temperature_adc: 519_888,
            pressure_adc: 364_576,
            humidity_adc: 30_000,
            gas_resistance_adc: 700,
            gas_range: 8,
        }
    }

    #[allow(clippy::too_many_arguments)]
    fn field(
        gas_index: u8,
        measurement_index: u8,
        is_new: bool,
        gas_valid: bool,
        heater_stable: bool,
        environmental: RawMeasurement,
        low_gas: (u16, u8),
        high_gas: (u16, u8),
    ) -> [u8; LEN_FIELD] {
        let mut data = [0_u8; LEN_FIELD];
        data[0] = gas_index | if is_new { NEW_DATA_MASK } else { 0 };
        data[1] = measurement_index;
        data[2] = (environmental.pressure_adc >> 12) as u8;
        data[3] = (environmental.pressure_adc >> 4) as u8;
        data[4] = (environmental.pressure_adc << 4) as u8;
        data[5] = (environmental.temperature_adc >> 12) as u8;
        data[6] = (environmental.temperature_adc >> 4) as u8;
        data[7] = (environmental.temperature_adc << 4) as u8;
        data[8..10].copy_from_slice(&environmental.humidity_adc.to_be_bytes());
        data[13] = (low_gas.0 >> 2) as u8;
        data[14] = ((low_gas.0 & 0x03) << 6) as u8 | (low_gas.1 & GAS_RANGE_MASK);
        data[15] = (high_gas.0 >> 2) as u8;
        data[16] = ((high_gas.0 & 0x03) << 6) as u8 | (high_gas.1 & GAS_RANGE_MASK);
        let status = if gas_valid { GAS_VALID_MASK } else { 0 }
            | if heater_stable { HEATER_STABLE_MASK } else { 0 };
        data[14] |= status;
        data[16] |= status;
        data
    }

    fn measurement(index: u8, is_new: bool) -> Measurement {
        Measurement {
            status: MeasurementStatus::from_bits(if is_new { NEW_DATA_MASK } else { 0 }),
            measurement_index: index,
            ..Measurement::default()
        }
    }

    #[test]
    fn field_sort_matches_bosch_wraparound_rules() {
        assert!(!should_swap_fields(
            &measurement(255, true),
            &measurement(0, true)
        ));
        assert!(should_swap_fields(
            &measurement(0, true),
            &measurement(255, true)
        ));
        assert!(should_swap_fields(
            &measurement(6, true),
            &measurement(4, true)
        ));
        assert!(!should_swap_fields(
            &measurement(3, true),
            &measurement(5, true)
        ));
        assert!(should_swap_fields(
            &measurement(3, false),
            &measurement(4, true)
        ));
    }

    #[test]
    fn initialization_reads_identity_variant_and_calibration() {
        let sensor = initialized_sensor(Variant::GasHigh);
        assert_eq!(sensor.chip_id(), CHIP_ID);
        assert_eq!(sensor.variant(), Variant::GasHigh);
        assert_eq!(sensor.calibration(), &reference_calibration());
        assert_eq!(sensor.delay.nanoseconds, std::vec![RESET_DELAY_US * 1_000]);
    }

    #[test]
    fn initialization_rejects_bad_identity_and_variant() {
        let mut memory = TestInterface::sensor(Variant::GasHigh);
        memory.bytes[usize::from(REG_CHIP_ID)] = 0x60;
        assert_eq!(
            Bme68x::new(memory, RecordingDelay::default()).unwrap_err(),
            Error::UnexpectedChipId { found: 0x60 }
        );

        let mut memory = TestInterface::sensor(Variant::GasHigh);
        memory.bytes[usize::from(REG_VARIANT_ID)] = 2;
        assert_eq!(
            Bme68x::new(memory, RecordingDelay::default()).unwrap_err(),
            Error::InvalidRegisterValue {
                register: REG_VARIANT_ID,
                value: 2,
            }
        );
    }

    #[test]
    fn typed_configuration_round_trips_through_registers() {
        let mut sensor = initialized_sensor(Variant::GasHigh);
        let expected = Configuration {
            humidity_oversampling: Oversampling::X1,
            temperature_oversampling: Oversampling::X2,
            pressure_oversampling: Oversampling::X16,
            filter: crate::Filter::Size7,
            standby_time: StandbyTime::Millis20,
        };
        sensor.set_configuration(&expected).unwrap();
        assert_eq!(sensor.configuration().unwrap(), expected);
    }

    #[test]
    fn heater_profiles_are_length_checked_before_register_writes() {
        let mut sensor = initialized_sensor(Variant::GasHigh);
        let temperatures = [200_u16, 300];
        let durations = [100_u16];
        let heater = HeaterConfiguration::Sequential {
            enabled: true,
            temperatures_celsius: &temperatures,
            durations_ms: &durations,
        };
        assert_eq!(
            sensor.set_heater_configuration(&heater),
            Err(Error::InvalidConfiguration(
                ConfigError::ProfileLengthMismatch {
                    temperatures: 2,
                    durations: 1,
                }
            ))
        );
    }

    #[test]
    fn forced_decoding_selects_low_and_high_gas_fields_and_preserves_raw_data() {
        let environmental = raw_sample();
        let low_gas = (700, 8);
        let high_gas = (333, 4);

        for (variant, expected_gas) in [
            (
                Variant::GasLow,
                RawMeasurement {
                    gas_resistance_adc: low_gas.0,
                    gas_range: low_gas.1,
                    ..environmental
                },
            ),
            (
                Variant::GasHigh,
                RawMeasurement {
                    gas_resistance_adc: high_gas.0,
                    gas_range: high_gas.1,
                    ..environmental
                },
            ),
        ] {
            let mut sensor = initialized_sensor(variant);
            let mut encoded = field(2, 0xfe, true, true, true, environmental, low_gas, high_gas);
            match variant {
                Variant::GasLow => encoded[16] &= !(GAS_VALID_MASK | HEATER_STABLE_MASK),
                Variant::GasHigh => encoded[14] &= !(GAS_VALID_MASK | HEATER_STABLE_MASK),
            }
            sensor.interface.field_reads.push(encoded);
            sensor.interface.bytes[usize::from(REG_RES_HEAT0 + 2)] = 0xa1;
            sensor.interface.bytes[usize::from(REG_IDAC_HEAT0 + 2)] = 0xb2;
            sensor.interface.bytes[usize::from(REG_GAS_WAIT0 + 2)] = 0xc3;

            let measurements = sensor.measurements(OperationMode::Forced).unwrap();
            let decoded = measurements.as_slice()[0];
            let mut calibration = reference_calibration();
            let expected_values = compensation::compensate(expected_gas, variant, &mut calibration);

            assert_eq!(measurements.len(), 1);
            assert_eq!(decoded.status.bits(), 0xb0);
            assert_eq!(decoded.gas_index, 2);
            assert_eq!(decoded.measurement_index, 0xfe);
            assert_eq!(decoded.heater_resistance, 0xa1);
            assert_eq!(decoded.heater_current, 0xb2);
            assert_eq!(decoded.gas_wait, 0xc3);
            assert_eq!(decoded.raw, expected_gas);
            assert_eq!(decoded.values, expected_values);
        }
    }

    #[test]
    fn forced_read_polls_until_new_data_and_reports_no_new_data_as_empty() {
        let environmental = raw_sample();
        let stale = field(0, 1, false, true, true, environmental, (700, 8), (333, 4));
        let fresh = field(0, 2, true, true, true, environmental, (700, 8), (333, 4));
        let mut sensor = initialized_sensor(Variant::GasHigh);
        sensor.delay.nanoseconds.clear();
        sensor.interface.field_reads = std::vec![stale, stale, fresh];
        let measurements = sensor.measurements(OperationMode::Forced).unwrap();
        assert_eq!(measurements.as_slice()[0].measurement_index, 2);
        assert_eq!(
            sensor.delay.nanoseconds,
            std::vec![POLL_DELAY_US * 1_000; 2]
        );
        assert_eq!(sensor.interface.read_count(REG_FIELD0), 3);

        let mut sensor = initialized_sensor(Variant::GasHigh);
        sensor.delay.nanoseconds.clear();
        sensor.interface.field_reads = std::vec![stale; FORCED_DATA_ATTEMPTS];
        let measurements = sensor.measurements(OperationMode::Forced).unwrap();
        assert!(measurements.is_empty());
        assert_eq!(
            sensor.delay.nanoseconds,
            std::vec![POLL_DELAY_US * 1_000; 5]
        );
        assert_eq!(
            sensor.interface.read_count(REG_FIELD0),
            FORCED_DATA_ATTEMPTS
        );
    }

    #[test]
    fn profile_read_compacts_new_fields_and_sorts_wraparound_indices() {
        let environmental = raw_sample();
        let fields = [
            field(0, 100, false, true, true, environmental, (700, 8), (333, 4)),
            field(1, 255, true, true, true, environmental, (700, 8), (333, 4)),
            field(2, 0, true, true, true, environmental, (700, 8), (333, 4)),
        ];
        let mut sensor = initialized_sensor(Variant::GasHigh);
        sensor.interface.set_profile_fields(&fields);
        sensor.interface.bytes[usize::from(REG_IDAC_HEAT0 + 1)] = 0x11;
        sensor.interface.bytes[usize::from(REG_IDAC_HEAT0 + 2)] = 0x12;
        sensor.interface.bytes[usize::from(REG_RES_HEAT0 + 1)] = 0x21;
        sensor.interface.bytes[usize::from(REG_RES_HEAT0 + 2)] = 0x22;
        sensor.interface.bytes[usize::from(REG_GAS_WAIT0 + 1)] = 0x31;
        sensor.interface.bytes[usize::from(REG_GAS_WAIT0 + 2)] = 0x32;

        let measurements = sensor.measurements(OperationMode::Sequential).unwrap();
        assert_eq!(measurements.len(), 2);
        assert_eq!(
            measurements
                .iter()
                .map(|value| value.measurement_index)
                .collect::<Vec<_>>(),
            std::vec![255, 0]
        );
        assert_eq!(measurements.as_slice()[0].heater_current, 0x11);
        assert_eq!(measurements.as_slice()[0].heater_resistance, 0x21);
        assert_eq!(measurements.as_slice()[0].gas_wait, 0x31);
        assert_eq!(measurements.as_slice()[1].heater_current, 0x12);
        assert_eq!(measurements.as_slice()[1].heater_resistance, 0x22);
        assert_eq!(measurements.as_slice()[1].gas_wait, 0x32);
    }

    #[test]
    fn forced_heater_programs_registers_and_variant_specific_control_bits() {
        let mut sensor = initialized_sensor(Variant::GasHigh);
        sensor.interface.bytes[usize::from(REG_CTRL_GAS_0)] = 0xff;
        sensor.interface.bytes[usize::from(REG_CTRL_GAS_1)] = 0xc0;
        let configuration = HeaterConfiguration::Forced {
            enabled: true,
            temperature_celsius: 320,
            duration_ms: 64,
        };
        sensor.set_heater_configuration(&configuration).unwrap();

        assert_eq!(
            sensor.interface.bytes[usize::from(REG_RES_HEAT0)],
            compensation::calculate_heater_resistance(320, 25, &reference_calibration())
        );
        assert_eq!(sensor.interface.bytes[usize::from(REG_GAS_WAIT0)], 0x50);
        assert_eq!(sensor.interface.bytes[usize::from(REG_CTRL_GAS_0)], 0xf7);
        assert_eq!(sensor.interface.bytes[usize::from(REG_CTRL_GAS_1)], 0xe0);

        let disabled = HeaterConfiguration::Forced {
            enabled: false,
            temperature_celsius: 200,
            duration_ms: 10,
        };
        sensor.set_heater_configuration(&disabled).unwrap();
        assert_eq!(sensor.interface.bytes[usize::from(REG_CTRL_GAS_0)], 0xff);
        assert_eq!(sensor.interface.bytes[usize::from(REG_CTRL_GAS_1)], 0xc0);
    }

    #[test]
    fn sequential_heater_programs_all_steps_and_profile_length() {
        let temperatures = [200_u16, 300, 500];
        let durations = [63_u16, 64, 4_032];
        let mut sensor = initialized_sensor(Variant::GasLow);
        sensor.interface.bytes[usize::from(REG_CTRL_GAS_0)] = 0xff;
        sensor.interface.bytes[usize::from(REG_CTRL_GAS_1)] = 0xc0;
        sensor
            .set_heater_configuration(&HeaterConfiguration::Sequential {
                enabled: true,
                temperatures_celsius: &temperatures,
                durations_ms: &durations,
            })
            .unwrap();

        let registers = sensor.heater_registers().unwrap();
        let expected_resistance = temperatures.map(|temperature| {
            compensation::calculate_heater_resistance(temperature, 25, &reference_calibration())
        });
        assert_eq!(&registers.resistance[..3], &expected_resistance);
        assert_eq!(&registers.gas_wait[..3], &[0x3f, 0x50, 0xff]);
        assert_eq!(sensor.interface.bytes[usize::from(REG_CTRL_GAS_0)], 0xf7);
        assert_eq!(sensor.interface.bytes[usize::from(REG_CTRL_GAS_1)], 0xd3);
    }

    #[test]
    fn parallel_heater_programs_shared_duration_raw_waits_and_control() {
        let temperatures = [250_u16, 350];
        let durations = [0x0123_u16, 0x0045];
        let mut sensor = initialized_sensor(Variant::GasHigh);
        sensor.interface.bytes[usize::from(REG_CTRL_GAS_0)] = 0xff;
        sensor.interface.bytes[usize::from(REG_CTRL_GAS_1)] = 0xc0;
        sensor
            .set_heater_configuration(&HeaterConfiguration::Parallel {
                enabled: true,
                temperatures_celsius: &temperatures,
                durations_ms: &durations,
                shared_duration_ms: 100,
            })
            .unwrap();

        assert_eq!(
            sensor.interface.bytes[usize::from(REG_SHARED_HEATER_DURATION)],
            compensation::encode_shared_heater_duration(100)
        );
        assert_eq!(
            &sensor.interface.bytes[usize::from(REG_GAS_WAIT0)..usize::from(REG_GAS_WAIT0) + 2],
            &[0x23, 0x45]
        );
        assert_eq!(sensor.interface.bytes[usize::from(REG_CTRL_GAS_0)], 0xf7);
        assert_eq!(sensor.interface.bytes[usize::from(REG_CTRL_GAS_1)], 0xe2);
    }

    #[test]
    fn invalid_raw_operations_and_reserved_fields_are_rejected_without_bus_writes() {
        let mut sensor = initialized_sensor(Variant::GasHigh);
        let writes_before = sensor.interface.writes.len();
        assert_eq!(
            sensor.write_registers(&[0x10, 0x11], &[0x22]),
            Err(Error::InvalidConfiguration(
                ConfigError::RegisterWriteLengthMismatch {
                    registers: 2,
                    values: 1,
                }
            ))
        );
        assert_eq!(
            sensor.write_registers(&[], &[]),
            Err(Error::InvalidConfiguration(
                ConfigError::InvalidRegisterWriteLength { length: 0 }
            ))
        );
        assert_eq!(
            sensor.write_registers(&[0; MAX_REGISTER_WRITES + 1], &[0; MAX_REGISTER_WRITES + 1]),
            Err(Error::InvalidConfiguration(
                ConfigError::InvalidRegisterWriteLength {
                    length: MAX_REGISTER_WRITES + 1,
                }
            ))
        );
        assert_eq!(sensor.interface.writes.len(), writes_before);

        sensor.interface.bytes[usize::from(REG_CTRL_HUM)] = 6;
        assert_eq!(
            sensor.configuration(),
            Err(Error::InvalidRegisterValue {
                register: REG_CTRL_HUM,
                value: 6,
            })
        );
        assert_eq!(
            sensor.measurements(OperationMode::Sleep),
            Err(Error::InvalidConfiguration(
                ConfigError::UnsupportedDataMode
            ))
        );
    }

    #[test]
    fn raw_register_access_preserves_the_exact_bus_error() {
        let mut sensor = initialized_sensor(Variant::GasHigh);
        sensor.interface.fail_next_read(0x42);
        assert_eq!(
            sensor.read_registers(0x42, &mut [0; 2]),
            Err(Error::Bus(TestBusError::Read {
                register: 0x42,
                occurrence: 1,
            }))
        );

        sensor.interface.fail_next_write(0x43);
        assert_eq!(
            sensor.write_registers(&[0x43], &[0xaa]),
            Err(Error::Bus(TestBusError::Write {
                register: 0x43,
                occurrence: 1,
            }))
        );
    }

    #[test]
    fn invalid_profile_gas_index_reports_the_exact_field_register() {
        let environmental = raw_sample();
        let invalid = field(10, 1, true, true, true, environmental, (700, 8), (333, 4));
        let valid = field(0, 2, true, true, true, environmental, (700, 8), (333, 4));
        let mut sensor = initialized_sensor(Variant::GasHigh);
        sensor
            .interface
            .set_profile_fields(&[valid, invalid, valid]);
        assert_eq!(
            sensor.measurements(OperationMode::Parallel),
            Err(Error::InvalidRegisterValue {
                register: REG_FIELD0 + LEN_FIELD as u8,
                value: 10,
            })
        );
    }

    #[test]
    fn operation_mode_timeout_has_bosch_poll_count_and_delay() {
        let mut sensor = initialized_sensor(Variant::GasHigh);
        sensor.interface.bytes[usize::from(REG_CTRL_MEAS)] = 0xa5;
        sensor.interface.hold_mode_awake = true;
        sensor.delay.nanoseconds.clear();
        assert_eq!(
            sensor.set_operation_mode(OperationMode::Sequential),
            Err(Error::Timeout)
        );
        assert_eq!(
            sensor.interface.read_count(REG_CTRL_MEAS),
            MODE_CHANGE_ATTEMPTS
        );
        assert_eq!(
            sensor.interface.write_count(REG_CTRL_MEAS),
            MODE_CHANGE_ATTEMPTS
        );
        assert_eq!(sensor.delay.nanoseconds.len(), MODE_CHANGE_ATTEMPTS);
        assert!(sensor
            .delay
            .nanoseconds
            .iter()
            .all(|delay| *delay == POLL_DELAY_US * 1_000));
    }

    #[test]
    fn operation_mode_preserves_bus_errors_at_each_transaction_boundary() {
        let mut sensor = initialized_sensor(Variant::GasHigh);
        sensor.interface.fail_next_read(REG_CTRL_MEAS);
        assert_eq!(
            sensor.set_operation_mode(OperationMode::Forced),
            Err(Error::Bus(TestBusError::Read {
                register: REG_CTRL_MEAS,
                occurrence: 1,
            }))
        );

        let mut sensor = initialized_sensor(Variant::GasHigh);
        sensor.interface.bytes[usize::from(REG_CTRL_MEAS)] = OperationMode::Forced.register_value();
        sensor.interface.fail_next_write(REG_CTRL_MEAS);
        assert_eq!(
            sensor.set_operation_mode(OperationMode::Sequential),
            Err(Error::Bus(TestBusError::Write {
                register: REG_CTRL_MEAS,
                occurrence: 1,
            }))
        );

        let mut sensor = initialized_sensor(Variant::GasHigh);
        sensor.interface.fail_next_write(REG_CTRL_MEAS);
        assert_eq!(
            sensor.set_operation_mode(OperationMode::Sequential),
            Err(Error::Bus(TestBusError::Write {
                register: REG_CTRL_MEAS,
                occurrence: 1,
            }))
        );
    }

    #[test]
    fn self_test_reports_missing_data_invalid_current_and_invalid_gas() {
        let environmental = raw_sample();
        let stale = field(0, 1, false, true, true, environmental, (700, 8), (333, 4));
        let mut sensor = initialized_sensor(Variant::GasHigh);
        sensor.interface.field_reads = std::vec![stale; FORCED_DATA_ATTEMPTS];
        assert_eq!(sensor.self_test(), Err(Error::Timeout));

        let valid = field(0, 1, true, true, true, environmental, (700, 8), (333, 4));
        let mut sensor = initialized_sensor(Variant::GasHigh);
        sensor.interface.field_reads.push(valid);
        sensor.interface.bytes[usize::from(REG_IDAC_HEAT0)] = 0;
        assert_eq!(
            sensor.self_test(),
            Err(Error::SelfTestFailed(SelfTestFailure::InvalidHeaterCurrent))
        );

        let gas_invalid = field(0, 1, true, false, true, environmental, (700, 8), (333, 4));
        let mut sensor = initialized_sensor(Variant::GasHigh);
        sensor.interface.field_reads.push(gas_invalid);
        sensor.interface.bytes[usize::from(REG_IDAC_HEAT0)] = 1;
        assert_eq!(
            sensor.self_test(),
            Err(Error::SelfTestFailed(
                SelfTestFailure::InvalidGasMeasurement
            ))
        );
    }

    #[test]
    fn self_test_runs_the_full_alternating_heater_sequence_before_range_analysis() {
        let environmental = raw_sample();
        let valid = field(0, 1, true, true, true, environmental, (700, 8), (333, 4));
        let mut sensor = initialized_sensor(Variant::GasHigh);
        sensor.interface.field_reads = std::vec![valid; 7];
        sensor.interface.bytes[usize::from(REG_IDAC_HEAT0)] = 1;
        sensor.delay.nanoseconds.clear();

        assert_eq!(
            sensor.self_test(),
            Err(Error::SelfTestFailed(
                SelfTestFailure::MeasurementOutOfRange
            ))
        );
        assert_eq!(
            sensor
                .delay
                .nanoseconds
                .iter()
                .filter(|delay| **delay == 1_000_000_000)
                .count(),
            1
        );
        assert_eq!(
            sensor
                .delay
                .nanoseconds
                .iter()
                .filter(|delay| **delay == 2_000_000_000)
                .count(),
            6
        );

        let resistance_writes = sensor
            .interface
            .writes
            .iter()
            .filter(|(register, _)| *register == REG_RES_HEAT0)
            .map(|(_, data)| data[0])
            .collect::<Vec<_>>();
        let high = compensation::calculate_heater_resistance(350, 25, &reference_calibration());
        let low = compensation::calculate_heater_resistance(150, 25, &reference_calibration());
        assert_eq!(
            resistance_writes,
            std::vec![high, high, low, high, low, high, low]
        );
    }
}
