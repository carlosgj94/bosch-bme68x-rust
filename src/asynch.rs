// Portions of this file are derived from Bosch Sensortec's
// BME68x SensorAPI v4.4.8.
// Copyright (c) 2023 Bosch Sensortec GmbH. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause

//! Asynchronous `embedded-hal-async` BME680/BME688 driver.
//!
//! This module awaits every bus operation and delay while keeping the typed
//! API, fixed-size buffers, and error semantics of the blocking frontend.

use embedded_hal_async::delay::DelayNs;
use embedded_hal_async::i2c::{I2c, Operation as I2cOperation};
use embedded_hal_async::spi::{Operation as SpiOperation, SpiDevice};

use core::fmt;

use crate::compensation;
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

const MEM_PAGE_MASK: u8 = 0x10;
const MEM_PAGE0: u8 = 0x10;
const MEM_PAGE1: u8 = 0x00;
const SPI_READ_MASK: u8 = 0x80;
const SPI_WRITE_MASK: u8 = 0x7f;

/// One of the two valid 7-bit BME680/BME688 I2C addresses.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
#[repr(u8)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum I2cAddress {
    /// `0x76`, selected when SDO is tied low.
    Low = 0x76,
    /// `0x77`, selected when SDO is tied high.
    High = 0x77,
}

impl From<I2cAddress> for u8 {
    fn from(value: I2cAddress) -> Self {
        value as Self
    }
}

/// Error returned when converting an unsupported address.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct InvalidI2cAddress(pub u8);

impl fmt::Display for InvalidI2cAddress {
    fn fmt(&self, formatter: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(formatter, "invalid BME68x I2C address 0x{:02x}", self.0)
    }
}

impl TryFrom<u8> for I2cAddress {
    type Error = InvalidI2cAddress;

    fn try_from(value: u8) -> Result<Self, Self::Error> {
        match value {
            0x76 => Ok(Self::Low),
            0x77 => Ok(Self::High),
            other => Err(InvalidI2cAddress(other)),
        }
    }
}

/// Low-level asynchronous logical-register access used by the driver.
///
/// Most users should construct [`I2cInterface`] or [`SpiInterface`] instead of
/// implementing this trait themselves.
#[allow(async_fn_in_trait)]
pub trait RegisterInterface {
    /// Error produced by the underlying bus.
    type Error;

    /// Read consecutive bytes beginning at a logical `BME68x` register address.
    async fn read(&mut self, register: u8, data: &mut [u8]) -> Result<(), Self::Error>;

    /// Write `data` beginning at a logical `BME68x` register address.
    async fn write(&mut self, register: u8, data: &[u8]) -> Result<(), Self::Error>;

    /// Write a list of logical register/value pairs.
    ///
    /// The default implementation performs one write transaction per pair.
    async fn write_pairs(&mut self, registers: &[u8], values: &[u8]) -> Result<(), Self::Error> {
        for (&register, &value) in registers.iter().zip(values) {
            self.write(register, core::slice::from_ref(&value)).await?;
        }
        Ok(())
    }
}

/// `BME68x` I2C transport using an `embedded-hal-async` 1.0 bus.
#[derive(Debug)]
pub struct I2cInterface<I2C> {
    bus: I2C,
    address: I2cAddress,
}

impl<I2C> I2cInterface<I2C> {
    /// Wrap an I2C bus and select the sensor address.
    pub const fn new(bus: I2C, address: I2cAddress) -> Self {
        Self { bus, address }
    }

    /// Return the selected address.
    #[must_use]
    pub const fn address(&self) -> I2cAddress {
        self.address
    }

    /// Release the owned I2C bus.
    pub fn release(self) -> I2C {
        self.bus
    }
}

impl<I2C> RegisterInterface for I2cInterface<I2C>
where
    I2C: I2c,
{
    type Error = I2C::Error;

    async fn read(&mut self, register: u8, data: &mut [u8]) -> Result<(), Self::Error> {
        self.bus
            .write_read(self.address.into(), &[register], data)
            .await
    }

    async fn write(&mut self, register: u8, data: &[u8]) -> Result<(), Self::Error> {
        let command = [register];
        self.bus
            .transaction(
                self.address.into(),
                &mut [I2cOperation::Write(&command), I2cOperation::Write(data)],
            )
            .await
    }

    async fn write_pairs(&mut self, registers: &[u8], values: &[u8]) -> Result<(), Self::Error> {
        if registers.is_empty() {
            return Ok(());
        }
        if registers.len() > MAX_REGISTER_WRITES || registers.len() != values.len() {
            for (&register, &value) in registers.iter().zip(values) {
                self.write(register, core::slice::from_ref(&value)).await?;
            }
            return Ok(());
        }

        // Bosch's register API writes up to ten interleaved address/value
        // pairs in one I2C transaction.
        let mut wire = [0_u8; MAX_REGISTER_WRITES * 2];
        for (index, (&register, &value)) in registers.iter().zip(values).enumerate() {
            wire[index * 2] = register;
            wire[index * 2 + 1] = value;
        }
        self.bus
            .write(self.address.into(), &wire[..registers.len() * 2])
            .await
    }
}

/// `BME68x` SPI transport using an `embedded-hal-async` 1.0 `SpiDevice`.
///
/// `SpiDevice` owns chip-select handling and bus locking, making this adapter
/// suitable for a shared SPI bus.
#[derive(Debug)]
pub struct SpiInterface<SPI> {
    device: SPI,
    memory_page: Option<u8>,
}

impl<SPI> SpiInterface<SPI> {
    /// Wrap an SPI device. The current sensor memory page is discovered lazily.
    pub const fn new(device: SPI) -> Self {
        Self {
            device,
            memory_page: None,
        }
    }

    /// Release the owned SPI device.
    pub fn release(self) -> SPI {
        self.device
    }
}

impl<SPI> SpiInterface<SPI>
where
    SPI: SpiDevice<u8>,
{
    async fn raw_read(&mut self, register: u8, data: &mut [u8]) -> Result<(), SPI::Error> {
        let command = [register | SPI_READ_MASK];
        self.device
            .transaction(&mut [SpiOperation::Write(&command), SpiOperation::Read(data)])
            .await
    }

    async fn raw_write(&mut self, register: u8, data: &[u8]) -> Result<(), SPI::Error> {
        let command = [register & SPI_WRITE_MASK];
        self.device
            .transaction(&mut [SpiOperation::Write(&command), SpiOperation::Write(data)])
            .await
    }

    async fn select_page(&mut self, logical_register: u8) -> Result<(), SPI::Error> {
        let wanted = if logical_register > 0x7f {
            MEM_PAGE1
        } else {
            MEM_PAGE0
        };

        if self.memory_page.is_none() {
            let mut value = 0;
            self.raw_read(
                crate::registers::REG_MEM_PAGE,
                core::slice::from_mut(&mut value),
            )
            .await?;
            self.memory_page = Some(value & MEM_PAGE_MASK);
        }

        if self.memory_page != Some(wanted) {
            let mut value = 0;
            self.raw_read(
                crate::registers::REG_MEM_PAGE,
                core::slice::from_mut(&mut value),
            )
            .await?;
            value = (value & !MEM_PAGE_MASK) | wanted;
            self.raw_write(
                crate::registers::REG_MEM_PAGE,
                core::slice::from_ref(&value),
            )
            .await?;
            self.memory_page = Some(wanted);
        }

        Ok(())
    }
}

impl<SPI> RegisterInterface for SpiInterface<SPI>
where
    SPI: SpiDevice<u8>,
{
    type Error = SPI::Error;

    async fn read(&mut self, register: u8, data: &mut [u8]) -> Result<(), Self::Error> {
        self.select_page(register).await?;
        self.raw_read(register, data).await
    }

    async fn write(&mut self, register: u8, data: &[u8]) -> Result<(), Self::Error> {
        self.select_page(register).await?;
        let result = self.raw_write(register, data).await;
        if register == crate::registers::REG_MEM_PAGE {
            self.memory_page = if result.is_ok() {
                data.first().map(|value| value & MEM_PAGE_MASK)
            } else {
                None
            };
        }
        // A reset can change the sensor's active SPI memory page. Invalidate
        // the cache even if the bus reports an error: the command may already
        // have reached the device before that error was observed.
        if register == REG_SOFT_RESET && data.first() == Some(&SOFT_RESET_COMMAND) {
            self.memory_page = None;
        }
        result
    }

    async fn write_pairs(&mut self, registers: &[u8], values: &[u8]) -> Result<(), Self::Error> {
        if registers.is_empty() {
            return Ok(());
        }
        if registers.len() > MAX_REGISTER_WRITES || registers.len() != values.len() {
            for (&register, &value) in registers.iter().zip(values) {
                self.write(register, core::slice::from_ref(&value)).await?;
            }
            return Ok(());
        }
        if registers.contains(&crate::registers::REG_MEM_PAGE) {
            for (&register, &value) in registers.iter().zip(values) {
                self.write(register, core::slice::from_ref(&value)).await?;
            }
            return Ok(());
        }

        let first_page = registers.first().map(|register| *register > 0x7f);
        let one_page = registers
            .iter()
            .all(|register| Some(*register > 0x7f) == first_page);
        if !one_page {
            for (&register, &value) in registers.iter().zip(values) {
                self.write(register, core::slice::from_ref(&value)).await?;
            }
            return Ok(());
        }

        self.select_page(registers[0]).await?;
        let mut wire = [0_u8; MAX_REGISTER_WRITES * 2];
        for (index, (&register, &value)) in registers.iter().zip(values).enumerate() {
            wire[index * 2] = register & SPI_WRITE_MASK;
            wire[index * 2 + 1] = value;
        }
        let result = self.raw_write(wire[0], &wire[1..registers.len() * 2]).await;
        if registers
            .iter()
            .zip(values)
            .any(|(&register, &value)| register == REG_SOFT_RESET && value == SOFT_RESET_COMMAND)
        {
            self.memory_page = None;
        }
        result
    }
}

const MODE_CHANGE_ATTEMPTS: usize = 100;
const FORCED_DATA_ATTEMPTS: usize = 5;

/// An asynchronous BME680/BME688 sensor instance.
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
    pub async fn new(interface: I, delay: D) -> Result<Self, Error<I::Error>> {
        Self::new_with_ambient_temperature(interface, delay, 25).await
    }

    /// Reset and initialize a sensor with an explicit ambient temperature.
    ///
    /// # Errors
    ///
    /// Returns a bus, identity, or sensor-register error if initialization fails.
    pub async fn new_with_ambient_temperature(
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
        sensor.initialize().await?;
        Ok(sensor)
    }

    /// Re-run reset, identity, variant, and calibration discovery.
    ///
    /// # Errors
    ///
    /// Returns a bus, identity, or sensor-register error if discovery fails.
    pub async fn initialize(&mut self) -> Result<(), Error<I::Error>> {
        self.soft_reset().await?;

        let mut chip_id = 0;
        self.read_registers(REG_CHIP_ID, core::slice::from_mut(&mut chip_id))
            .await?;
        if chip_id != CHIP_ID {
            return Err(Error::UnexpectedChipId { found: chip_id });
        }
        self.chip_id = chip_id;

        let mut variant = 0;
        self.read_registers(REG_VARIANT_ID, core::slice::from_mut(&mut variant))
            .await?;
        self.variant = Variant::from_register(variant).ok_or(Error::InvalidRegisterValue {
            register: REG_VARIANT_ID,
            value: variant,
        })?;

        let mut block_1 = [0_u8; LEN_COEFF1];
        let mut block_2 = [0_u8; LEN_COEFF2];
        let mut block_3 = [0_u8; LEN_COEFF3];
        self.read_registers(REG_COEFF1, &mut block_1).await?;
        self.read_registers(REG_COEFF2, &mut block_2).await?;
        self.read_registers(REG_COEFF3, &mut block_3).await?;
        self.calibration = CalibrationData::from_register_blocks(&block_1, &block_2, &block_3);
        Ok(())
    }

    /// Issue the documented `0xb6` soft-reset command and wait 10 ms.
    ///
    /// # Errors
    ///
    /// Returns the concrete bus error if the reset write fails.
    pub async fn soft_reset(&mut self) -> Result<(), Error<I::Error>> {
        self.interface
            .write_pairs(&[REG_SOFT_RESET], &[SOFT_RESET_COMMAND])
            .await
            .map_err(Error::Bus)?;
        self.delay.delay_us(RESET_DELAY_US).await;
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
    pub async fn read_registers(
        &mut self,
        register: u8,
        data: &mut [u8],
    ) -> Result<(), Error<I::Error>> {
        self.interface
            .read(register, data)
            .await
            .map_err(Error::Bus)
    }

    /// Write between one and ten logical register/value pairs.
    ///
    /// # Errors
    ///
    /// Returns a configuration error for invalid slices or the concrete bus
    /// error if the write fails.
    pub async fn write_registers(
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
            .await
            .map_err(Error::Bus)
    }

    /// Read the current operation mode.
    ///
    /// # Errors
    ///
    /// Returns the concrete bus error if the mode register cannot be read.
    pub async fn operation_mode(&mut self) -> Result<OperationMode, Error<I::Error>> {
        let mut value = 0;
        self.read_registers(REG_CTRL_MEAS, core::slice::from_mut(&mut value))
            .await?;
        // The two-bit mask means every possible value maps to a mode.
        Ok(OperationMode::from_register(value & MODE_MASK).unwrap_or(OperationMode::Sleep))
    }

    /// Put the sensor to sleep, then enter the requested mode.
    ///
    /// # Errors
    ///
    /// Returns a bus error or [`Error::Timeout`] if sleep is not reached.
    pub async fn set_operation_mode(&mut self, mode: OperationMode) -> Result<(), Error<I::Error>> {
        let mut register_value = 0;
        let mut reached_sleep = false;
        for _ in 0..MODE_CHANGE_ATTEMPTS {
            self.read_registers(REG_CTRL_MEAS, core::slice::from_mut(&mut register_value))
                .await?;
            if register_value & MODE_MASK == OperationMode::Sleep.register_value() {
                reached_sleep = true;
                break;
            }
            register_value &= !MODE_MASK;
            self.write_registers(&[REG_CTRL_MEAS], &[register_value])
                .await?;
            self.delay.delay_us(POLL_DELAY_US).await;
        }
        if !reached_sleep {
            return Err(Error::Timeout);
        }

        if mode != OperationMode::Sleep {
            register_value = (register_value & !MODE_MASK) | mode.register_value();
            self.write_registers(&[REG_CTRL_MEAS], &[register_value])
                .await?;
        }
        Ok(())
    }

    /// Read the current oversampling, filter, and standby settings.
    ///
    /// # Errors
    ///
    /// Returns a bus error or [`Error::InvalidRegisterValue`] for reserved
    /// register encodings.
    pub async fn configuration(&mut self) -> Result<Configuration, Error<I::Error>> {
        let mut registers = [0_u8; 5];
        self.read_registers(REG_CTRL_GAS_1, &mut registers).await?;

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
    pub async fn set_configuration(
        &mut self,
        configuration: &Configuration,
    ) -> Result<(), Error<I::Error>> {
        let previous_mode = self.operation_mode().await?;
        self.set_operation_mode(OperationMode::Sleep).await?;

        let addresses = [0x71, 0x72, 0x73, 0x74, 0x75];
        let mut values = [0_u8; 5];
        self.read_registers(REG_CTRL_GAS_1, &mut values).await?;

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
        self.write_registers(&addresses, &values).await?;

        if previous_mode != OperationMode::Sleep {
            self.set_operation_mode(previous_mode).await?;
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
    #[allow(clippy::too_many_lines)]
    pub async fn set_heater_configuration(
        &mut self,
        configuration: &HeaterConfiguration<'_>,
    ) -> Result<(), Error<I::Error>> {
        self.set_operation_mode(OperationMode::Sleep).await?;

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
                self.write_registers(&[REG_RES_HEAT0], &[resistance])
                    .await?;
                self.write_registers(&[REG_GAS_WAIT0], &[gas_wait]).await?;
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
                self.write_registers(&resistance_registers[..len], &resistance[..len])
                    .await?;
                self.write_registers(&wait_registers[..len], &gas_wait[..len])
                    .await?;
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
                self.write_registers(&[REG_SHARED_HEATER_DURATION], &[shared])
                    .await?;

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
                self.write_registers(&resistance_registers[..len], &resistance[..len])
                    .await?;
                self.write_registers(&wait_registers[..len], &gas_wait[..len])
                    .await?;
                u8::try_from(len).unwrap_or(0)
            }
        };

        let mut control = [0_u8; 2];
        self.read_registers(REG_CTRL_GAS_0, &mut control).await?;
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
            .await
    }

    /// Read all ten raw heater-resistance and gas-wait profile registers.
    ///
    /// # Errors
    ///
    /// Returns the concrete bus error if either register read fails.
    pub async fn heater_registers(&mut self) -> Result<HeaterRegisters, Error<I::Error>> {
        let mut registers = HeaterRegisters::default();
        self.read_registers(REG_RES_HEAT0, &mut registers.resistance)
            .await?;
        self.read_registers(REG_GAS_WAIT0, &mut registers.gas_wait)
            .await?;
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
    pub async fn measurements(
        &mut self,
        mode: OperationMode,
    ) -> Result<Measurements, Error<I::Error>> {
        match mode {
            OperationMode::Forced => self.forced_measurement().await,
            OperationMode::Parallel | OperationMode::Sequential => {
                self.profile_measurements().await
            }
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
    pub async fn self_test(&mut self) -> Result<(), Error<I::Error>> {
        self.initialize().await?;
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
        self.set_heater_configuration(&initial_heater).await?;
        self.set_configuration(&configuration).await?;
        self.set_operation_mode(OperationMode::Forced).await?;
        self.delay.delay_us(1_000_000).await;
        let initial = self
            .measurements(OperationMode::Forced)
            .await?
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
            self.set_heater_configuration(&heater).await?;
            self.set_configuration(&configuration).await?;
            self.set_operation_mode(OperationMode::Forced).await?;
            self.delay.delay_us(2_000_000).await;
            *measurement = self
                .measurements(OperationMode::Forced)
                .await?
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

    async fn forced_measurement(&mut self) -> Result<Measurements, Error<I::Error>> {
        for _ in 0..FORCED_DATA_ATTEMPTS {
            let mut field = [0_u8; LEN_FIELD];
            self.read_registers(REG_FIELD0, &mut field).await?;
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
                )
                .await?;
                self.read_registers(
                    REG_IDAC_HEAT0 + gas_index,
                    core::slice::from_mut(&mut heater_current),
                )
                .await?;
                self.read_registers(
                    REG_GAS_WAIT0 + gas_index,
                    core::slice::from_mut(&mut gas_wait),
                )
                .await?;
                let measurement =
                    self.decode_field(&field, heater_current, heater_resistance, gas_wait);
                return Ok(Measurements::new(
                    [measurement, Measurement::default(), Measurement::default()],
                    1,
                ));
            }
            self.delay.delay_us(POLL_DELAY_US).await;
        }
        Ok(Measurements::default())
    }

    async fn profile_measurements(&mut self) -> Result<Measurements, Error<I::Error>> {
        let mut fields = [0_u8; LEN_FIELD * FIELD_COUNT];
        let mut settings = [0_u8; MAX_PROFILE_LEN * 3];
        self.read_registers(REG_FIELD0, &mut fields).await?;
        self.read_registers(REG_IDAC_HEAT0, &mut settings).await?;

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
    extern crate std;

    use core::convert::Infallible;
    use core::future::Future;
    use core::task::{Context, Poll, Waker};

    use super::*;
    use std::sync::Arc;
    use std::task::Wake;
    use std::vec::Vec;

    struct NoopWake;

    impl Wake for NoopWake {
        fn wake(self: Arc<Self>) {}
    }

    fn block_on<F: Future>(future: F) -> F::Output {
        let waker = Waker::from(Arc::new(NoopWake));
        let mut context = Context::from_waker(&waker);
        let mut future = core::pin::pin!(future);
        loop {
            match future.as_mut().poll(&mut context) {
                Poll::Ready(output) => return output,
                Poll::Pending => std::thread::yield_now(),
            }
        }
    }

    #[derive(Debug, Default)]
    struct RecordingI2c {
        addresses: Vec<u8>,
        transactions: Vec<Vec<Vec<u8>>>,
    }

    impl embedded_hal::i2c::ErrorType for RecordingI2c {
        type Error = Infallible;
    }

    impl I2c for RecordingI2c {
        async fn transaction(
            &mut self,
            address: u8,
            operations: &mut [I2cOperation<'_>],
        ) -> Result<(), Self::Error> {
            let mut writes = Vec::new();
            for operation in operations {
                match operation {
                    I2cOperation::Write(data) => writes.push(data.to_vec()),
                    I2cOperation::Read(data) => data.fill(0),
                }
            }
            self.addresses.push(address);
            self.transactions.push(writes);
            Ok(())
        }
    }

    #[derive(Debug, Eq, PartialEq)]
    enum SpiEvent {
        Write(Vec<u8>),
        Read(usize),
    }

    #[derive(Debug, Default)]
    struct RecordingSpi {
        memory_page: u8,
        transactions: Vec<Vec<SpiEvent>>,
    }

    impl embedded_hal::spi::ErrorType for RecordingSpi {
        type Error = Infallible;
    }

    impl SpiDevice<u8> for RecordingSpi {
        async fn transaction(
            &mut self,
            operations: &mut [SpiOperation<'_, u8>],
        ) -> Result<(), Self::Error> {
            let mut command = None;
            let mut events = Vec::new();
            for operation in operations {
                match operation {
                    SpiOperation::Write(data) => {
                        if command.is_none() {
                            command = data.first().copied();
                        } else if command == Some(crate::registers::REG_MEM_PAGE & SPI_WRITE_MASK) {
                            if let Some(value) = data.first() {
                                self.memory_page = value & MEM_PAGE_MASK;
                            }
                        }
                        events.push(SpiEvent::Write(data.to_vec()));
                    }
                    SpiOperation::Read(data) => {
                        if command == Some(crate::registers::REG_MEM_PAGE | SPI_READ_MASK) {
                            if let Some(value) = data.first_mut() {
                                *value = self.memory_page;
                            }
                        } else {
                            data.fill(0xa5);
                        }
                        events.push(SpiEvent::Read(data.len()));
                    }
                    SpiOperation::Transfer(read, _) => {
                        read.fill(0);
                    }
                    SpiOperation::TransferInPlace(data) => data.fill(0),
                    SpiOperation::DelayNs(_) => {}
                }
            }
            self.transactions.push(events);
            Ok(())
        }
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
    fn async_i2c_interleaves_register_pairs_in_one_transaction() {
        let mut interface = I2cInterface::new(RecordingI2c::default(), I2cAddress::Low);
        block_on(interface.write_pairs(&[0x71, 0x72, 0x75], &[0x11, 0x22, 0x55])).unwrap();

        let bus = interface.release();
        assert_eq!(bus.addresses, std::vec![0x76]);
        assert_eq!(
            bus.transactions,
            std::vec![std::vec![std::vec![0x71, 0x11, 0x72, 0x22, 0x75, 0x55]]]
        );
    }

    #[test]
    fn async_spi_switches_pages_batches_pairs_and_invalidates_on_reset() {
        let mut interface = SpiInterface::new(RecordingSpi::default());
        let mut value = 0;

        block_on(interface.read(0x1d, core::slice::from_mut(&mut value))).unwrap();
        assert_eq!(value, 0xa5);
        assert_eq!(interface.memory_page, Some(MEM_PAGE0));

        block_on(interface.write_pairs(&[0x5a, 0x64], &[0xaa, 0xbb])).unwrap();
        let batched = interface.device.transactions.last().unwrap();
        assert_eq!(
            batched,
            &std::vec![
                SpiEvent::Write(std::vec![0x5a]),
                SpiEvent::Write(std::vec![0xaa, 0x64, 0xbb]),
            ]
        );

        block_on(interface.write(REG_SOFT_RESET, &[SOFT_RESET_COMMAND])).unwrap();
        assert_eq!(interface.memory_page, None);

        block_on(interface.read(REG_CHIP_ID, core::slice::from_mut(&mut value))).unwrap();
        assert_eq!(interface.memory_page, Some(MEM_PAGE1));

        let first_mixed_transaction = interface.device.transactions.len();
        block_on(interface.write_pairs(&[0x70, 0xd0], &[0x11, 0x22])).unwrap();
        let mixed_transactions = &interface.device.transactions[first_mixed_transaction..];
        assert!(mixed_transactions.contains(&std::vec![
            SpiEvent::Write(std::vec![0x70]),
            SpiEvent::Write(std::vec![0x11]),
        ]));
        assert!(mixed_transactions.contains(&std::vec![
            SpiEvent::Write(std::vec![0x50]),
            SpiEvent::Write(std::vec![0x22]),
        ]));
    }
}
