use core::fmt;

use embedded_hal::i2c::{I2c, Operation};

use super::RegisterInterface;

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

/// `BME68x` I2C transport using an `embedded-hal` 1.0 bus.
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

    fn read(&mut self, register: u8, data: &mut [u8]) -> Result<(), Self::Error> {
        self.bus.write_read(self.address.into(), &[register], data)
    }

    fn write(&mut self, register: u8, data: &[u8]) -> Result<(), Self::Error> {
        let command = [register];
        self.bus.transaction(
            self.address.into(),
            &mut [Operation::Write(&command), Operation::Write(data)],
        )
    }

    fn write_pairs(&mut self, registers: &[u8], values: &[u8]) -> Result<(), Self::Error> {
        if registers.is_empty() {
            return Ok(());
        }
        if registers.len() > 10 || registers.len() != values.len() {
            for (&register, &value) in registers.iter().zip(values) {
                self.write(register, core::slice::from_ref(&value))?;
            }
            return Ok(());
        }
        // Bosch's register API sends up to ten interleaved address/value pairs
        // in one I2C write. The first address is the I2C register command.
        let mut wire = [0_u8; 20];
        for (index, (&register, &value)) in registers.iter().zip(values).enumerate() {
            wire[index * 2] = register;
            wire[index * 2 + 1] = value;
        }
        self.bus
            .write(self.address.into(), &wire[..registers.len() * 2])
    }
}
