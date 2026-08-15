//! Bus adapters for `embedded-hal` 1.0.

mod i2c;
mod spi;

pub use i2c::{I2cAddress, I2cInterface, InvalidI2cAddress};
pub use spi::SpiInterface;

/// Low-level logical-register access used by the driver.
///
/// Most users should construct [`I2cInterface`] or [`SpiInterface`] instead of
/// implementing this trait themselves.
pub trait RegisterInterface {
    /// Error produced by the underlying bus.
    type Error;

    /// Read consecutive bytes beginning at a logical `BME68x` register address.
    ///
    /// # Errors
    ///
    /// Returns the concrete bus error if the transaction fails.
    fn read(&mut self, register: u8, data: &mut [u8]) -> Result<(), Self::Error>;

    /// Write `data` beginning at a logical `BME68x` register address.
    ///
    /// # Errors
    ///
    /// Returns the concrete bus error if the transaction fails.
    fn write(&mut self, register: u8, data: &[u8]) -> Result<(), Self::Error>;

    /// Write a list of logical register/value pairs.
    ///
    /// The default implementation performs one write transaction per pair.
    ///
    /// # Errors
    ///
    /// Returns the first concrete bus error encountered.
    fn write_pairs(&mut self, registers: &[u8], values: &[u8]) -> Result<(), Self::Error> {
        for (&register, &value) in registers.iter().zip(values) {
            self.write(register, core::slice::from_ref(&value))?;
        }
        Ok(())
    }
}
