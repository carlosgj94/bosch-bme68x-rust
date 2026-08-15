use embedded_hal::spi::{Operation, SpiDevice};

use crate::registers::{REG_MEM_PAGE, REG_SOFT_RESET, SOFT_RESET_COMMAND};

use super::RegisterInterface;

const MEM_PAGE_MASK: u8 = 0x10;
const MEM_PAGE0: u8 = 0x10;
const MEM_PAGE1: u8 = 0x00;
const SPI_READ_MASK: u8 = 0x80;
const SPI_WRITE_MASK: u8 = 0x7f;

/// `BME68x` SPI transport using an `embedded-hal` 1.0 `SpiDevice`.
///
/// `SpiDevice` owns chip-select handling and bus locking, so this adapter is
/// safe to use on a shared SPI bus.
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
    fn raw_read(&mut self, register: u8, data: &mut [u8]) -> Result<(), SPI::Error> {
        let command = [register | SPI_READ_MASK];
        self.device
            .transaction(&mut [Operation::Write(&command), Operation::Read(data)])
    }

    fn raw_write(&mut self, register: u8, data: &[u8]) -> Result<(), SPI::Error> {
        let command = [register & SPI_WRITE_MASK];
        self.device
            .transaction(&mut [Operation::Write(&command), Operation::Write(data)])
    }

    fn select_page(&mut self, logical_register: u8) -> Result<(), SPI::Error> {
        let wanted = if logical_register > 0x7f {
            MEM_PAGE1
        } else {
            MEM_PAGE0
        };

        if self.memory_page.is_none() {
            let mut value = 0;
            self.raw_read(REG_MEM_PAGE, core::slice::from_mut(&mut value))?;
            self.memory_page = Some(value & MEM_PAGE_MASK);
        }

        if self.memory_page != Some(wanted) {
            let mut value = 0;
            self.raw_read(REG_MEM_PAGE, core::slice::from_mut(&mut value))?;
            value = (value & !MEM_PAGE_MASK) | wanted;
            self.raw_write(REG_MEM_PAGE, core::slice::from_ref(&value))?;
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

    fn read(&mut self, register: u8, data: &mut [u8]) -> Result<(), Self::Error> {
        self.select_page(register)?;
        self.raw_read(register, data)
    }

    fn write(&mut self, register: u8, data: &[u8]) -> Result<(), Self::Error> {
        self.select_page(register)?;
        let result = self.raw_write(register, data);
        if register == REG_MEM_PAGE {
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
        if registers.contains(&REG_MEM_PAGE) {
            for (&register, &value) in registers.iter().zip(values) {
                self.write(register, core::slice::from_ref(&value))?;
            }
            return Ok(());
        }
        let first_page = registers.first().map(|register| *register > 0x7f);
        let one_page = registers
            .iter()
            .all(|register| Some(*register > 0x7f) == first_page);
        if !one_page {
            for (&register, &value) in registers.iter().zip(values) {
                self.write(register, core::slice::from_ref(&value))?;
            }
            return Ok(());
        }

        self.select_page(registers[0])?;
        let mut wire = [0_u8; 20];
        for (index, (&register, &value)) in registers.iter().zip(values).enumerate() {
            wire[index * 2] = register & SPI_WRITE_MASK;
            wire[index * 2 + 1] = value;
        }
        let result = self.raw_write(wire[0], &wire[1..registers.len() * 2]);
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

#[cfg(test)]
mod tests {
    extern crate std;

    use embedded_hal::spi::{ErrorKind, ErrorType};
    use std::vec::Vec;

    use super::*;

    #[derive(Clone, Copy, Debug, Eq, PartialEq)]
    struct TestError;

    impl embedded_hal::spi::Error for TestError {
        fn kind(&self) -> ErrorKind {
            ErrorKind::Other
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
        fail_next: bool,
    }

    impl ErrorType for RecordingSpi {
        type Error = TestError;
    }

    impl SpiDevice<u8> for RecordingSpi {
        fn transaction(&mut self, operations: &mut [Operation<'_, u8>]) -> Result<(), Self::Error> {
            if self.fail_next {
                self.fail_next = false;
                return Err(TestError);
            }

            let mut command = None;
            let mut events = Vec::new();
            for operation in operations {
                match operation {
                    Operation::Write(data) => {
                        if command.is_none() {
                            command = data.first().copied();
                        } else if command == Some(REG_MEM_PAGE & SPI_WRITE_MASK) {
                            if let Some(value) = data.first() {
                                self.memory_page = value & MEM_PAGE_MASK;
                            }
                        }
                        events.push(SpiEvent::Write(data.to_vec()));
                    }
                    Operation::Read(data) => {
                        if command == Some(REG_MEM_PAGE | SPI_READ_MASK) {
                            if let Some(value) = data.first_mut() {
                                *value = self.memory_page;
                            }
                        } else {
                            data.fill(0xa5);
                        }
                        events.push(SpiEvent::Read(data.len()));
                    }
                    Operation::Transfer(read, _) => read.fill(0),
                    Operation::TransferInPlace(data) => data.fill(0),
                    Operation::DelayNs(_) => {}
                }
            }
            self.transactions.push(events);
            Ok(())
        }
    }

    #[test]
    fn direct_memory_page_write_updates_cache_and_next_access_switches_page() {
        let mut interface = SpiInterface::new(RecordingSpi::default());

        interface.write(REG_MEM_PAGE, &[MEM_PAGE0]).unwrap();
        assert_eq!(interface.memory_page, Some(MEM_PAGE0));
        assert_eq!(interface.device.memory_page, MEM_PAGE0);

        let mut value = 0;
        interface
            .read(0xd0, core::slice::from_mut(&mut value))
            .unwrap();
        assert_eq!(value, 0xa5);
        assert_eq!(interface.memory_page, Some(MEM_PAGE1));
        assert_eq!(interface.device.memory_page, MEM_PAGE1);
    }

    #[test]
    fn pairs_containing_memory_page_are_written_individually_and_tracked() {
        let mut interface = SpiInterface::new(RecordingSpi::default());

        interface
            .write_pairs(&[REG_MEM_PAGE, 0x70], &[MEM_PAGE0, 0xaa])
            .unwrap();

        assert_eq!(interface.memory_page, Some(MEM_PAGE0));
        assert_eq!(interface.device.memory_page, MEM_PAGE0);
        assert!(interface.device.transactions.contains(&std::vec![
            SpiEvent::Write(std::vec![REG_MEM_PAGE & SPI_WRITE_MASK]),
            SpiEvent::Write(std::vec![MEM_PAGE0]),
        ]));
        assert!(interface.device.transactions.contains(&std::vec![
            SpiEvent::Write(std::vec![0x70]),
            SpiEvent::Write(std::vec![0xaa]),
        ]));
    }

    #[test]
    fn failed_soft_reset_still_invalidates_page_cache() {
        let mut interface = SpiInterface::new(RecordingSpi::default());
        let mut value = 0;
        interface
            .read(0xd0, core::slice::from_mut(&mut value))
            .unwrap();
        assert_eq!(interface.memory_page, Some(MEM_PAGE1));

        interface.device.fail_next = true;
        assert_eq!(
            interface.write(REG_SOFT_RESET, &[SOFT_RESET_COMMAND]),
            Err(TestError)
        );
        assert_eq!(interface.memory_page, None);
    }
}
