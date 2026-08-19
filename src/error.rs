//! Error types returned by the driver.

/// A configuration supplied to the sensor was not valid.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum ConfigError {
    /// A heater profile must contain between one and ten steps.
    InvalidProfileLength {
        /// Supplied number of steps.
        length: usize,
    },
    /// The temperature and duration profile slices have different lengths.
    ProfileLengthMismatch {
        /// Number of temperature entries.
        temperatures: usize,
        /// Number of duration entries.
        durations: usize,
    },
    /// The parallel temperature and repetition-multiplier slices differ.
    ParallelProfileLengthMismatch {
        /// Number of temperature entries.
        temperatures: usize,
        /// Number of repetition-multiplier entries.
        repetition_multipliers: usize,
    },
    /// Parallel mode requires a non-zero shared heater duration.
    MissingSharedHeaterDuration,
    /// Data readout is not defined for sleep mode.
    UnsupportedDataMode,
    /// Raw register writes contain an invalid number of address/data pairs.
    InvalidRegisterWriteLength {
        /// Supplied number of pairs.
        length: usize,
    },
    /// Raw register address and value slices have different lengths.
    RegisterWriteLengthMismatch {
        /// Number of register addresses.
        registers: usize,
        /// Number of values.
        values: usize,
    },
}

/// The reason a live sensor self-test failed.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum SelfTestFailure {
    /// A measurement did not satisfy the Bosch self-test limits.
    MeasurementOutOfRange,
    /// A gas measurement was not marked valid.
    InvalidGasMeasurement,
    /// The heater current DAC reported an invalid value.
    InvalidHeaterCurrent,
    /// The alternating heater-temperature response was too small.
    GasResponseTooSmall,
}

/// Driver error preserving the concrete bus error.
#[derive(Debug, Eq, PartialEq)]
pub enum Error<E> {
    /// Communication with the sensor failed.
    Bus(E),
    /// A device responded, but its chip identifier was not `0x61`.
    UnexpectedChipId {
        /// Identifier read from the chip-id register.
        found: u8,
    },
    /// A reserved value was observed in a typed sensor register field.
    InvalidRegisterValue {
        /// Logical register containing the value.
        register: u8,
        /// Unsupported field value after masking and shifting.
        value: u8,
    },
    /// A configuration could not be represented safely.
    InvalidConfiguration(ConfigError),
    /// The live Bosch-style self-test failed.
    SelfTestFailed(SelfTestFailure),
    /// The sensor did not enter the requested state within the polling limit.
    Timeout,
}

impl<E> From<E> for Error<E> {
    fn from(value: E) -> Self {
        Self::Bus(value)
    }
}
