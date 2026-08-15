//! Gas-heater configuration types.

/// Gas-heater settings for one operating mode.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum HeaterConfiguration<'a> {
    /// One heater temperature and duration for forced mode.
    Forced {
        /// Whether gas measurement and the heater are enabled.
        enabled: bool,
        /// Target heater temperature in degrees Celsius (capped to 400 °C).
        temperature_celsius: u16,
        /// Heater duration in milliseconds.
        duration_ms: u16,
    },
    /// A heater profile for sequential mode.
    Sequential {
        /// Whether gas measurement and the heater are enabled.
        enabled: bool,
        /// Target temperatures in degrees Celsius, one to ten entries.
        temperatures_celsius: &'a [u16],
        /// Heater durations in milliseconds, one per temperature.
        durations_ms: &'a [u16],
    },
    /// A heater profile for parallel mode.
    Parallel {
        /// Whether gas measurement and the heater are enabled.
        enabled: bool,
        /// Target temperatures in degrees Celsius, one to ten entries.
        temperatures_celsius: &'a [u16],
        /// Per-step gas-wait register durations, one per temperature.
        durations_ms: &'a [u16],
        /// Shared heater duration in milliseconds; must be non-zero.
        shared_duration_ms: u16,
    },
}

impl HeaterConfiguration<'_> {
    /// Operating mode required by this heater configuration.
    #[must_use]
    pub const fn mode(&self) -> crate::OperationMode {
        match self {
            Self::Forced { .. } => crate::OperationMode::Forced,
            Self::Sequential { .. } => crate::OperationMode::Sequential,
            Self::Parallel { .. } => crate::OperationMode::Parallel,
        }
    }

    #[cfg(any(feature = "blocking", feature = "async"))]
    pub(crate) const fn enabled(&self) -> bool {
        match self {
            Self::Forced { enabled, .. }
            | Self::Sequential { enabled, .. }
            | Self::Parallel { enabled, .. } => *enabled,
        }
    }
}

/// Raw heater profile registers read back from the sensor.
#[derive(Clone, Copy, Debug, Default, Eq, PartialEq)]
pub struct HeaterRegisters {
    /// Raw `RES_HEAT0..9` values.
    pub resistance: [u8; 10],
    /// Raw `GAS_WAIT0..9` values.
    pub gas_wait: [u8; 10],
}
