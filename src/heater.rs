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
        /// Per-step TPHG repetition multipliers, one per temperature.
        ///
        /// In parallel mode Bosch defines each `GAS_WAITx` byte as a raw
        /// repetition multiplier, not as milliseconds. A value of zero is the
        /// documented special case: skip the shared wait and execute one TPHG
        /// conversion.
        repetition_multipliers: &'a [u8],
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
    /// Raw `IDAC_HEAT0..9` values.
    pub current: [u8; 10],
    /// Raw `RES_HEAT0..9` values.
    pub resistance: [u8; 10],
    /// Raw `GAS_WAIT0..9` values.
    pub gas_wait: [u8; 10],
    /// Raw shared-heater-duration register.
    pub shared_duration: u8,
}

/// Complete raw gas-heater control and profile readback.
#[derive(Clone, Copy, Debug, Default, Eq, PartialEq)]
pub struct HeaterConfigurationReadback {
    /// All current-DAC, heater-resistance, gas-wait, and shared-duration
    /// profile registers.
    pub registers: HeaterRegisters,
    /// Exact `CTRL_GAS_0` register value.
    pub control_gas_0: u8,
    /// Exact `CTRL_GAS_1` register value.
    pub control_gas_1: u8,
}

impl HeaterConfigurationReadback {
    /// Whether the heater-control bit enables the heater.
    #[must_use]
    pub const fn heater_enabled(self) -> bool {
        self.control_gas_0 & 0x08 == 0
    }

    /// Raw two-bit `run_gas` field.
    #[must_use]
    pub const fn run_gas(self) -> u8 {
        (self.control_gas_1 >> 4) & 0x03
    }

    /// Raw four-bit `nb_conv` profile-length field.
    ///
    /// Bosch programs zero for a forced-mode one-step conversion and the
    /// configured step count for sequential/parallel profiles.
    #[must_use]
    pub const fn profile_length(self) -> u8 {
        self.control_gas_1 & 0x0f
    }
}

/// Environmental, operating-mode, and gas-heater register readback.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct SensorConfigurationReadback {
    /// Current sensor operating mode.
    pub operation_mode: crate::OperationMode,
    /// Current temperature, pressure, humidity, filter, and standby settings.
    pub environmental: crate::Configuration,
    /// Current gas-heater control and profile registers.
    pub heater: HeaterConfigurationReadback,
}
