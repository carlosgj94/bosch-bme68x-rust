// Register definitions are derived from Bosch Sensortec's
// BME68x SensorAPI v4.4.8.
// Copyright (c) 2023 Bosch Sensortec GmbH. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause

//! `BME68x` register map and protocol constants.

pub(crate) const CHIP_ID: u8 = 0x61;
pub(crate) const SOFT_RESET_COMMAND: u8 = 0xb6;

pub(crate) const REG_COEFF3: u8 = 0x00;
pub(crate) const REG_FIELD0: u8 = 0x1d;
pub(crate) const REG_IDAC_HEAT0: u8 = 0x50;
pub(crate) const REG_RES_HEAT0: u8 = 0x5a;
pub(crate) const REG_GAS_WAIT0: u8 = 0x64;
pub(crate) const REG_SHARED_HEATER_DURATION: u8 = 0x6e;
pub(crate) const REG_CTRL_GAS_0: u8 = 0x70;
pub(crate) const REG_CTRL_GAS_1: u8 = 0x71;
pub(crate) const REG_CTRL_HUM: u8 = 0x72;
pub(crate) const REG_CTRL_MEAS: u8 = 0x74;
pub(crate) const REG_CONFIG: u8 = 0x75;
pub(crate) const REG_MEM_PAGE: u8 = 0xf3;
pub(crate) const REG_COEFF1: u8 = 0x8a;
pub(crate) const REG_CHIP_ID: u8 = 0xd0;
pub(crate) const REG_SOFT_RESET: u8 = 0xe0;
pub(crate) const REG_COEFF2: u8 = 0xe1;
pub(crate) const REG_VARIANT_ID: u8 = 0xf0;

pub(crate) const LEN_COEFF1: usize = 23;
pub(crate) const LEN_COEFF2: usize = 14;
pub(crate) const LEN_COEFF3: usize = 5;
pub(crate) const LEN_FIELD: usize = 17;
pub(crate) const FIELD_COUNT: usize = 3;
pub(crate) const MAX_PROFILE_LEN: usize = 10;
pub(crate) const MAX_REGISTER_WRITES: usize = 10;

pub(crate) const MODE_MASK: u8 = 0x03;
pub(crate) const NEW_DATA_MASK: u8 = 0x80;
pub(crate) const GAS_INDEX_MASK: u8 = 0x0f;
pub(crate) const GAS_RANGE_MASK: u8 = 0x0f;
pub(crate) const GAS_VALID_MASK: u8 = 0x20;
pub(crate) const HEATER_STABLE_MASK: u8 = 0x10;

pub(crate) const RESET_DELAY_US: u32 = 10_000;
pub(crate) const POLL_DELAY_US: u32 = 10_000;
