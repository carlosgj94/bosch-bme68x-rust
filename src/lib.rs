//! Platform-independent Bosch BME680/BME688 sensor driver.
//!
//! This crate is `no_std` and communicates through `embedded-hal` 1.0 traits.

#![no_std]
#![cfg_attr(docsrs, feature(doc_cfg))]

pub mod compensation;
mod data;
mod error;
#[cfg(feature = "float")]
#[cfg_attr(docsrs, doc(cfg(feature = "float")))]
pub mod float;
mod heater;
#[cfg(any(feature = "blocking", feature = "async"))]
mod registers;
mod types;

#[cfg(feature = "blocking")]
pub mod blocking;
#[cfg(feature = "blocking")]
pub mod interface;

#[cfg(feature = "async")]
#[cfg_attr(docsrs, doc(cfg(feature = "async")))]
pub mod asynch;

pub use data::{Measurement, MeasurementStatus, Measurements};
pub use error::{ConfigError, Error, SelfTestFailure};
pub use heater::{HeaterConfiguration, HeaterRegisters};
pub use types::*;
