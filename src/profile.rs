//! Fixed-capacity reassembly of multi-step heater profiles.
//!
//! A `BME68x` exposes only three FIFO-style data fields. Profiles longer than
//! three steps therefore need repeated polling. This module deliberately does
//! not perform I/O or keep time: applications feed each newly read field to a
//! collector, retain control of their polling deadline, and always get a
//! bounded partial result when the deadline expires.

use crate::{Measurement, Measurements, OperationMode};

/// Maximum number of heater steps supported by the `BME68x` register map.
pub const MAX_PROFILE_STEPS: usize = 10;

/// Invalid profile-collector construction.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum ProfileCollectorError {
    /// A profile must contain between one and ten steps.
    InvalidStepCount {
        /// Requested number of steps.
        steps: u8,
    },
    /// Only sequential and parallel modes produce a multi-step profile.
    UnsupportedMode {
        /// Requested operation mode.
        mode: OperationMode,
    },
}

/// Why collection of one logical profile ended.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum ProfileFinishReason {
    /// Every expected step produced a gas-valid conversion.
    Complete,
    /// The application's monotonic polling deadline expired.
    Timeout,
    /// The application deliberately stopped the sensor before completion.
    SensorStopped,
    /// A bus error interrupted polling; the concrete bus error remains owned
    /// by the driver/application error path.
    BusError,
    /// A repeating profile reached its next cycle before this scan completed.
    ProfileRollover,
}

/// One retained heater step and its host-observed offset from scan start.
#[derive(Clone, Copy, Debug, Default, Eq, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct ProfileStep {
    /// Full compensated, raw, status, index, and heater-register data.
    pub measurement: Measurement,
    /// Host monotonic offset at the read that first yielded this step.
    ///
    /// This is a read-time bound, not a synthesized wall-clock timestamp.
    pub observed_offset_us: u32,
}

/// Counters that make loss and ambiguity explicit.
#[derive(Clone, Copy, Debug, Default, Eq, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct ProfileCounters {
    /// Exact fields seen again while they were still in the three-slot FIFO.
    pub duplicates: u16,
    /// Sub-measurement indexes skipped in the forward direction.
    pub overwritten_fields: u16,
    /// Old or out-of-order fields ignored after a newer index was accepted.
    pub out_of_order_fields: u16,
    /// Half-range (`128`) index jumps, whose direction cannot be inferred.
    pub ambiguous_index_jumps: u16,
    /// New fields whose gas index is outside the configured profile.
    pub invalid_gas_indexes: u16,
    /// Parallel-mode dummy/intermediate gas conversions superseded in place.
    pub intermediate_fields: u16,
    /// A lower gas index observed after collection had already advanced.
    pub profile_rollovers: u16,
    /// New fields ignored after rollover or explicit finish froze collection.
    pub fields_after_rollover: u16,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
struct Fingerprint {
    measurement_index: u8,
    gas_index: u8,
}

/// No-allocation collector for one logical 1--10 step heater-profile scan.
///
/// Call [`Self::ingest_batch`] after every hardware poll. In parallel mode,
/// invalid intermediate/dummy conversions are held as a pending value; the
/// gas-valid conversion wins, or the last invalid conversion is retained when
/// the gas index advances or [`Self::finish`] is called. A gas-index rollover
/// freezes the collector so fields from two scans can never be mixed.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct ProfileCollector {
    mode: OperationMode,
    expected_steps: u8,
    steps: [Option<ProfileStep>; MAX_PROFILE_STEPS],
    observed_mask: u16,
    gas_valid_mask: u16,
    heater_stable_mask: u16,
    duplicate_mask: u16,
    observed_field_count: u16,
    observed_field_count_overflowed: bool,
    pending_parallel: Option<ProfileStep>,
    last_measurement_index: Option<u8>,
    last_gas_index: Option<u8>,
    recent: [Option<Fingerprint>; 3],
    recent_cursor: u8,
    frozen: bool,
    finish_reason: Option<ProfileFinishReason>,
    counters: ProfileCounters,
}

impl ProfileCollector {
    /// Construct an empty collector for one sequential or parallel profile.
    ///
    /// # Errors
    ///
    /// Returns an error for a zero/over-length profile or a non-profile mode.
    pub const fn new(
        mode: OperationMode,
        expected_steps: u8,
    ) -> Result<Self, ProfileCollectorError> {
        if expected_steps == 0 || expected_steps as usize > MAX_PROFILE_STEPS {
            return Err(ProfileCollectorError::InvalidStepCount {
                steps: expected_steps,
            });
        }
        if !matches!(mode, OperationMode::Sequential | OperationMode::Parallel) {
            return Err(ProfileCollectorError::UnsupportedMode { mode });
        }

        Ok(Self {
            mode,
            expected_steps,
            steps: [None; MAX_PROFILE_STEPS],
            observed_mask: 0,
            gas_valid_mask: 0,
            heater_stable_mask: 0,
            duplicate_mask: 0,
            observed_field_count: 0,
            observed_field_count_overflowed: false,
            pending_parallel: None,
            last_measurement_index: None,
            last_gas_index: None,
            recent: [None; 3],
            recent_cursor: 0,
            frozen: false,
            finish_reason: None,
            counters: ProfileCounters {
                duplicates: 0,
                overwritten_fields: 0,
                out_of_order_fields: 0,
                ambiguous_index_jumps: 0,
                invalid_gas_indexes: 0,
                intermediate_fields: 0,
                profile_rollovers: 0,
                fields_after_rollover: 0,
            },
        })
    }

    /// Feed all newly available fields from one hardware read, oldest first.
    ///
    /// All fields in the batch receive the same host-observed read offset.
    /// Call [`Self::ingest`] individually if the application has finer timing.
    pub fn ingest_batch(&mut self, fields: &Measurements, observed_offset_us: u32) {
        for measurement in fields {
            self.ingest(*measurement, observed_offset_us);
        }
    }

    /// Feed one newly read field into the collector.
    pub fn ingest(&mut self, measurement: Measurement, observed_offset_us: u32) {
        if !measurement.status.is_new() {
            return;
        }
        if self.observed_field_count == u16::MAX {
            self.observed_field_count_overflowed = true;
        } else {
            self.observed_field_count += 1;
        }
        if self.frozen {
            self.counters.fields_after_rollover =
                self.counters.fields_after_rollover.saturating_add(1);
            return;
        }
        if measurement.gas_index >= self.expected_steps {
            self.counters.invalid_gas_indexes = self.counters.invalid_gas_indexes.saturating_add(1);
            return;
        }

        let fingerprint = Fingerprint {
            measurement_index: measurement.measurement_index,
            gas_index: measurement.gas_index,
        };
        if self
            .recent
            .iter()
            .flatten()
            .any(|seen| *seen == fingerprint)
        {
            self.counters.duplicates = self.counters.duplicates.saturating_add(1);
            self.duplicate_mask |= 1_u16 << measurement.gas_index;
            return;
        }

        if let Some(previous) = self.last_measurement_index {
            match measurement.measurement_index.wrapping_sub(previous) {
                0 => {
                    self.counters.duplicates = self.counters.duplicates.saturating_add(1);
                    self.duplicate_mask |= 1_u16 << measurement.gas_index;
                    return;
                }
                1 => {}
                2..=127 => {
                    let lost = u16::from(
                        measurement
                            .measurement_index
                            .wrapping_sub(previous)
                            .saturating_sub(1),
                    );
                    self.counters.overwritten_fields =
                        self.counters.overwritten_fields.saturating_add(lost);
                }
                128 => {
                    self.counters.ambiguous_index_jumps =
                        self.counters.ambiguous_index_jumps.saturating_add(1);
                    return;
                }
                129..=u8::MAX => {
                    self.counters.out_of_order_fields =
                        self.counters.out_of_order_fields.saturating_add(1);
                    return;
                }
            }
        }

        if self
            .last_gas_index
            .is_some_and(|last| measurement.gas_index < last)
        {
            self.commit_pending_parallel();
            self.counters.profile_rollovers = self.counters.profile_rollovers.saturating_add(1);
            self.frozen = true;
            self.finish_reason = Some(ProfileFinishReason::ProfileRollover);
            self.counters.fields_after_rollover =
                self.counters.fields_after_rollover.saturating_add(1);
            return;
        }

        self.last_measurement_index = Some(measurement.measurement_index);
        self.last_gas_index = Some(measurement.gas_index);
        self.recent[usize::from(self.recent_cursor)] = Some(fingerprint);
        self.recent_cursor = (self.recent_cursor + 1) % 3;

        let step = ProfileStep {
            measurement,
            observed_offset_us,
        };
        match self.mode {
            OperationMode::Sequential => self.commit(step),
            OperationMode::Parallel => self.ingest_parallel(step),
            OperationMode::Sleep | OperationMode::Forced => {}
        }
    }

    /// Finalize a bounded polling attempt and retain a last invalid parallel
    /// conversion when no gas-valid conversion arrived before the deadline.
    pub fn finish(&mut self, reason: ProfileFinishReason) {
        self.commit_pending_parallel();
        self.frozen = true;
        if self.finish_reason.is_none() {
            self.finish_reason = Some(reason);
        }
    }

    /// Configured number of profile steps.
    #[must_use]
    pub const fn expected_steps(&self) -> u8 {
        self.expected_steps
    }

    /// Number of heater steps for which a terminal field was retained.
    #[must_use]
    pub const fn observed_steps(&self) -> u32 {
        self.observed_mask.count_ones()
    }

    /// Fixed-capacity slots indexed by heater/gas index.
    #[must_use]
    pub const fn steps(&self) -> &[Option<ProfileStep>; MAX_PROFILE_STEPS] {
        &self.steps
    }

    /// Return one retained step by gas index.
    #[must_use]
    pub fn step(&self, gas_index: u8) -> Option<&ProfileStep> {
        self.steps
            .get(usize::from(gas_index))
            .and_then(Option::as_ref)
    }

    /// Bitmap of expected heater steps for which a terminal field was kept.
    #[must_use]
    pub const fn observed_mask(&self) -> u16 {
        self.observed_mask
    }

    /// Bitmap of retained steps with the gas-valid bit set.
    #[must_use]
    pub const fn gas_valid_mask(&self) -> u16 {
        self.gas_valid_mask
    }

    /// Bitmap of retained steps with the heater-stable bit set.
    #[must_use]
    pub const fn heater_stable_mask(&self) -> u16 {
        self.heater_stable_mask
    }

    /// Bitmap of valid gas indexes for which a duplicate field was seen.
    #[must_use]
    pub const fn duplicate_mask(&self) -> u16 {
        self.duplicate_mask
    }

    /// Saturating count of every new-status field supplied to the collector.
    ///
    /// This includes duplicates, invalid indexes, out-of-order fields, and
    /// fields ignored after collection froze, making polling behavior fully
    /// observable even when a field is not retained as a profile step.
    #[must_use]
    pub const fn observed_field_count(&self) -> u16 {
        self.observed_field_count
    }

    /// Whether [`Self::observed_field_count`] saturated at [`u16::MAX`].
    #[must_use]
    pub const fn observed_field_count_overflowed(&self) -> bool {
        self.observed_field_count_overflowed
    }

    /// Bitmap of expected steps for which no terminal field was retained.
    #[must_use]
    pub const fn missing_mask(&self) -> u16 {
        self.expected_mask() & !self.observed_mask
    }

    /// Bitmap of observed steps that did not have valid gas data.
    #[must_use]
    pub const fn gas_invalid_mask(&self) -> u16 {
        self.observed_mask & !self.gas_valid_mask
    }

    /// Whether every configured step has a retained terminal field.
    #[must_use]
    pub const fn is_structurally_complete(&self) -> bool {
        self.observed_mask == self.expected_mask()
    }

    /// Whether every configured step has yielded a gas-valid conversion.
    ///
    /// This is the safe condition for explicitly stopping a repeating
    /// parallel scan without confusing dummy fields with completed steps.
    #[must_use]
    pub const fn all_steps_gas_valid(&self) -> bool {
        self.gas_valid_mask == self.expected_mask()
    }

    /// Whether all retained profile steps report heater stability.
    #[must_use]
    pub const fn all_steps_heater_stable(&self) -> bool {
        self.heater_stable_mask == self.expected_mask()
    }

    /// Whether this collector stopped accepting fields after a profile wrap
    /// or an explicit [`Self::finish`] call.
    #[must_use]
    pub const fn is_frozen(&self) -> bool {
        self.frozen
    }

    /// Why this profile stopped accepting measurements, if it has finished.
    #[must_use]
    pub const fn finish_reason(&self) -> Option<ProfileFinishReason> {
        self.finish_reason
    }

    /// Loss, duplicate, rollover, and invalid-index accounting.
    #[must_use]
    pub const fn counters(&self) -> &ProfileCounters {
        &self.counters
    }

    const fn expected_mask(&self) -> u16 {
        (1_u16 << self.expected_steps) - 1
    }

    fn ingest_parallel(&mut self, step: ProfileStep) {
        if self
            .pending_parallel
            .is_some_and(|pending| pending.measurement.gas_index != step.measurement.gas_index)
        {
            self.commit_pending_parallel();
        }

        if step.measurement.status.gas_valid() {
            self.pending_parallel = None;
            self.commit(step);
        } else {
            if self.pending_parallel.is_some() {
                self.counters.intermediate_fields =
                    self.counters.intermediate_fields.saturating_add(1);
            }
            self.pending_parallel = Some(step);
        }
    }

    fn commit_pending_parallel(&mut self) {
        if let Some(pending) = self.pending_parallel.take() {
            self.commit(pending);
        }
    }

    fn commit(&mut self, step: ProfileStep) {
        let index = usize::from(step.measurement.gas_index);
        let bit = 1_u16 << index;
        if self.steps[index].is_some() {
            self.counters.intermediate_fields = self.counters.intermediate_fields.saturating_add(1);
        }

        let should_replace = self.steps[index].map_or(true, |current| {
            !current.measurement.status.gas_valid() || step.measurement.status.gas_valid()
        });
        if !should_replace {
            return;
        }

        self.steps[index] = Some(step);
        self.observed_mask |= bit;
        if step.measurement.status.gas_valid() {
            self.gas_valid_mask |= bit;
        } else {
            self.gas_valid_mask &= !bit;
        }
        if step.measurement.status.heater_stable() {
            self.heater_stable_mask |= bit;
        } else {
            self.heater_stable_mask &= !bit;
        }
    }
}

#[cfg(test)]
mod tests {
    extern crate std;

    use super::*;
    use crate::{MeasurementStatus, RawMeasurement};

    fn measurement(gas_index: u8, measurement_index: u8, status: u8) -> Measurement {
        Measurement {
            status: MeasurementStatus::from_bits(status),
            gas_index,
            measurement_index,
            raw: RawMeasurement {
                temperature_adc: u32::from(measurement_index),
                ..RawMeasurement::default()
            },
            ..Measurement::default()
        }
    }

    fn ingest_three_slot_read(
        collector: &mut ProfileCollector,
        data: &[Measurement],
        observed_offset_us: u32,
    ) {
        assert!(data.len() <= 3);
        for measurement in data {
            collector.ingest(*measurement, observed_offset_us);
        }
    }

    #[test]
    fn ten_step_profile_is_reassembled_across_four_three_slot_reads() {
        let mut collector = ProfileCollector::new(OperationMode::Sequential, 10).unwrap();
        for first in [0_u8, 3, 6, 9] {
            let count = usize::from((10 - first).min(3));
            let mut fields = [Measurement::default(); 3];
            for (offset, field) in fields[..count].iter_mut().enumerate() {
                let step = first + u8::try_from(offset).unwrap();
                *field = measurement(step, 100_u8.wrapping_add(step), 0xb0);
            }
            ingest_three_slot_read(&mut collector, &fields[..count], u32::from(first) * 1_000);
        }

        assert!(collector.is_structurally_complete());
        assert!(collector.all_steps_gas_valid());
        assert!(collector.all_steps_heater_stable());
        assert_eq!(collector.observed_steps(), 10);
        assert_eq!(collector.missing_mask(), 0);
        assert_eq!(collector.step(9).unwrap().measurement.gas_index, 9);
        assert_eq!(*collector.counters(), ProfileCounters::default());
    }

    #[test]
    fn missing_steps_and_fifo_overwrite_are_explicit() {
        let mut collector = ProfileCollector::new(OperationMode::Sequential, 7).unwrap();
        ingest_three_slot_read(
            &mut collector,
            &[
                measurement(0, 10, 0xb0),
                measurement(1, 11, 0xb0),
                measurement(2, 12, 0xb0),
            ],
            0,
        );
        ingest_three_slot_read(
            &mut collector,
            &[measurement(5, 15, 0xb0), measurement(6, 16, 0xb0)],
            10_000,
        );
        collector.finish(ProfileFinishReason::Timeout);

        assert!(!collector.is_structurally_complete());
        assert_eq!(collector.missing_mask(), (1 << 3) | (1 << 4));
        assert_eq!(collector.counters().overwritten_fields, 2);
    }

    #[test]
    fn repeated_fifo_reads_count_duplicates_without_replacing_steps() {
        let fields = [
            measurement(0, 20, 0xb0),
            measurement(1, 21, 0xb0),
            measurement(2, 22, 0xb0),
        ];
        let mut collector = ProfileCollector::new(OperationMode::Sequential, 3).unwrap();
        ingest_three_slot_read(&mut collector, &fields, 1_000);
        ingest_three_slot_read(&mut collector, &fields, 2_000);

        assert!(collector.is_structurally_complete());
        assert_eq!(collector.counters().duplicates, 3);
        assert_eq!(collector.duplicate_mask(), 0x0007);
        assert_eq!(collector.observed_field_count(), 6);
        assert!(!collector.observed_field_count_overflowed());
        assert_eq!(collector.step(0).unwrap().observed_offset_us, 1_000);
    }

    #[test]
    fn repeated_measurement_index_marks_the_current_valid_gas_step() {
        let mut collector = ProfileCollector::new(OperationMode::Sequential, 3).unwrap();
        collector.ingest(measurement(0, 20, 0xb0), 0);
        collector.ingest(measurement(1, 20, 0xb0), 1_000);

        assert_eq!(collector.counters().duplicates, 1);
        assert_eq!(collector.duplicate_mask(), 1 << 1);
        assert!(collector.step(1).is_none());
    }

    #[test]
    fn measurement_index_wrap_is_forward_progress_not_loss() {
        let mut collector = ProfileCollector::new(OperationMode::Sequential, 4).unwrap();
        ingest_three_slot_read(
            &mut collector,
            &[
                measurement(0, 253, 0xb0),
                measurement(1, 254, 0xb0),
                measurement(2, 255, 0xb0),
            ],
            0,
        );
        collector.ingest(measurement(3, 0, 0xb0), 1_000);

        assert!(collector.is_structurally_complete());
        assert_eq!(collector.counters().overwritten_fields, 0);
        assert_eq!(collector.counters().out_of_order_fields, 0);
    }

    #[test]
    fn rollover_freezes_before_fields_from_two_profiles_can_mix() {
        let mut collector = ProfileCollector::new(OperationMode::Sequential, 10).unwrap();
        ingest_three_slot_read(
            &mut collector,
            &[
                measurement(7, 40, 0xb0),
                measurement(8, 41, 0xb0),
                measurement(9, 42, 0xb0),
            ],
            0,
        );
        collector.ingest(measurement(0, 43, 0xb0), 1_000);
        collector.ingest(measurement(1, 44, 0xb0), 2_000);

        assert!(collector.is_frozen());
        assert_eq!(collector.counters().profile_rollovers, 1);
        assert_eq!(collector.counters().fields_after_rollover, 2);
        assert!(collector.step(0).is_none());
        assert_eq!(collector.missing_mask() & 0x007f, 0x007f);
    }

    #[test]
    fn parallel_dummy_fields_do_not_complete_a_step_early() {
        let mut collector = ProfileCollector::new(OperationMode::Parallel, 2).unwrap();
        collector.ingest(measurement(0, 0, 0x80), 100);
        collector.ingest(measurement(0, 1, 0x80), 200);
        assert_eq!(collector.observed_mask(), 0);

        collector.ingest(measurement(0, 2, 0xb0), 300);
        collector.ingest(measurement(1, 3, 0xb0), 400);

        assert!(collector.is_structurally_complete());
        assert!(collector.all_steps_gas_valid());
        assert_eq!(collector.step(0).unwrap().measurement.measurement_index, 2);
        assert_eq!(collector.counters().intermediate_fields, 1);
    }

    #[test]
    fn timeout_retains_last_invalid_parallel_step_with_quality_mask() {
        let mut collector = ProfileCollector::new(OperationMode::Parallel, 2).unwrap();
        collector.ingest(measurement(0, 10, 0x80), 100);
        collector.ingest(measurement(0, 11, 0x80), 200);
        collector.ingest(measurement(1, 12, 0xb0), 300);
        collector.finish(ProfileFinishReason::Timeout);

        assert!(collector.is_structurally_complete());
        assert_eq!(collector.gas_valid_mask(), 1 << 1);
        assert_eq!(collector.gas_invalid_mask(), 1 << 0);
        assert!(!collector.all_steps_gas_valid());
        assert_eq!(collector.step(0).unwrap().measurement.measurement_index, 11);
        assert_eq!(
            collector.finish_reason(),
            Some(ProfileFinishReason::Timeout)
        );
    }

    #[test]
    fn invalid_gas_index_and_old_field_are_accounted_without_panics() {
        let mut collector = ProfileCollector::new(OperationMode::Sequential, 3).unwrap();
        collector.ingest(measurement(0, 50, 0xb0), 0);
        collector.ingest(measurement(9, 51, 0xb0), 0);
        collector.ingest(measurement(1, 49, 0xb0), 0);

        assert_eq!(collector.counters().invalid_gas_indexes, 1);
        assert_eq!(collector.counters().out_of_order_fields, 1);
        assert!(collector.step(1).is_none());
    }

    #[test]
    fn half_range_index_jump_is_rejected_as_ambiguous() {
        let mut collector = ProfileCollector::new(OperationMode::Sequential, 2).unwrap();
        collector.ingest(measurement(0, 10, 0xb0), 0);
        collector.ingest(measurement(1, 138, 0xb0), 1_000);

        assert_eq!(collector.counters().ambiguous_index_jumps, 1);
        assert!(collector.step(1).is_none());
    }

    #[test]
    fn observed_field_counter_includes_invalid_and_frozen_fields_and_saturates() {
        let mut collector = ProfileCollector::new(OperationMode::Sequential, 2).unwrap();
        collector.ingest(measurement(9, 1, 0xb0), 0);
        collector.finish(ProfileFinishReason::SensorStopped);
        collector.ingest(measurement(0, 2, 0xb0), 1);
        collector.ingest(measurement(0, 3, 0x30), 2);

        assert_eq!(collector.observed_field_count(), 2);
        assert_eq!(collector.counters().invalid_gas_indexes, 1);
        assert_eq!(collector.counters().fields_after_rollover, 1);

        collector.observed_field_count = u16::MAX;
        collector.ingest(measurement(0, 4, 0xb0), 3);
        assert_eq!(collector.observed_field_count(), u16::MAX);
        assert!(collector.observed_field_count_overflowed());
    }

    #[test]
    fn constructor_rejects_unsupported_modes_and_lengths() {
        assert_eq!(
            ProfileCollector::new(OperationMode::Parallel, 0),
            Err(ProfileCollectorError::InvalidStepCount { steps: 0 })
        );
        assert_eq!(
            ProfileCollector::new(OperationMode::Parallel, 11),
            Err(ProfileCollectorError::InvalidStepCount { steps: 11 })
        );
        assert_eq!(
            ProfileCollector::new(OperationMode::Forced, 1),
            Err(ProfileCollectorError::UnsupportedMode {
                mode: OperationMode::Forced
            })
        );
    }

    #[cfg(any(feature = "blocking", feature = "async"))]
    #[test]
    fn measurements_batch_forwards_every_new_field() {
        let source = [
            measurement(0, 70, 0xb0),
            measurement(1, 71, 0xb0),
            measurement(2, 72, 0xb0),
        ];
        let fields = Measurements::new(source, 3);
        let mut collector = ProfileCollector::new(OperationMode::Sequential, 3).unwrap();
        collector.ingest_batch(&fields, 12_345);

        assert!(collector.is_structurally_complete());
        assert_eq!(collector.step(2).unwrap().observed_offset_us, 12_345);
    }
}
