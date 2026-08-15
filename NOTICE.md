# Attribution and reference implementation

The compensation formulas, register behavior, heater configuration, and
self-test behavior in this crate are derived from Bosch Sensortec's
`BME68x_SensorAPI` v4.4.8, commit
`80ea120a8b8ac987d7d79eb68a9ed796736be845`, distributed under BSD-3-Clause.

The Bosch source is used as a host-side differential-test oracle. It is not
linked into firmware built with this crate.

Pinned source SHA-256 values:

- `bme68x.c`: `bad525ea57fe57a5d7dda19d287781bcbab50fdbf2dedea737c16b206cbe0157`
- `bme68x.h`: `09ed6babf92955ced360790958e2e2f03efb85bc4f0a8a864a4151a905eaf0ca`
- `bme68x_defs.h`: `5f59c0bafbcc416c091a1a1cc40bb7362f0b48ea02ac7fa5197813f491de01aa`
