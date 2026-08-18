# ADR: Preserve source clock domains in recorded sensor records

- **Status:** Accepted for protocol v0.1 implementation candidate
- **Date:** 2026-08-18

## Context

The validated reference stack already showed that the LSM6DSOX's real sample timing differs between physical units and must be correlated to the Pico monotonic clock. Earlier planning proposed putting a mapped LSM timestamp directly into ToF records.

A mapped timestamp is derived data: it depends on the clock model fitted at that moment. If only the mapped value is recorded, a later improved clock model cannot reproduce the original observation exactly.

## Decision

Protocol v0.1 preserves source-domain timing:

- IMU samples carry extended native LSM timestamp ticks.
- Magnetometer and ToF observations carry explicit MCU monotonic observation/read times because those sensors do not currently supply a synchronized native timestamp in the reference acquisition path.
- `CLOCK_SYNC` carries the raw bracketed MCU-before / LSM-tick / MCU-after observation.
- Sensor records do not carry a precomputed LSM↔MCU mapped timestamp.

Common-time conversion is performed by host/replay code from the recorded clock-correlation observations.

## Consequences

- Old recordings can be reprocessed with improved clock-fitting algorithms.
- The wire protocol does not embed the reference Pico's current regression model.
- Raw timing uncertainty remains visible instead of being hidden by a single derived timestamp.
- Host implementations on Python and Android must share the same clock-model semantics and numeric fixtures later.

If future hardware supplies a genuinely synchronized hardware timestamp for another sensor, a new record type or compatible field definition may preserve it directly.
