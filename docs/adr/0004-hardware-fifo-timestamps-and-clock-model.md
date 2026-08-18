# ADR: Hardware FIFO/timestamps and runtime clock model

- **Status:** Accepted
- **Date:** 2026-08-18

## Context

This decision was established during the reference sensor-stack design and is recorded early so later platform work does not accidentally undo it.

## Decision

The LSM6DSOX hardware FIFO protects inertial acquisition while MicroPython is blocked by long ToF frame reads. FIFO timestamp records define the sensor timeline. `INTERNAL_FREQ_FINE` provides a per-unit factory estimate, while repeated direct correlations fit LSM time to MCU monotonic time. No downstream algorithm may hard-code the reference unit's ~101-Hz real ODR.

## Consequences

Future implementations should preserve this boundary unless a new ADR explicitly supersedes it with measured evidence and a migration plan.
