# ADR: Preserve raw data and explicit timestamps

- **Status:** Accepted
- **Date:** 2026-08-18

## Context

This decision was established during the reference sensor-stack design and is recorded early so later platform work does not accidentally undo it.

## Decision

Retain raw measurements and explicit sensor/MCU timing metadata wherever practical. Do not infer time from record index or nominal sample rate, and avoid irreversible MCU filtering that would prevent future calibration/estimator improvements or replay.

## Consequences

Future implementations should preserve this boundary unless a new ADR explicitly supersedes it with measured evidence and a migration plan.
