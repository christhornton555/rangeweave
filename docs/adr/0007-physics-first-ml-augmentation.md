# ADR: Physics-first, uncertainty-aware ML augmentation

- **Status:** Accepted
- **Date:** 2026-08-18

## Context

This decision was established during the reference sensor-stack design and is recorded early so later platform work does not accidentally undo it.

## Decision

Establish timestamping, geometry, calibration and conventional estimation baselines before learned components. ML should provide bounded corrections, confidence, bias/dynamics estimates or map priors with uncertainty rather than becoming an unchecked source of absolute pose.

## Consequences

Future implementations should preserve this boundary unless a new ADR explicitly supersedes it with measured evidence and a migration plan.
