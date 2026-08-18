# ADR: RGB-free and no installed-beacon core requirements

- **Status:** Accepted
- **Date:** 2026-08-18

## Context

This decision was established during the reference sensor-stack design and is recorded early so later platform work does not accidentally undo it.

## Decision

The core mapping/tracking path should not require RGB imagery or installed external beacons. Optional future modalities may be explored, but the reference architecture remains useful without them. “No RGB” is not a universal privacy guarantee and should not be marketed as one.

## Consequences

Future implementations should preserve this boundary unless a new ADR explicitly supersedes it with measured evidence and a migration plan.
