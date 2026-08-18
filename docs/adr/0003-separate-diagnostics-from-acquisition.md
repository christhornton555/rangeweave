# ADR: Separate diagnostics from acquisition firmware

- **Status:** Accepted
- **Date:** 2026-08-18

## Context

This decision was established during the reference sensor-stack design and is recorded early so later platform work does not accidentally undo it.

## Decision

Keep the self-certifying v0.5-derived diagnostic as a hardware/reproducibility tool. Build acquisition firmware separately around a packetizer/queue. This prevents transport/logging changes from quietly mutating the known-good hardware diagnostic.

## Consequences

Future implementations should preserve this boundary unless a new ADR explicitly supersedes it with measured evidence and a migration plan.
