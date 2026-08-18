# ADR: Python reference + Kotlin golden-fixture conformance

- **Status:** Accepted
- **Date:** 2026-08-18

## Context

This decision was established during the reference sensor-stack design and is recorded early so later platform work does not accidentally undo it.

## Decision

Python/PC is the initial algorithm/reference environment, but Android is a first-class host. The packet specification and calibration/coordinate conventions must be language-neutral. Python and Kotlin decoders must consume the same checked-in golden byte fixtures before protocol complexity grows.

## Consequences

Future implementations should preserve this boundary unless a new ADR explicitly supersedes it with measured evidence and a migration plan.
