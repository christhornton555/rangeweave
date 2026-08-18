# ADR: Transport-independent record model

- **Status:** Accepted
- **Date:** 2026-08-18

## Context

This decision was established during the reference sensor-stack design and is recorded early so later platform work does not accidentally undo it.

## Decision

The packet/record model is the portability boundary. USB is the first transport, not the architecture. Firmware packetization must be transport-agnostic so later Android USB, BLE/Wi-Fi, UART or local-storage implementations preserve the same record semantics. Host parsing/mapping must not depend on Pico GPIOs or MicroPython objects.

## Consequences

Future implementations should preserve this boundary unless a new ADR explicitly supersedes it with measured evidence and a migration plan.
