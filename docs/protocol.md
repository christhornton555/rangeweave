# Rangeweave packet protocol

**Status: EXPERIMENTAL / protocol v0.1 implementation candidate.**

The first byte-level protocol now exists. The normative specification is [`../protocol/spec-v0.1.md`](../protocol/spec-v0.1.md).

## v0.1 decisions

- COBS-framed binary stream with `0x00` delimiters.
- CRC-16/CCITT-FALSE on every decoded frame.
- Global uint32 packet sequence for loss detection.
- Little-endian fixed-width integer fields.
- Maximum decoded frame size: 1024 bytes.
- Initial records:
  - `IMU_BATCH`
  - `MAG`
  - `TOF_GRID`
  - `CLOCK_SYNC`
  - `STATUS`
- IMU samples may be batched (up to 16) to avoid making USB framing overhead part of the long-term architecture.
- Raw sensor/source clock domains are preserved. Host software derives common time from `CLOCK_SYNC`; sensor packets do not contain precomputed clock-model output.
- ToF range, reflectance and target-status arrays are independently presence-masked. Protocol v0.1 supports one target per zone and grids up to 64 zones.
- Live and replay data use the same framed bytes.

## Why the ToF packet changed from the early plan

The earlier roadmap proposed transmitting both the observed MCU timestamp and an estimated/mapped LSM timestamp with each ToF frame.

Protocol v0.1 does **not** transmit the mapped timestamp. The mapped value is derived from the current clock fit, so recording it as though it were raw evidence would make later clock-model improvements harder to apply consistently.

Instead:

```text
IMU              -> native extended LSM ticks
MAG / ToF         -> MCU monotonic observation/read times
CLOCK_SYNC        -> raw MCU-before / LSM-tick / MCU-after correlation
host / replay     -> fitted common timeline
```

See [ADR-0009](adr/0009-preserve-source-clock-domains.md).

## Cross-platform gate

The standard-library Python reference decoder is in [`../host/python/rangeweave_protocol.py`](../host/python/rangeweave_protocol.py).

Canonical byte fixtures are in [`../protocol/test-vectors/v0.1.json`](../protocol/test-vectors/v0.1.json).

The Kotlin/Android decoder must consume the same fixtures; no Android-specific reinterpretation of field widths, byte order, timing or zone array order is allowed.

## Still deliberately deferred

Protocol v0.1 does not yet define device/configuration metadata, command/control, compression, wall-clock timestamps, encryption/authentication or multiple ToF targets per zone.

The next protocol subtask, before acquisition firmware is declared complete, is a small metadata/config record that describes the sensor profile without silently introducing a globally stable tracking identifier.
