# Changelog

All notable project changes should be documented here once development moves into Git.

## Unreleased

### Added

- Initial public repository scaffold.
- Curated Pico 2 W diagnostics: split-bus scan, standalone IMU, standalone VL53L5CX and v0.5-derived reproducibility self-test.
- Public Markdown build guide and living project plan.
- Reference v0.5 validation log/summary.
- Architecture/portability/calibration/protocol notes and initial ADRs.
- Experimental Rangeweave wire protocol v0.1 specification: COBS framing, CRC16, sequence numbers and `IMU_BATCH`, `MAG`, `TOF_GRID`, `CLOCK_SYNC`, `STATUS` and `STREAM_INFO` records.
- Ephemeral per-session `STREAM_INFO` metadata/configuration mechanism without a required globally stable hardware identifier.
- Dependency-light Python v0.1 reference decoder/encoder framing implementation.
- Language-neutral golden byte fixtures plus corruption/resynchronisation tests.
- ADRs for v0.1 stream framing and preservation of source clock domains.
- Experimental Pico 2 W acquisition producer split into sensor, timing, protocol and USB-transport modules.
- Acquisition-side golden-frame conformance tests and LSM timestamp wrap/backlog regression tests.
- `probe_serial.py` for the first live USB framing/sequence/health smoke test.

### Current baseline

- Sensor-stack validation phase complete on the first reference unit.
- Protocol v0.1 is merged as an implementation candidate.
- Pico acquisition firmware is entering physical hardware validation; it is not yet a validated replacement for the v0.5 diagnostic baseline.
