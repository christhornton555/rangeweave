# Changelog

All notable project changes should be documented here once development moves into Git.

## Unreleased

### Added

- Initial public repository scaffold.
- Curated Pico 2 W diagnostics: split-bus scan, standalone IMU, standalone VL53L5CX and v0.5-derived reproducibility self-test.
- Public Markdown build guide and living project plan.
- Reference v0.5 validation log/summary.
- Architecture/portability/calibration/protocol notes and initial ADRs.
- Experimental Rangeweave wire protocol v0.1 specification: COBS framing, CRC16, sequence numbers and initial sensor/health records.
- Dependency-light Python v0.1 reference decoder/encoder framing implementation.
- Language-neutral golden byte fixtures plus corruption/resynchronisation tests.
- ADRs for v0.1 stream framing and preservation of source clock domains.

### Current baseline

- Sensor-stack validation phase complete on the first reference unit.
- Protocol/acquisition phase is now in progress; protocol v0.1 is an implementation candidate pending Pico acquisition testing and Kotlin/Android conformance.
