# Changelog

All notable project changes should be documented here once development moves into Git.

## Unreleased

### Added

- Initial public repository scaffold.
- Curated Pico 2 W diagnostics: split-bus scan, standalone IMU, standalone VL53L5CX and v0.5-derived reproducibility self-test.
- Public Markdown build guide and living project plan.
- Reference v0.5 validation log/summary.
- Architecture/portability/calibration/protocol notes and initial ADRs.

### Current baseline

- Sensor-stack validation phase complete on the first reference unit.
- Next phase: transport-independent packet format + Pico acquisition firmware + Python capture/replay, followed early by Kotlin/Android conformance and ESP32-class portability testing.
