# Validation evidence

This directory contains intentionally published reference runs and summaries used to distinguish **measured behaviour** from roadmap claims.

## Current evidence

- [`reference-unit-v0.5.md`](reference-unit-v0.5.md) - validated reference sensor-stack summary.
- [`pico2w-reference-run-v0.5.txt`](pico2w-reference-run-v0.5.txt) - full console output from the reference self-test.
- [`boresight-reference-rig-2026-09.md`](boresight-reference-rig-2026-09.md) - physical reference-rig evidence for IMU/body axis mapping, relative rotation, the +/-250 deg/s failure, the switch to +/-500 deg/s, fixture-settling lessons, stationary-ToF quality gates, the successful P0-P5 fixed-wall sequence, held-out P5 prediction and stable six-pose boresight refit.
- [`orientation-reference-rig-2026-09.md`](orientation-reference-rig-2026-09.md) - continuous rotation-in-place wall evidence, calibrated ToF timing alignment, two independent resolver replays and the frozen empirical Phase 3 reference wall gate.
- [`provenance.md`](provenance.md) - hashes linking the published diagnostic baseline to the original validated candidate.
- [`runtime-environment.md`](runtime-environment.md) - validated Pico runtime fingerprint and UF2 provenance note.

## Reproduction reports

When testing another physical unit, record at minimum:

- unit/build ID and date;
- exact MicroPython `os.uname()` / `sys.implementation` output;
- project commit/tag;
- bus scan and WHO_AM_I values;
- complete `REPRODUCIBILITY SELF TEST` block;
- observed `FREQ_FINE`, measured clock tick/ODR and fit RMS;
- maximum FIFO backlog;
- magnetometer successes/attempts/retries/drops;
- ToF fps/errors/max read time;
- final `SYSTEM READY` result;
- relevant wiring/mechanical changes.

Do not reject a unit because its oscillator-derived ODR differs from the reference number if the structural/timing invariants pass.

For calibration/boresight/orientation reproduction also retain:

- raw capture directories (`metadata.json`, `packets.bin`, `notes.txt`);
- capture SHA-256 values;
- actual `STREAM_INFO` sensor/register configuration;
- physical `imu_sensor -> device_body` mounting/mapping;
- target/setup description;
- all quality-gate outputs, including gyro range utilisation, gravity closure, ToF plane residuals and half-capture drift;
- P0-P5 pose/motion sequence details where boresight is being calibrated;
- the P0-P4 training fit, held-out P5 prediction error and final P0-P5 fit stability;
- the generated `rangeweave.tof-body-rotation` artifact and its geometry-profile provenance;
- the timing-resolution role/profile/artifact and assembly identity used for continuous wall validation;
- usable-frame share, motion excursion, wall-normal RMS/p95/max and start/end delta.

Reference-rig calibration parameters and exact residual distributions must not be assumed to transfer unchanged to a differently assembled unit. The published Phase 3 wall gate is an empirical project validation contract and may be revised only with additional evidence.
