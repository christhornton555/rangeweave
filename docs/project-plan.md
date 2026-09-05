# Rangeweave project plan

This is the living implementation roadmap for Rangeweave. Detailed subsystem contracts live in the linked documents; this file records the order in which evidence should be established so later estimation work does not outrun the sensing/calibration foundations.

## Guiding principles

- preserve raw sensor measurements and real timestamps;
- separate sensor semantics, transport, capture/replay, geometry, calibration and estimator state;
- validate coordinate frames and physical transforms before using them in higher-level estimation;
- keep confidence/failure visible;
- support both a useful **quick-start** path and an optional higher-accuracy **calibrated** path;
- do not promote one reference device's measured constants as universal hardware-model constants without cross-device evidence.

## Completed foundations

The reference Pico 2 W stack has physically validated:

- split-bus LSM6DSOX/LIS3MDL/VL53L5CX acquisition;
- FIFO/timestamp handling and Pico-to-LSM clock correlation;
- Rangeweave v0.1 binary protocol and canonical capture/replay;
- physical VL53L5CX 8x8 zone orientation and nominal sparse projection;
- reference-rig `imu_sensor -> device_body` mapping;
- short-baseline relative gyro integration with independent gravity closure;
- +/-500 deg/s gyro-range configuration/quality checks;
- guided fixed-plane P0-P5 ToF/body rotational boresight workflow;
- held-out P5 boresight validation and promoted per-device artifact.

Optional per-zone ToF intrinsic known-plane calibration exists as a separate workflow and remains non-blocking for the nominal reference geometry path.

## Phase 3 - persistent orientation

### Implemented

- frozen scalar-first Hamilton quaternion and active `local_reference_from_body` conventions;
- deterministic timestamp-driven six-axis gyro+gravity orientation estimator;
- canonical replay inspector with gravity-confidence diagnostics;
- M1-M5 hold-move-hold comparison against the validated relative estimator;
- continuous fixed-wall orientation validator;
- two independent 30 s multi-axis rotation-in-place physical wall captures;
- exploratory ToF timing-offset scan;
- `rangeweave.tof-time-alignment` v1 artifact/resolver with quick-start and calibrated modes.

### Physical evidence so far

All five retained hold-move-hold motions agree with the trusted short-baseline relative estimator to below one degree at the endpoints.

For continuous wall validation at zero ToF timing compensation:

| capture | max excursion | wall-normal RMS | p95 | max | start/end delta |
|---|---:|---:|---:|---:|---:|
| wall 1 | 38.503 deg | 0.855 deg | 1.825 deg | 3.829 deg | 0.644 deg |
| wall 2 | 28.412 deg | 0.747 deg | 1.486 deg | 2.352 deg | 0.150 deg |

Independent scans reproduce a broad timing optimum around `-20` to `-28 ms`; `-24 ms` is the lowest-RMS 2 ms grid point in both reference-rig runs. At `-24 ms`, RMS becomes `0.776 deg` and `0.637 deg` respectively.

This is represented as a **per-build reference-rig calibration**, not as a universal VL53L5CX constant.

### Calibration/product model

Rangeweave keeps two first-class operating modes:

- **Quick-start:** no mandatory physical calibration; use a matching `nominal-fallback` profile where trustworthy, otherwise a conservative fallback with visible `uncalibrated` status.
- **Calibrated:** short guided movements/poses/scans generate per-build versioned artifacts with provenance and configuration fingerprints.

The calibrated workflow is modular. A builder can calibrate timing and boresight while leaving ToF intrinsics nominal, or vice versa.

For ToF timing specifically, runtime precedence is:

```text
explicit override
  -> matching per-build calibrated artifact
  -> matching nominal quick-start profile
  -> 0 ms + visible uncalibrated status
```

A calibrated timing artifact includes an explicit physical `assembly_id` because protocol v0.1 does not expose a stable sensor serial. Material producer/protocol/ToF configuration mismatch invalidates the artifact rather than silently reusing it.

See [`orientation-estimation.md`](orientation-estimation.md) and [`timing-calibration.md`](timing-calibration.md).

### Remaining Phase 3 exit work

1. replay both retained wall captures through the new calibrated timing-artifact resolver;
2. freeze conservative reference physical wall-normal acceptance gates from the worse compensated run;
3. add orientation-aware ToF geometry/viewing;
4. declare the gravity-referenced rotation-only Phase 3 exit gate when the frozen bounds and failure/confidence behaviour are documented.

Magnetometer-aided heading is a later extension, not a prerequisite for the first rotation-only exit gate.

## Later phases

After orientation is stable and replayable:

- physically map/calibrate `mag_sensor -> device_body`;
- implement hard-/soft-iron magnetometer calibration and disturbance gating;
- introduce translation/6DoF pose estimation;
- develop odometry/registration/loop-closure only after their prerequisites have physical evidence;
- port the validated semantics to Android and alternative MCU/transports without redefining the packet/frame meaning.

## Current immediate action

Use the retained reference-rig timing artifact through `inspect_orientation_wall.py` on the two existing wall captures. No new hardware capture is required for this step.
