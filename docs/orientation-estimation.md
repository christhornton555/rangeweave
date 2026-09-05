# Orientation estimation plan

**Status:** Phase 3 is the active estimation layer. The acquisition/timing path, `imu_sensor -> device_body` mapping, ToF geometry convention and reference-rig `tof_optical -> device_body` rotational boresight are already validated and provide the inputs this layer needs.

The project-owned attitude conventions are frozen in [`attitude-conventions.md`](attitude-conventions.md). The deterministic Python gyro+gravity orientation core, replay inspector and continuous fixed-wall validator are implemented and now have physical reference-rig evidence.

## Goal

Estimate the sensing head's orientation over time from the timestamped IMU stream, with explicit conventions, confidence and failure modes, so ToF rays/points can be rotated into a stable local reference frame before translation/odometry is introduced.

The first success criterion is deliberately rotation-only: when the sensing head rotates in place while observing static geometry, applying the estimated body orientation plus the calibrated ToF/body boresight should keep that geometry's orientation stable within measured bounds.

## Inputs already available

- LSM6DSOX FIFO accel/gyro samples with hardware timestamps;
- measured LSM clock correlation rather than assumed nominal ODR;
- physically validated reference-rig mapping:
  - `body X = -imu X`
  - `body Y = -imu Z`
  - `body Z = -imu Y`;
- short-baseline gyro integration with independent endpoint gravity closure;
- current +/-500 deg/s gyro configuration and range-utilisation checks;
- versioned `rangeweave.tof-body-rotation` calibration artifact for the reference sensing head;
- versioned `rangeweave.tof-time-alignment` timing/profile resolver with quick-start and calibrated modes;
- ToF plane/ray geometry in the frozen `tof_optical` frame;
- canonical capture/replay so estimator development can be performed offline before live use.

## Scope of Phase 3

### 3A. Freeze attitude conventions — IMPLEMENTED

The reference path now defines and golden-tests:

- quaternion component order: scalar-first `(w, x, y, z)`;
- Hamilton multiplication;
- active `R_reference_from_body` transform direction;
- composition order consistent with existing Rangeweave matrices;
- body-frame angular velocity and right-multiplied gyro increments;
- accelerometer specific-force/up direction versus physical gravity down;
- gravity-referenced `local_reference` construction;
- local yaw-zero semantics and the explicit statement that yaw is unobservable from accelerometer + gyro alone.

These conventions are project-owned and must be converted explicitly at Python/Android/graphics/robotics boundaries. See [`attitude-conventions.md`](attitude-conventions.md).

### 3B. Six-axis orientation baseline — IMPLEMENTED CANDIDATE

`host/python/rangeweave_orientation.py` provides a timestamp-driven gyro + accelerometer baseline and `host/python/inspect_orientation.py` replays it on canonical captures.

Current properties:

- integrates mapped body-frame gyro using LSM hardware-timestamp correlation;
- initializes gravity-referenced pitch/roll and a local, non-global yaw zero from an explicit stationary initial interval;
- estimates a **fixed initial gyro bias only from that explicit stationary interval**, avoiding silent online reinterpretation of slow real motion as bias;
- uses accelerometer only as a gravity/specific-force observation when magnitude is credible;
- exposes gravity-correction weight and innovation diagnostics;
- remains deterministic and standard-library-only under replay;
- keeps estimator state separate from permanent calibration artifacts;
- does not use the magnetometer.

The proportional gravity gain and confidence behaviour remain engineering defaults under physical validation rather than universal tuning constants.

### 3C. Rotation-in-place validation — PHYSICAL EVIDENCE COMPLETE; BOUNDS PENDING

The first regression level is complete: all five retained clean hold-move-hold boresight motions have been replayed through the persistent estimator and compared with the already validated short-baseline relative-rotation estimator.

| motion | trusted relative angle | gravity closure | persistent-estimator delta |
|---|---:|---:|---:|
| M1 | 17.822 deg | 0.647 deg | 0.656 deg |
| M2 | 28.080 deg | 0.390 deg | 0.777 deg |
| M3 | 45.426 deg | 0.322 deg | 0.339 deg |
| M4 | 23.322 deg | 0.745 deg | 0.736 deg |
| M5 | 24.295 deg | 0.510 deg | 0.859 deg |

All five captures had clean decoder/sequence/health status and acceptable +/-500 deg/s range utilisation. Across varied real motions the persistent estimator remained in sub-degree start/end agreement with the trusted relative estimator.

The continuous wall-normal validator then tested the full chain:

```text
ToF normal -> R_body_from_tof -> estimated local/reference orientation
```

on two independent 30 s multi-axis rotation-in-place captures against a fixed wall.

At zero explicit ToF timing compensation:

| capture | max excursion | residual RMS | p95 | max | start/end delta |
|---|---:|---:|---:|---:|---:|
| wall 1 | 38.503 deg | 0.855 deg | 1.825 deg | 3.829 deg | 0.644 deg |
| wall 2 | 28.412 deg | 0.747 deg | 1.486 deg | 2.352 deg | 0.150 deg |

Both independently reproduce sub-degree RMS and sub-degree start/end stability.

#### ToF timing evidence

Protocol v0.1 records `mcu_ready_us` when software observes VL53L5CX data-ready, not the exact sensor-internal ranging instant. Independent exploratory scans on both wall captures reproduce the same broad optimum around `-20` to `-28 ms`, with `-24 ms` the lowest-RMS 2 ms grid point in both captures.

At `-24 ms`:

| capture | residual RMS | p95 | max | start/end delta |
|---|---:|---:|---:|---:|
| wall 1 | 0.776 deg | 1.664 deg | 3.530 deg | 0.703 deg |
| wall 2 | 0.637 deg | 1.222 deg | 1.859 deg | 0.151 deg |

The timing compensation improves the moving-frame residual distribution while stationary start/end consistency remains essentially unchanged.

`-24 ms` is **not** a universal VL53L5CX constant. The effective observation-time offset can depend on the assembled sensing head, producer firmware/driver path, operating mode/timing settings and timestamp semantics. `host/python/rangeweave_tof_timing.py` therefore resolves timing through two first-class modes:

- **quick-start**: matched `nominal-fallback` profile when available, otherwise conservative `0 ms` with visible `uncalibrated` status;
- **calibrated**: a matching per-build `rangeweave.tof-time-alignment` artifact with an explicit physical `assembly_id` and configuration fingerprint.

The retained reference-rig artifact is [`../calibration/tof-time-alignment-reference-rig-20260905.json`](../calibration/tof-time-alignment-reference-rig-20260905.json). See [`timing-calibration.md`](timing-calibration.md).

The remaining 3C task is to replay the two reference wall captures through the artifact resolver and freeze conservative acceptance gates from the worse compensated run rather than the best case.

### 3D. Magnetometer integration, later

Do not use LIS3MDL heading as an unquestioned yaw reference yet.

Before enabling magnetic heading:

- physically establish `mag_sensor -> device_body` mapping;
- implement hard-iron and soft-iron calibration;
- characterize magnetic disturbances on the real assembly/environment;
- add confidence/disturbance gating so bad magnetic data can be rejected;
- validate heading improvement against repeatable rotations/return tests.

The six-axis estimator must remain useful when magnetic heading is unavailable or untrusted.

### 3E. Orientation-aware ToF geometry

Once the attitude layer passes rotation-in-place validation:

- rotate calibrated ToF rays/points from `tof_optical` through `device_body` into the local/reference frame;
- add an orientation-aware point-cloud/plane viewer;
- verify static geometry remains angularly stable while the sensing head rotates;
- only then proceed to translation and 6DoF pose estimation.

## Non-goals for Phase 3

Phase 3 does **not** yet claim:

- position/translation estimation;
- rigid sensor-origin translation calibration unless required by a specific test;
- freehand odometry or SLAM;
- loop closure;
- globally referenced yaw without a validated heading source;
- Android parity.

## Exit gate

Phase 3 is complete when a replayable orientation estimator with frozen frame/quaternion conventions can rotate real ToF observations into a local reference frame such that rotation-in-place of the physical sensing head keeps static geometry stable within quantified bounds, with estimator confidence/failure visible.

Magnetometer-aided heading may be a later Phase 3 extension; it is not required for the first gravity-referenced rotation-only exit gate.

## Immediate implementation order

1. ~~write/freeze the attitude/quaternion convention and golden composition tests~~ — done;
2. ~~implement the timestamp-driven six-axis orientation core and replay inspector~~ — done;
3. ~~replay existing boresight motion captures as regression evidence~~ — M1-M5 complete, all sub-degree start/end delta;
4. ~~implement continuous wall-normal validation tooling~~ — done;
5. ~~make independent continuous multi-axis rotation-in-place wall captures against a static wall~~ — two complete;
6. ~~measure and represent ToF timing alignment without hard-coding one device's value~~ — resolver + reference artifact implemented;
7. **replay both wall captures through calibrated artifact resolution and freeze conservative empirical acceptance bounds**;
8. add orientation-aware ToF geometry/viewing;
9. then begin magnetometer mapping/calibration and later full 6DoF pose work.
