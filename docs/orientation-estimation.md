# Orientation estimation plan

**Status:** Phase 3 is the active estimation layer. The acquisition/timing path, `imu_sensor -> device_body` mapping, ToF geometry convention and reference-rig `tof_optical -> device_body` rotational boresight are already validated and provide the inputs this layer needs.

The project-owned attitude conventions are frozen in [`attitude-conventions.md`](attitude-conventions.md). The deterministic Python gyro+gravity orientation core has now passed the reference-rig hold-move-hold regressions and two independent continuous rotation-in-place wall validations using calibrated ToF/body boresight and calibrated ToF/IMU timing alignment.

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
- versioned `rangeweave.tof-time-alignment` resolver supporting quick-start and calibrated modes;
- ToF plane/ray geometry in the frozen `tof_optical` frame;
- canonical capture/replay so estimator development can be performed offline before live use.

## Scope of Phase 3

### 3A. Freeze attitude conventions — IMPLEMENTED

The reference path defines and golden-tests:

- quaternion component order: scalar-first `(w, x, y, z)`;
- Hamilton multiplication;
- active `R_reference_from_body` transform direction;
- composition order consistent with existing Rangeweave matrices;
- body-frame angular velocity and right-multiplied gyro increments;
- accelerometer specific-force/up direction versus physical gravity down;
- gravity-referenced `local_reference` construction;
- local yaw-zero semantics and the explicit statement that yaw is unobservable from accelerometer + gyro alone.

These conventions are project-owned and must be converted explicitly at Python/Android/graphics/robotics boundaries. See [`attitude-conventions.md`](attitude-conventions.md).

### 3B. Six-axis orientation baseline — IMPLEMENTED / PHYSICALLY VALIDATED ON REFERENCE RIG

`host/python/rangeweave_orientation.py` provides the timestamp-driven gyro + accelerometer baseline and `host/python/inspect_orientation.py` replays it on canonical captures.

Current properties:

- integrates mapped body-frame gyro using LSM hardware-timestamp correlation;
- initializes gravity-referenced pitch/roll and a local, non-global yaw zero from an explicit stationary initial interval;
- estimates a **fixed initial gyro bias only from that explicit stationary interval**, avoiding silent online reinterpretation of slow real motion as bias;
- uses accelerometer only as a gravity/specific-force observation when magnitude is credible;
- exposes gravity-correction weight and innovation diagnostics;
- remains deterministic and standard-library-only under replay;
- keeps estimator state separate from permanent calibration artifacts;
- does not use the magnetometer.

### 3C. Rotation-in-place validation — REFERENCE GATE FROZEN

All five retained clean hold-move-hold boresight motions replay in sub-degree start/end agreement with the already validated short-baseline relative-rotation estimator:

| motion | trusted relative angle | gravity closure | persistent-estimator delta |
|---|---:|---:|---:|
| M1 | 17.822 deg | 0.647 deg | 0.656 deg |
| M2 | 28.080 deg | 0.390 deg | 0.777 deg |
| M3 | 45.426 deg | 0.322 deg | 0.339 deg |
| M4 | 23.322 deg | 0.745 deg | 0.736 deg |
| M5 | 24.295 deg | 0.510 deg | 0.859 deg |

Two independent continuous wall captures then exercised the full chain:

```text
ToF normal -> R_body_from_tof -> estimated local/reference orientation
```

The reference rig's effective ToF timing alignment was measured from both runs rather than assumed. Both scans showed the same broad `-20` to `-28 ms` optimum, with `-24 ms` the lowest-RMS 2 ms grid point. That value is stored only as the calibrated timing artifact for `assembly_id=reference-rig-2026-09`; it is not a universal VL53L5CX constant.

Calibrated results:

| capture | excursion | usable | RMS | p95 | max | start/end |
|---|---:|---:|---:|---:|---:|---:|
| wall 1 | 38.503 deg | 443/452 (98.0%) | 0.776 deg | 1.664 deg | 3.530 deg | 0.703 deg |
| wall 2 | 28.412 deg | 452/453 (99.8%) | 0.637 deg | 1.222 deg | 1.859 deg | 0.151 deg |

Both runs reproduced the same values through the calibrated timing-artifact resolver without supplying `-24 ms` numerically.

The frozen empirical **Phase 3 reference wall gate** is:

```text
stream/health integrity:       clean
configured gyro range:         PASS
usable wall observations:      >= 95%
orientation excursion:         >= 20 deg
wall-normal residual RMS:      <= 1.0 deg
wall-normal residual p95:      <= 2.0 deg
wall-normal residual maximum:  <= 5.0 deg
start/end wall-normal delta:   <= 1.0 deg
```

These are project validation bounds derived from the reference evidence, not universal cross-unit sensor specifications. The p95/RMS bounds carry most of the stability meaning; the looser maximum bound prevents one noisy ToF frame from making the gate brittle. Independent builds may motivate later evidence-based revisions, but old validation evidence must retain the gate version used at the time.

The executable gate is `host/python/validate_phase3_wall.py`; the pure numeric contract is `host/python/rangeweave_phase3_gate.py`. Full evidence is in [`validation/orientation-reference-rig-2026-09.md`](validation/orientation-reference-rig-2026-09.md).

### 3D. Magnetometer integration, later

Do not use LIS3MDL heading as an unquestioned yaw reference yet.

Before enabling magnetic heading:

- physically establish `mag_sensor -> device_body` mapping;
- implement hard-iron and soft-iron calibration;
- characterize magnetic disturbances on the real assembly/environment;
- add confidence/disturbance gating so bad magnetic data can be rejected;
- validate heading improvement against repeatable rotations/return tests.

The six-axis estimator must remain useful when magnetic heading is unavailable or untrusted.

### 3E. Orientation-aware ToF geometry — NEXT AFTER EXECUTABLE GATE REPLAY

Once both retained wall captures reproduce PASS through the final executable gate:

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
- Android parity;
- identical calibration values or residual distributions across independently assembled units.

## Exit gate

The first gravity-referenced Phase 3 orientation gate is satisfied when a calibrated replayable orientation path passes the executable reference wall validation with frozen frame/quaternion conventions, clean acquisition health and visible calibration provenance.

Magnetometer-aided heading may be a later Phase 3 extension; it is not required for this first gravity-referenced rotation-only gate.

## Immediate implementation order

1. ~~write/freeze the attitude/quaternion convention and golden composition tests~~ — done;
2. ~~implement the timestamp-driven six-axis orientation core and replay inspector~~ — done;
3. ~~replay existing boresight motion captures as regression evidence~~ — M1-M5 complete;
4. ~~implement continuous wall-normal validation tooling~~ — done;
5. ~~capture independent continuous multi-axis rotation-in-place wall sequences~~ — two complete;
6. ~~measure and represent ToF/IMU timing alignment without hard-coding a sensor-model constant~~ — reference per-build artifact implemented;
7. ~~freeze empirical reference wall acceptance bounds~~ — done;
8. **replay both retained wall captures through `validate_phase3_wall.py` and record PASS/FAIL**;
9. add orientation-aware ToF geometry/viewing;
10. then begin magnetometer mapping/calibration and later full 6DoF pose work.
