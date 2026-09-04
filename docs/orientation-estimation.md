# Orientation estimation plan

**Status:** Phase 3 is the active estimation layer. The acquisition/timing path, `imu_sensor -> device_body` mapping, ToF geometry convention and reference-rig `tof_optical -> device_body` rotational boresight are already validated and provide the inputs this layer needs.

The project-owned attitude conventions are now frozen in [`attitude-conventions.md`](attitude-conventions.md). A first deterministic Python gyro+gravity orientation core and replay inspector are implemented as a **Phase 3 candidate under physical validation**.

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

`host/python/rangeweave_orientation.py` now provides a timestamp-driven gyro + accelerometer baseline and `host/python/inspect_orientation.py` replays it on canonical captures.

Current properties:

- integrates mapped body-frame gyro using LSM hardware-timestamp correlation;
- initializes gravity-referenced pitch/roll and a local, non-global yaw zero from an explicit stationary initial interval;
- estimates a **fixed initial gyro bias only from that explicit stationary interval**, avoiding silent online reinterpretation of slow real motion as bias;
- uses accelerometer only as a gravity/specific-force observation when magnitude is credible;
- exposes gravity-correction weight and innovation diagnostics;
- remains deterministic and standard-library-only under replay;
- keeps estimator state separate from permanent calibration artifacts;
- does not use the magnetometer.

The proportional gravity gain and confidence behaviour are initial engineering defaults, not physically promoted tuning constants. They must be evaluated against recorded and new physical evidence before becoming acceptance criteria.

### 3C. Rotation-in-place validation — ACTIVE

The first regression level is complete: all five retained clean hold-move-hold boresight motions have been replayed through the persistent estimator and compared with the already validated short-baseline relative-rotation estimator.

| motion | trusted relative angle | gravity closure | persistent-estimator delta |
|---|---:|---:|---:|
| M1 | 17.822 deg | 0.647 deg | 0.656 deg |
| M2 | 28.080 deg | 0.390 deg | 0.777 deg |
| M3 | 45.426 deg | 0.322 deg | 0.339 deg |
| M4 | 23.322 deg | 0.745 deg | 0.736 deg |
| M5 | 24.295 deg | 0.510 deg | 0.859 deg |

All five captures had clean decoder/sequence/health status and acceptable +/-500 deg/s range utilisation. Across varied real motions the persistent estimator remained in sub-degree start/end agreement with the trusted relative estimator. M2 had a noisier startup window than the others but still produced sub-degree agreement.

This is strong regression evidence for axis/sign/composition correctness and basic real-data behaviour, but it is **not** yet the Phase 3 exit test. No final orientation acceptance threshold is frozen from these hold-move-hold regressions alone.

The next physical validation is one continuous multi-axis rotation-in-place capture against a fixed wall/static plane. For every usable ToF frame:

```text
ToF normal -> R_body_from_tof -> estimated local/reference orientation
```

the same physical wall normal should remain approximately constant throughout the motion.

Plane normals are useful here because they are insensitive to the small translations caused by a photographic-head pivot that does not coincide with the ToF optical centre.

The continuous validation should report at least:

- stream/health integrity and gyro-range utilisation;
- orientation initialization quality and gravity-correction diagnostics;
- number/share of usable ToF frames;
- transformed local wall-normal mean/reference;
- angular residual distribution over time (median, RMS, high percentile and maximum);
- start/end stationary wall-normal consistency;
- evidence of any residual correlated with fast rotation or acceleration-gated periods.

Protocol v0.1 records `mcu_ready_us` when software observes VL53L5CX data-ready. The physical result was available no later than that timestamp, but the exact internal ranging instant is not exposed. Continuous-motion validation must therefore keep ToF timing semantics visible rather than silently treating the read-complete time as measurement time; if residuals correlate with angular rate, a repeatable effective ToF timing offset may need to become an explicit calibration/timing parameter.

Measure, do not pre-assume, the acceptable angular residual/drift bounds from this evidence.

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
2. ~~implement the timestamp-driven six-axis orientation core and replay inspector~~ — candidate implemented;
3. ~~replay existing boresight motion captures as regression evidence~~ — M1-M5 complete, all sub-degree start/end delta;
4. **implement continuous wall-normal validation tooling**;
5. make one new continuous multi-axis rotation-in-place capture against a static wall;
6. evaluate transformed wall-normal stability and freeze empirical acceptance bounds;
7. add orientation-aware ToF geometry/viewing;
8. then begin magnetometer mapping/calibration and later full 6DoF pose work.
