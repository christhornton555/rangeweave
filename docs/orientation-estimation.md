# Orientation estimation plan

**Status:** Phase 3 is the active estimation layer. The acquisition/timing path, `imu_sensor -> device_body` mapping, ToF geometry convention and reference-rig `tof_optical -> device_body` rotational boresight are already validated and provide the inputs this layer needs.

The project-owned attitude conventions are now frozen in [`attitude-conventions.md`](attitude-conventions.md). A first deterministic Python gyro+gravity orientation core and replay inspector are implemented as an **unvalidated Phase 3 candidate**; real-capture regression and rotation-in-place validation are still required before persistent attitude is a supported reference-path claim.

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

### 3C. Rotation-in-place validation — NEXT

Validate on recorded data before depending on live behaviour.

Use two levels of evidence:

1. **Existing calibration captures:** replay known clean hold/move/hold motions as regression cases for sign, composition and gravity consistency. The replay inspector can compare the persistent estimator's start-to-end rotation against the already validated short-baseline relative-rotation estimator.
2. **New continuous rotation capture:** keep the sensing head at approximately one location, rotate it through varied pitch/yaw/roll poses in front of a fixed wall or other static geometry, and evaluate the complete attitude path.

For the wall test, plane normals are especially useful because they are insensitive to small translations caused by the photographic head's pivot not coinciding with the ToF optical centre. After applying:

```text
ToF normal -> R_body_from_tof -> estimated local/reference orientation
```

the same physical wall normal should remain approximately constant throughout the rotation.

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

1. ~~write/freeze the attitude/quaternion convention and golden composition tests~~ — done on the Phase 3 implementation branch;
2. ~~implement the timestamp-driven six-axis orientation core and replay inspector~~ — candidate implemented; physical/replay validation pending;
3. **replay existing boresight motion captures as regression evidence**;
4. make one new continuous multi-axis rotation-in-place capture against a static wall;
5. evaluate transformed wall-normal stability and freeze empirical acceptance bounds;
6. add orientation-aware ToF geometry/viewing;
7. then begin magnetometer mapping/calibration and later full 6DoF pose work.
