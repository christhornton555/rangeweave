# Orientation estimation plan

**Status:** Phase 3 is the next major estimation layer. The acquisition/timing path, `imu_sensor -> device_body` mapping, ToF geometry convention and reference-rig `tof_optical -> device_body` rotational boresight are already validated and provide the inputs this layer needs.

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

### 3A. Freeze attitude conventions

Before a persistent attitude state is implemented, document and test:

- quaternion component order;
- transform direction represented by the quaternion/matrix;
- active versus passive rotation interpretation;
- quaternion multiplication/composition order;
- angular-velocity sign/frame convention;
- gravity direction and initialization convention;
- definition of the initial local/reference frame;
- explicit statement that yaw is unobservable from accelerometer + gyro alone.

The project should choose these conventions deliberately rather than inheriting them from a Python, Android or graphics API.

### 3B. Six-axis orientation baseline

Implement a timestamp-driven gyro + accelerometer estimator in Python first.

Required properties:

- integrate gyro using sensor timestamps;
- initialize pitch/roll from gravity while leaving yaw arbitrary;
- use accelerometer only as a gravity observation when its magnitude/behaviour is credible;
- maintain or estimate gyro bias without silently treating real motion as bias;
- expose diagnostics/confidence rather than always returning a trusted attitude;
- remain replayable and deterministic from a recorded capture;
- keep estimator state separate from permanent calibration artifacts.

A conventional complementary/Mahony-style or equivalent physics-based baseline is appropriate before considering learned corrections.

### 3C. Rotation-in-place validation

Validate on recorded data before depending on live behaviour.

Use two levels of evidence:

1. **Existing calibration captures:** replay known clean hold/move/hold motions as regression cases for sign, composition and gravity consistency.
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

1. write/freeze the attitude/quaternion convention and golden composition tests;
2. implement the timestamp-driven six-axis orientation core and replay inspector;
3. replay existing boresight motion captures as regression evidence;
4. make one new continuous multi-axis rotation-in-place capture against a static wall;
5. evaluate transformed wall-normal stability and freeze empirical acceptance bounds;
6. add orientation-aware ToF geometry/viewing;
7. then begin magnetometer mapping/calibration and later full 6DoF pose work.
