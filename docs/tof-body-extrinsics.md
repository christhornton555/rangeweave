# ToF / device-body rotational extrinsics

**Status: v1 rotation contract, physical reference-rig IMU mapping, guided P0-P5 fixed-plane boresight capture, held-out P5 validation and versioned artifact generation are implemented.**

Rangeweave distinguishes the VL53L5CX optical frame from the completed sensing head's mechanical/body frame. That distinction lets a builder assemble a useful system without mechanically aligning the ToF package to sub-degree precision.

## Frames

`tof_optical`:

```text
+X = image/scene right
+Y = image down
+Z = optical forward
```

`device_body`:

```text
+X = device right
+Y = device down
+Z = intended mechanical forward direction
```

Both are right-handed. The nominal design makes them parallel, but a real PCB/bracket/enclosure can introduce a small common boresight rotation.

## Physical IMU mapping on the current reference rig

The LSM6DSOX package-axis mapping has been physically established for the current rigid breadboard assembly:

```text
device_body +X -> imu_sensor -X
device_body +Y -> imu_sensor -Z
device_body +Z -> imu_sensor -Y
```

The Python reference implementation therefore maps:

```text
body X = -imu X
body Y = -imu Z
body Z = -imu Y
```

This mapping is **specific to this physical mounting**. A third-party assembly that rotates or flips the IMU must establish its own `imu_sensor -> device_body` rotation rather than copying these signs.

The LIS3MDL `mag_sensor -> device_body` mapping is still unvalidated.

## Rotation contract

The v1 ToF/body extrinsic contains rotation only:

```text
v_body = R_body_from_tof * v_tof
```

`host/python/rangeweave_extrinsics.py` stores this as a 3x3 proper rotation matrix in schema:

```text
rangeweave.tof-body-rotation
```

An uncalibrated unit uses the identity rotation as a `nominal-fallback`. Translation between sensor origins is a separate later calibration problem.

## Why this is separate from the 64-zone geometry profile

The ToF geometry profile describes rays **inside `tof_optical`**:

```text
p_tof = Z * (x_per_z, y_per_z, 1)
```

A common package/assembly rotation belongs in `R_body_from_tof`; it must not be absorbed independently into the 64 zone slopes. This keeps intrinsic optical calibration and rigid assembly alignment independently testable and replaceable.

The current boresight plane path uses the built-in ST-derived geometry profile with role `nominal-fallback` unless a future workflow explicitly supplies a calibrated intrinsic profile. A promoted boresight artifact records the geometry role used.

## Relative body rotation

`host/python/rangeweave_imu_relative.py` estimates the short-baseline rotation between two stationary poses using:

- recorded `CLOCK_SYNC` observations for the LSM tick/time scale;
- the physically validated reference-rig IMU mapping;
- data-selected stationary endpoint windows;
- endpoint gyro-bias estimates with interpolation during integration;
- gyro integration in `device_body`;
- stationary accelerometer gravity vectors as an independent closure check.

This is deliberately not a world-attitude filter and does not use magnetometer heading.

The acquisition firmware uses LSM6DSOX gyro `CTRL2_G = 0x44` (104 Hz, +/-500 deg/s). The earlier +/-250 deg/s configuration was exceeded during a real mixed-axis calibration motion. Host tooling reports configured full-scale utilisation and warns at 80% / rejects at 90% for boresight use.

## Fixed-plane boresight calibration

For pose `k`:

```text
n_ref =
    R_reference_from_body[k]
    * R_body_from_tof
    * n_tof[k]
```

The absolute room orientation of the plane normal `n_ref` is an unknown nuisance parameter. The solver therefore does not require the user to measure the wall's yaw or align the room to an external datum.

The standard sequence uses a stationary baseline plus five relative-motion / stationary-ToF pairs:

```text
P0
M1 -> P1
M2 -> P2
M3 -> P3
M4 -> P4
M5 -> P5
```

It composes each `R_previous_from_current` into a common reference frame and feeds the corresponding ToF plane normals to `solve_fixed_plane_boresight()`.

At least four stationary observations are mathematically required, but physical validation showed that a four-pose training set could leave one boresight direction weakly constrained even with clean data. The builder workflow therefore now recommends **six stationary poses** spanning multiple axes.

## Held-out validation contract

P5 is not used in the initial training fit. P0-P4 determine the candidate extrinsic and common wall normal. M5 independently supplies P5's body orientation. The calibration then predicts the wall normal that the ToF should observe at P5:

```text
n_tof_pred =
    R_body_from_tof^T
    * R_reference_from_body[P5]^T
    * n_ref
```

Only after that prediction is formed is the P5 ToF plane normal revealed. The angular separation between predicted and observed P5 normals is the held-out validation error.

This means P5's **ToF observation** is held out; M5's IMU-derived orientation is an independent input. Fully holding out body orientation as well would require an external pose reference.

## Recommended builder target and support

Use an ordinary **flat wall**, not a small calibration board, when practical.

For P0:

- choose a flat, clear, preferably matt-painted wall;
- aim near the centre of at least a **1 m x 1 m unobstructed patch**;
- place the ToF approximately **500 mm from the wall**;
- aim approximately square-on.

These numbers are setup guidance, not measured solver inputs. Their purpose is to keep all ToF beams on the same plane while allowing useful pitch/yaw/roll excursions.

A 3-way photographic pan/tilt/roll head is a convenient low-cost support. The reference validation used an improvised rigid carrier built from upper/lower MDF plates, the camera mount's own threaded connectors, a selfie-stick/support clamp and spring clamps holding the breadboard. The exact fixture is not important; rigidity, cable slack and enough settling time are.

See [`boresight-calibration.md`](boresight-calibration.md) for the current physical procedure.

## Quality gates

Current boresight-specific workflow gates are empirical reference-rig safeguards:

```text
minimum relative motion:          5 deg
gyro warning / rejection:         80% / 90% of configured full scale
maximum gyro/gravity closure:     2 deg
maximum ToF plane RMS residual:   10 mm
maximum ToF plane residual:       30 mm
maximum ToF half-capture drift:   10 mm
```

These values are not universal LSM6DSOX/VL53L5CX specifications.

Guided motion capture uses a 3 s discarded warm-up, 5 s recorded initial hold, 10 s movement allowance and 12 s hands-off final settling hold. The long final hold is based on a physical M4 failure in which camera-head settling contaminated the endpoint estimate.

## Final reference-rig evidence

The completed September 2026 P0-P5 sequence produced the following P0-P4 candidate fit:

```text
Rx +5.430 deg
Ry +3.780 deg
Rz -0.300 deg
normal RMS 0.847 deg
```

A geometrically different P5 was excluded from that fit. Its predicted versus observed ToF wall normal differed by only:

```text
0.644 deg
```

After revealing P5, the six-pose refit became:

```text
Rx +5.880 deg
Ry +3.930 deg
Rz +0.160 deg
normal RMS 0.797 deg
normal max 1.648 deg
```

The full 3-D rotation changed by only 0.639 deg after P5 was added. This contrasts with an earlier P0-P3 -> P4 test, where the fit moved by 5.607 deg because the four-pose training geometry had not yet constrained all axes strongly enough.

See [`validation/boresight-reference-rig-2026-09.md`](validation/boresight-reference-rig-2026-09.md).

## Artifact generation

After a successful guided P0-P5 session:

```powershell
py host/python/generate_boresight_artifact.py <session-prefix>
```

The output defaults to:

```text
calibration/tof-body-rotation-<session-prefix>.json
```

The generator re-runs physical quality checks, performs the P0-P4 -> held-out P5 validation, refits all six accepted poses, and writes the exact `rotation_body_from_tof` matrix plus fit/validation/capture/configuration provenance.

## Remaining work

The reference-rig rotational boresight path is validated. Remaining adjacent work is separate:

1. reproduce the workflow on independently assembled units;
2. later calibrate rigid translation between sensor origins where required;
3. establish `mag_sensor -> device_body` mapping and magnetic calibration;
4. define/fuse world-frame attitude and proceed to orientation/odometry validation.

Do not use the reference-rig rotation as a universal extrinsic for other builds.
