# ToF / device-body rotational extrinsics

**Status: v1 rotation contract, physical reference-rig IMU mapping, short-baseline relative rotation, fixed-plane boresight sequencing and provisional physical boresight solve are implemented/validated. Final held-out boresight validation and builder-facing guided capture remain open.**

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

The LSM6DSOX package-axis mapping has now been physically established for the current rigid breadboard assembly:

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

## Relative body rotation

`host/python/rangeweave_imu_relative.py` estimates the short-baseline rotation between two stationary poses using:

- recorded `CLOCK_SYNC` observations for the LSM tick/time scale;
- the physically validated reference-rig IMU mapping;
- data-selected stationary endpoint windows;
- endpoint gyro-bias estimates with interpolation during integration;
- gyro integration in `device_body`;
- stationary accelerometer gravity vectors as an independent closure check.

This is deliberately not a world-attitude filter and does not use magnetometer heading.

The acquisition firmware now uses LSM6DSOX gyro `CTRL2_G = 0x44` (104 Hz, +/-500 deg/s). The earlier +/-250 deg/s configuration was exceeded during a real mixed-axis calibration motion. Host tooling now reports configured full-scale utilisation and warns at 80% / rejects at 90% for boresight use.

Physical +/-500 deg/s validation on the reference rig recovered a ~21 deg pitch with ~0.78 deg gravity closure and a later compound motion with ~0.30 deg closure while using no more than ~14% of configured full scale.

## Fixed-plane boresight calibration

For pose `k`:

```text
n_ref =
    R_reference_from_body[k]
    * R_body_from_tof
    * n_tof[k]
```

The absolute room orientation of the plane normal `n_ref` is an unknown nuisance parameter. The solver therefore does not require the user to measure the wall's yaw or align the room to an external datum.

The sequence implementation uses a stationary baseline plus alternating relative-motion and stationary-ToF captures:

```text
P0
M1 -> P1
M2 -> P2
M3 -> P3
...
```

It composes each `R_previous_from_current` into a common reference frame and feeds the corresponding ToF plane normals to `solve_fixed_plane_boresight()`.

At least four stationary observations are required. Multiple axes must be excited sufficiently; repeated or tiny motions do not constrain all three boresight parameters.

## Recommended builder target

For boresight, the default recommendation is now an ordinary **flat wall**, not a small calibration board.

For P0:

- choose a flat, clear, preferably matt-painted wall;
- aim near the centre of at least a **1 m x 1 m unobstructed patch**;
- place the ToF approximately **500 mm from the wall**;
- aim approximately square-on.

These numbers are setup guidance, not measured solver inputs. Their purpose is to keep all ToF beams on the same plane while allowing useful pitch/yaw/roll excursions.

A 3-way photographic pan/tilt/roll head is a convenient low-cost way to hold a breadboard or sensor package still and change orientation repeatably. The head's rotation axes do not need to intersect the ToF optical centre because the boresight solve uses plane normals rather than absolute sensor position.

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

## Physical solver evidence

A clean four-pose physical sequence on the reference rig produced a provisional result of approximately:

```text
Rx -1.440 deg
Ry +1.910 deg
Rz +1.210 deg
normal RMS 0.535 deg
normal max 0.666 deg
```

The fit was small, plausible and well inside the solver's +/-20 deg search envelope. It remains provisional because the intended additional held-out/roll-rich validation pose failed under the old +/-250 deg/s gyro range and the complete sequence has not yet been repeated under +/-500 deg/s.

See [`validation/boresight-reference-rig-2026-09.md`](validation/boresight-reference-rig-2026-09.md).

## Remaining work

Before this becomes the final builder-facing automatic calibration path:

1. implement one guided command that owns capture warm-up and HOLD / MOVE NOW / STOP MOVING cues;
2. repeat the complete fixed-wall sequence under +/-500 deg/s;
3. reserve a geometrically different pose for held-out validation;
4. promote a versioned per-device `rangeweave.tof-body-rotation` artifact with capture/firmware/geometry provenance;
5. validate independently assembled units;
6. later calibrate translation and the magnetometer/body mapping where required.

Do not use the provisional reference-rig rotation as a universal extrinsic for other builds.
