# ToF / device-body rotational extrinsics

**Status: v1 rotation contract and synthetic fixed-plane boresight solver implemented; physical IMU-to-body axis validation and live capture integration remain to be completed.**

Rangeweave distinguishes the VL53L5CX optical frame from the completed sensing head's mechanical/body frame.

That distinction is intentional. A builder should be able to solder and assemble a working Rangeweave unit without mechanically aligning the ToF package to sub-degree precision. Small PCB, package, bracket and enclosure errors can make the optical boresight point a few degrees away from the direction that visually appears to be "straight ahead".

## Frames

`tof_optical` remains the project-owned VL53L5CX optical frame:

```text
+X = image/scene right
+Y = image down
+Z = optical forward
```

`device_body` is a separate right-handed assembled-device frame whose nominal axes use the same handed orientation:

```text
+X = device right
+Y = device down
+Z = intended mechanical forward direction
```

The nominal design therefore has parallel `tof_optical` and `device_body` axes, but an individual assembled unit is not assumed to achieve that exactly.

The physical mapping from the LSM6DSOX package axes into `device_body` is **not yet frozen**. It must be established experimentally before IMU-derived body rotations are admitted to physical boresight calibration.

## Rotation contract

The v1 ToF/body extrinsic contains rotation only:

```text
v_body = R_body_from_tof * v_tof
```

`host/python/rangeweave_extrinsics.py` stores this as a 3x3 proper rotation matrix in schema:

```text
rangeweave.tof-body-rotation
```

A nominal uncalibrated unit uses the identity rotation:

```text
R_body_from_tof = I
```

This is a `nominal-fallback`, not a claim that every physical unit is perfectly aligned.

Translation between the ToF optical centre and a later device-body/IMU origin is a separate calibration problem. It is deliberately not hidden inside the v1 rotational artifact.

## Why this is separate from the 64-zone geometry profile

The ToF geometry profile describes rays **inside `tof_optical`**:

```text
p_tof = Z * (x_per_z, y_per_z, 1)
```

A common package/assembly rotation should not be absorbed independently into all 64 zone slopes. Keeping the two calibrations separate allows:

1. the ST-derived 64-zone profile to remain a useful nominal optical model;
2. a builder-specific boresight rotation to correct mechanical assembly error;
3. optional later per-device ray calibration to refine the intrinsic optical lattice;
4. downstream IMU/world transforms to reason about a clearly named body frame.

## Live plane-alignment aid

The live ToF viewer retains the colour 8x8 depth image and adds a side panel.

For every usable frame it projects the 64 distances with the current nominal geometry, fits:

```text
Z = aX + bY + c
```

and reports the equivalent known-plane convention:

```text
Rx = top/bottom plane tilt relative to tof_optical
Ry = left/right plane yaw relative to tof_optical
```

It also reports:

- valid zone count;
- RMS point-to-plane residual;
- maximum absolute residual;
- whether the nominal geometry fallback is being used.

This is a **plane alignment tool**, not a device-body orientation measurement. A mechanically crooked ToF package can still show `Rx ~= 0` and `Ry ~= 0` after the complete assembly is aimed so that its optical frame faces a plane squarely.

That distinction is useful: it tells the builder where the ToF actually looks without requiring them to infer the optical axis from the PCB or package edges.

## Fixed-plane boresight calibration

`solve_fixed_plane_boresight()` implements the first dependency-free rotational solver.

The physical concept is:

1. keep one flat board or wall fixed;
2. hold the completed Rangeweave unit stationary at several poses;
3. obtain a ToF plane normal for each pose;
4. obtain the **relative device-body rotation** between poses from a validated motion source;
5. solve for the one constant `R_body_from_tof` that makes every observation describe the same fixed plane in a common reference frame.

For pose `k`:

```text
n_ref =
    R_reference_from_body[k]
    * R_body_from_tof
    * n_tof[k]
```

The absolute room orientation of `n_ref` is an unknown nuisance parameter. The solver therefore does **not** require the user to measure the board's yaw with a protractor or align it to a room datum.

The v1 solver performs a deterministic coordinate search over a small assembly-error envelope and reports:

- fitted X/Y/Z rotation parameters;
- common reference-plane normal;
- RMS and maximum angular consistency error;
- simple per-axis observability diagnostics.

Multiple poses with useful multi-axis motion are required. Repeating the same pose cannot determine a 3D boresight rotation and is rejected as underconstrained.

## Intended builder experience

Normal use remains:

```text
assemble Rangeweave
        |
        v
nominal ST ToF geometry
+
nominal identity ToF/body rotation
        |
        v
works immediately
```

Optional guided refinement becomes:

```text
"Calibrate sensor alignment"
        |
        v
point at one fixed board/wall
        |
        v
hold still, turn left/right/up/down
(no exact angle targets)
        |
        v
ToF plane normals + relative IMU/body rotations
        |
        v
per-device R_body_from_tof
```

The user should not need to measure a few degrees of package yaw manually.

## Remaining physical validation

Before this can become the builder-facing automatic calibration path, Rangeweave still needs to:

1. physically establish the LSM6DSOX package-axis mapping into `device_body`;
2. define the relative-orientation estimator used during a short calibration sequence;
3. feed synchronized stationary ToF plane observations and body rotations into the solver;
4. validate the recovered boresight against held-out physical poses;
5. decide how/where the per-device extrinsic artifact is stored and selected;
6. later calibrate translation if applications require a precise common sensor origin.

The current synthetic tests deliberately impose a known ToF/body boresight error and verify that fixed-plane observations plus known relative body rotations recover it.
