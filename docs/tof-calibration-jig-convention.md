# ToF calibration jig geometry convention

**Status: candidate physical-workflow convention for Phase 2 calibration.**

This document defines the geometry that a reproducible Rangeweave ToF calibration jig and manifest must describe. It sits above the generic known-plane solver in [`tof-geometry-calibration.md`](tof-geometry-calibration.md).

The goal is to make a physical calibration pose unambiguous and mechanically reproducible without depending on informal meanings of "pitch" and "yaw".

## Frozen frame used by the jig

All quantities are expressed in the project-owned `tof_optical` frame:

```text
+X = scene/image right
+Y = image down
+Z = forward from the sensor into the scene
```

The frame is right-handed.

For this first physical workflow, the origin is the **nominal VL53L5CX optical centre** used by the existing geometry layer. Exact per-device optical-centre translation remains a separate later calibration problem.

## Centre-pivot plane

The calibration board has a defined pivot point on the `tof_optical` Z axis:

```text
P = (0, 0, D)
```

where `D = pivot_distance_mm > 0` is measured from the nominal optical centre.

The board must rotate about this same physical pivot point for every pose. Therefore the pivot remains fixed while the plane orientation changes.

A fronto-parallel board has the canonical plane normal:

```text
n0 = (0, 0, +1)
```

and plane equation:

```text
Z = D
```

The chosen +Z normal is a mathematical convention; it does not claim that the visible board surface physically "faces" +Z.

## Rotation convention

Do not store calibration poses using unspecified `pitch` or `yaw` semantics.

The executable v1 pose convention is named:

```text
tof-optical-centre-pivot-rx-then-ry-v1
```

It uses **active, right-handed rotations about the fixed `tof_optical` axes**, in this order:

1. rotate the canonical normal about `+X` by `rotation_x_deg`;
2. rotate the result about `+Y` by `rotation_y_deg`.

Equivalently:

```text
n = Ry(rotation_y_deg) * Rx(rotation_x_deg) * (0, 0, 1)
```

For `rx = rotation_x_deg` and `ry = rotation_y_deg`, expressed in radians:

```text
nx = sin(ry) * cos(rx)
ny = -sin(rx)
nz = cos(ry) * cos(rx)
```

The rotations preserve unit length, so `n` is already a unit normal.

Mixed-axis rotations are non-commutative. `Rx` then `Ry` is therefore project policy, not interchangeable wording.

## Sign meaning in the real scene

Because `tof_optical +Y` points downward, the right-hand signs are worth stating physically.

### Positive `rotation_x_deg`

For `+Rx`:

- the lower `+Y` side of the board is **farther** from the sensor than the pivot;
- the upper `-Y` side is **closer**.

For a pure X rotation:

```text
Z(Y) = D + tan(rx) * Y
```

### Positive `rotation_y_deg`

For `+Ry`:

- the right `+X` side of the board is **closer** to the sensor than the pivot;
- the left `-X` side is **farther**.

For a pure Y rotation:

```text
Z(X) = D - tan(ry) * X
```

These sign meanings are covered by golden regression tests rather than being left as diagram-only documentation.

## Plane offset

The generic calibration solver consumes planes in the form:

```text
nx*X + ny*Y + nz*Z = d
```

Since the plane always contains the fixed pivot `P = (0,0,D)`:

```text
d = n dot P = nz * D
```

No separate sensor-to-board distance measurement is therefore required for each tilt, provided the physical board really rotates about the defined pivot.

The v1 workflow requires `nz > 0`, keeping the calibration plane in the forward-facing half-space used by the current solver.

## Golden examples

At `D = 800 mm`:

### Fronto-parallel

```text
Rx = 0 deg
Ry = 0 deg
n  = (0, 0, 1)
d  = 800 mm
```

### +15 deg about X

```text
n  = (0,
      -0.258819045,
       0.965925826)
d  = 772.740661 mm
```

The bottom of the board is farther away than the top.

### +15 deg about Y

```text
n  = (0.258819045,
      0,
      0.965925826)
d  = 772.740661 mm
```

The right side of the board is closer than the left.

### Mixed pose: +12 deg X then +10 deg Y

```text
n  = ( 0.169853548,
      -0.207911691,
       0.963287341)
d  = 770.629873 mm
```

This mixed case is a regression golden value specifically to freeze rotation order.

## Reference implementation

`host/python/rangeweave_tof_calibration_workflow.py` implements this convention as `PivotPlanePose` and converts a 64-zone stationary observation into the existing solver's `CalibrationPlane` representation.

The implementation intentionally does not yet:

- choose the physical pivot distance `D`;
- prescribe board size or material;
- prescribe hinge/gimbal construction;
- extract robust zone distances from a recorded capture;
- define the calibration manifest format;
- decide the final fit and held-out validation poses.

Those are the next parts of the physical workflow. This document freezes only the coordinate/rotation geometry they must obey.
