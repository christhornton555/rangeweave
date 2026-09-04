# Optional ToF intrinsic known-plane calibration workflow

**Status: implemented candidate physical-pose convention and robust stationary-capture reducer for optional per-device VL53L5CX ray/geometry refinement. This is not the ToF/body boresight workflow.**

Rangeweave does **not** require per-device ToF geometry calibration before use. The normal path uses the built-in ST-derived `nominal-fallback` geometry profile. This workflow is an optional precision-refinement step for builders who want to characterise their particular VL53L5CX and generate a per-device `tof_geometry.json` profile.

For the separate task of aligning the complete ToF assembly to `device_body`, use [`boresight-calibration.md`](boresight-calibration.md). Boresight calibration normally uses a fixed wall and does **not** require measured wall orientation.

## What this intrinsic workflow estimates

The ToF geometry profile describes the per-zone rays **inside `tof_optical`**. A common rigid package/assembly rotation does not belong here; it belongs in `R_body_from_tof`.

The intrinsic solver needs observations of known physical planes so it can refine independent zone geometry without assuming the nominal ST lattice is exact.

## Practical target

A reasonably flat board is a convenient intrinsic-calibration target because its orientation and a point on its plane can be measured independently.

A roughly **600 mm x 600 mm / 24 x 24 inch** board remains a useful portable option, but it is not a mathematical requirement. A larger known plane is equally valid. The target must be large enough that the zones used by a capture remain on the same measured plane.

For the current nominal profile, 500-600 mm sensor-to-target distances are convenient starting values. Exact distance is not fixed by the solver; the measured plane point for each capture is what matters.

Unlike boresight calibration, this intrinsic workflow requires **known plane geometry for each capture**. A wall is only useful here if its relevant orientation/point can be measured sufficiently well for the intended intrinsic calibration.

## Required observation for each capture

Each intrinsic calibration pose needs:

1. plane orientation in the frozen `tof_optical` frame;
2. one known point on that plane, expressed in `tof_optical` millimetres;
3. one stationary Rangeweave capture containing the ToF observations.

The known point may move between captures. No centre pivot or constant sensor-target distance is required.

## Frozen frame and orientation convention

`tof_optical` is right-handed:

```text
+X = scene/image right
+Y = image down
+Z = forward from the sensor into the scene
```

The executable v1 known-plane convention is:

```text
tof-optical-known-point-rx-then-ry-v1
```

A fronto-parallel plane has canonical normal:

```text
n0 = (0, 0, +1)
```

Apply active right-handed rotations about the fixed `tof_optical` axes in this order:

```text
n = Ry(rotation_y_deg) * Rx(rotation_x_deg) * n0
```

For `rx`, `ry` in radians:

```text
nx = sin(ry) * cos(rx)
ny = -sin(rx)
nz = cos(ry) * cos(rx)
```

Because `+Y` points down:

- positive `Rx` makes the lower `+Y` side farther from the sensor;
- positive `Ry` makes the right `+X` side closer to the sensor.

Mixed-axis order is non-commutative and therefore part of the versioned convention.

## Plane position

The solver consumes:

```text
nx*X + ny*Y + nz*Z = d
```

For a known point `P = (Px, Py, Pz)` on the plane:

```text
d = n dot P
```

A common convenience is a measured point approximately on the optical axis:

```text
P = (0, 0, D)
d = nz * D
```

`D` may differ between captures.

## Reducing a stationary capture

`host/python/rangeweave_tof_calibration_capture.py` reduces many ToF frames into one robust 64-zone observation.

For every producer-native zone it records:

- valid positive-return count/fraction;
- median distance;
- MAD (median absolute deviation);
- first-half versus second-half median drift.

The solver-facing value is the median. A zone below the configured valid-return fraction (default `0.90`) becomes `None`; it is not interpolated, mirrored or filled from neighbours.

The reducer currently requires at least 30 distance-bearing ToF frames and structurally rejects known stream/capture integrity problems such as decoder errors, sequence gaps, bad metadata/hash parity, wrong grid/layout or non-zero acquisition-health deltas.

## Stability metrics: generic reducer versus boresight gates

The generic intrinsic-capture reducer **reports** MAD and half-capture drift but does not impose a universal sensor-wide stability threshold. That remains intentional: an intrinsic calibration campaign may need different evidence-based acceptance limits.

The separate boresight workflow now applies its own empirical physical-quality gates (`10 mm` plane RMS, `30 mm` max plane residual, `10 mm` maximum half-drift). Those are boresight workflow safeguards, not generic VL53L5CX intrinsic-calibration specifications.

Inspect an intrinsic candidate capture with:

```powershell
py host/python/inspect_tof_calibration_capture.py <capture>
```

Add diagnostic grids with:

```powershell
py host/python/inspect_tof_calibration_capture.py <capture> --show-grids
```

## Example intrinsic dataset

A builder might measure several known plane poses such as:

```text
Rx +15 deg, Ry   0 deg
Rx -15 deg, Ry   0 deg
Rx   0 deg, Ry +15 deg
Rx   0 deg, Ry -15 deg
Rx +12 deg, Ry +10 deg
```

The exact numbers are not requirements. Use measured values rather than pretending a fixture achieved nominal angles exactly.

Reserve at least one independent pose for held-out validation rather than fitting every observation.

A fronto-parallel pose is useful diagnostically but does not by itself constrain the X/Y ray slopes.

## Optional, not prerequisite

```text
assemble Rangeweave
       |
       v
use built-in ST nominal fallback
       |
       +---- sufficient accuracy? ---- yes ---> use system
       |
       no / want maximum accuracy
       v
optional intrinsic known-plane calibration
       |
       v
tof_geometry.json
```

Capture, replay, raw depth viewing and nominal projection must continue to work without a per-device intrinsic artifact.

## Reference implementation

- `host/python/rangeweave_tof_calibration_workflow.py` implements `KnownPlanePose` and the v1 pose convention.
- `host/python/rangeweave_tof_calibration_capture.py` implements robust stationary reduction.
- `host/python/rangeweave_tof_calibration.py` contains the per-zone known-plane solver.

The intrinsic workflow still does not define the final multi-capture manifest/artifact promotion path or a universal physical fixture. Those remain future refinement tasks.
