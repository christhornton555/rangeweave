# Optional ToF known-plane calibration workflow

**Status: candidate physical-workflow convention and capture reducer for Phase 2 calibration.**

Rangeweave does **not** require per-device ToF geometry calibration before use. The normal out-of-box path uses the built-in ST-derived `nominal-fallback` geometry profile. The workflow described here is an optional precision-refinement step for builders who want to characterise their particular sensor and generate a per-device `tof_geometry.json` profile.

This workflow deliberately does not require a precision gimbal, centre pivot, fixed sensor-board distance, or standardised jig.

## Practical target

A useful physical target is simply a reasonably flat board large enough to fill the sensor field of view at the chosen calibration distance. For the current prototype, a board around **700 mm x 700 mm** is a practical starting point.

The board needs a support that can hold it stationary while a short capture is recorded and allow it to be set to several accurately measured orientations. The support may be as simple or elaborate as the builder finds convenient: clamps, wedges, a hinged stand, an adjustable easel, a photographic support, or a purpose-built fixture are all compatible with the software model.

The important requirements are measurement and stability, not a particular mechanical design.

## What must be known for each capture

Each calibration capture needs:

1. the board orientation in the frozen `tof_optical` frame;
2. one known point that lies on the board plane, expressed in `tof_optical` millimetres;
3. the stationary Rangeweave capture containing the 64 ToF zone measurements.

The known point will normally be a marked point near the board centre, but mathematically it may be any measured point on the plane.

The known point may move between captures. The board does **not** have to rotate about that point, and the sensor-board distance does **not** have to remain constant.

## Frozen frame

All quantities are expressed in the project-owned `tof_optical` frame:

```text
+X = scene/image right
+Y = image down
+Z = forward from the sensor into the scene
```

The frame is right-handed. For this workflow, the origin remains the nominal VL53L5CX optical centre used by the existing geometry layer. Exact per-device optical-centre translation is a separate later calibration problem.

## Orientation convention

To avoid ambiguous uses of `pitch` and `yaw`, the executable v1 convention is named:

```text
tof-optical-known-point-rx-then-ry-v1
```

A fronto-parallel board has canonical normal:

```text
n0 = (0, 0, +1)
```

The normal is transformed by active, right-handed rotations about the fixed `tof_optical` axes in this order:

1. rotate about `+X` by `rotation_x_deg`;
2. rotate the result about `+Y` by `rotation_y_deg`.

Equivalently:

```text
n = Ry(rotation_y_deg) * Rx(rotation_x_deg) * (0, 0, 1)
```

For angles `rx` and `ry` in radians:

```text
nx = sin(ry) * cos(rx)
ny = -sin(rx)
nz = cos(ry) * cos(rx)
```

Mixed-axis rotations are non-commutative, so the order is project policy rather than interchangeable wording.

Because `tof_optical +Y` points downward:

- positive `Rx` makes the board's lower `+Y` side farther from the sensor;
- positive `Ry` makes the board's right `+X` side closer to the sensor.

These signs and the mixed-axis order are regression-tested.

## Plane position

The calibration solver consumes a plane:

```text
nx*X + ny*Y + nz*Z = d
```

If a measured point on the board is:

```text
P = (Px, Py, Pz)
```

then:

```text
d = n dot P
  = nx*Px + ny*Py + nz*Pz
```

This is the general workflow. There is no fixed-pivot assumption.

### Simple centre-on-axis case

A convenient setup is to place a marked board-centre point approximately on the optical axis and measure its distance `D` from the nominal optical origin:

```text
P = (0, 0, D)
```

Then:

```text
d = nz * D
```

`D` may be different for every capture. The board does not need to pivot about that point.

## Reducing a stationary capture

`host/python/rangeweave_tof_calibration_capture.py` converts the many ToF frames in one stationary capture into one 64-zone observation for the known-plane solver.

For each producer-native zone it records:

- the number and fraction of valid positive distance returns;
- the robust **median distance** across the capture;
- **MAD** (median absolute deviation) around that median;
- the absolute difference between the first-half and second-half medians as a simple temporal-drift diagnostic.

The solver-facing value is the median, not the mean. A single large range outlier therefore does not pull the calibration observation away from the stable population.

A zone contributes its median only when it reaches the configured valid-return fraction. The current default is:

```text
min_valid_fraction = 0.90
```

A lower-coverage zone is emitted as `None`. Nothing is filled, mirrored, interpolated, or copied from a neighbouring zone; the existing independent-zone solver may still use that zone's valid observations from other plane poses.

The current default also requires at least 30 distance-bearing ToF frames. This is a structural minimum rather than the recommended capture duration; a several-second stationary capture is preferable.

### Capture integrity

A calibration capture is structurally rejected when the existing capture/decode evidence reports a known integrity problem, including:

- wrong ToF grid size or producer layout;
- too few distance-bearing frames;
- decoder bad frames;
- semantic decoding errors;
- sequence gaps;
- metadata/hash parity errors;
- any non-zero recorded acquisition-health counter delta.

If a capture contains no usable STATUS interval, that is reported as a warning rather than silently reported as a health PASS.

MAD and half-capture drift are intentionally **reported but not thresholded yet**. Rangeweave does not currently have enough physical board data to justify an arbitrary universal stability cutoff. The first real calibration captures should establish what normal stationary values look like before a default threshold is frozen.

Inspect any existing stationary capture with:

```powershell
py host/python/inspect_tof_calibration_capture.py <capture>
```

Add the producer-native diagnostic grids with:

```powershell
py host/python/inspect_tof_calibration_capture.py <capture> --show-grids
```

The command returns a non-zero status when structural capture integrity fails.

## Example calibration session

A builder might use a roughly 700 x 700 mm board, set it securely on a support, and record several poses such as:

```text
Rx +15 deg, Ry   0 deg
Rx -15 deg, Ry   0 deg
Rx   0 deg, Ry +15 deg
Rx   0 deg, Ry -15 deg
Rx +12 deg, Ry +10 deg
```

The exact angles are not requirements. Measured values such as `+14.6 deg` are preferable to pretending a physical setup achieved exactly `+15 deg`.

At least one additional pose should be reserved as a held-out validation capture and not used to fit the profile.

A fronto-parallel capture is useful for diagnostics but, by itself, does not constrain the X/Y ray slopes.

## Optional, not prerequisite

The expected builder path is:

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
optional known-plane calibration
       |
       v
tof_geometry.json
```

Nothing in capture, replay, raw depth viewing, or nominal 3D projection should require a calibration artifact. A calibrated profile is an optional replacement for the fallback when a builder wants to dial in per-device geometry.

## Reference implementation

`host/python/rangeweave_tof_calibration_workflow.py` implements the physical-pose convention as `KnownPlanePose` and converts a 64-zone stationary observation into the solver's existing `CalibrationPlane` representation.

`KnownPlanePose.centre_on_optical_axis()` is only a convenience for the common `P = (0,0,D)` arrangement. It does not imply a centre pivot or constant `D`.

`host/python/rangeweave_tof_calibration_capture.py` implements the robust stationary-capture reduction and refuses to attach a structurally invalid capture to a known plane.

The workflow does not yet:

- define the multi-capture calibration manifest format;
- prescribe how orientation/point measurements are obtained;
- choose a mandatory target size or construction;
- run the full solver directly from a manifest;
- define evidence-based default MAD/drift stability limits;
- compare a physical calibrated profile against the ST nominal fallback on held-out captures.

Those are the next parts of the optional calibration workflow.
