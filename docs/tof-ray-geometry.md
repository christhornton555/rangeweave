# VL53L5CX 64-zone ray geometry

**Status: physically exercised nominal host geometry fallback. The producer-native wire format remains unchanged. Exact per-device ray directions are not yet calibrated.**

This document defines the first Rangeweave mapping from one VL53L5CX 8x8 distance frame to 64 XYZ points in the frozen [`tof_optical`](coordinate-frames.md) frame.

## The important distance convention

VL53L5CX `distance_mm` is not a slant/hypotenuse range suitable for direct multiplication by a unit ray.

ST states that the device converts its radial measurement to a perpendicular distance internally: a perpendicular wall at 1 m should therefore report approximately 1 m in every zone. In the XYZ example published by ST, the reported `distance_mm` is assigned directly to Z and X/Y are reconstructed from the zone geometry.

Rangeweave therefore uses:

```text
Z = distance_mm
X = x_per_z[zone] * Z
Y = y_per_z[zone] * Z
```

A normalized unit ray is still useful for later geometric operations, but this is deliberately **not** the projection rule:

```text
wrong for VL53L5CX output:
point = distance_mm * unit_ray
```

That incorrect rule would make an axial flat wall bow toward the sensor at the edges.

## Geometry source and fallback role

The built-in model is named:

```text
vl53l5cx-st-plane-algo-2022-corrected-yaw
```

Its role in Rangeweave is explicitly:

```text
nominal fallback profile
```

It exists so an uncalibrated Rangeweave build can produce useful XYZ geometry immediately. It is **not** treated as the calibrated optical truth for every VL53L5CX, breakout board, cover window, or assembled Rangeweave unit.

The profile is derived from the VL53L5CX pitch/yaw lookup table published by an ST employee in the ST Community thread *For VL53L5CX: what does ResultsData->distance_mm[ZoneNum] exactly mean?*. The post describes it as the code ST used for the plane/XYZ example.

Relevant source material:

- VL53L5CX datasheet DS13754 Rev 13: 8x8 ZoneID ordering and a nominal 45° horizontal × 45° vertical detection volume (65° diagonal);
- ST Community: https://community.st.com/mems-sensors-48/for-vl53l5cx-what-does-resultsdata-distance-mm-zonenum-exactly-mean-7106
- related XYZ thread: https://community.st.com/imaging-sensors-49/vl53l5cx-multi-zone-sensor-get-x-y-z-of-points-relative-to-origin-29010

The published yaw table contains one duplicated value in ZoneID 48. A later community reply identifies the symmetric value as 215.40° rather than 203.20°. A retired ST employee subsequently acknowledged that the published point-cloud numbers contained a copy error. Rangeweave uses the symmetry-corrected 215.40° value for this **nominal ST fallback profile**.

A later ST Community example also circulated a different VL53L5CX pitch table beginning at 59°. That post explicitly described its verification as minimal. Rangeweave does not silently choose that alternate table as the canonical model.

Most importantly, Rangeweave does not impose the symmetry or curvature of either published table on calibrated systems. A measured per-device profile may legitimately bow inward, bow outward, be asymmetric, or contain zone-by-zone deviations from the nominal ST profile. Those differences are measurements to preserve, not errors to force back toward the fallback LUT.

## Coordinate conversion

The ST example uses an angular convention viewed from the device/lens side. Rangeweave `tof_optical` is instead defined while standing behind the sensor and looking forward into the scene:

```text
+X = scene right
+Y = image down
+Z = forward
```

That changes the horizontal sign relative to the ST lens-side example.

Rather than expose ST's historical pitch/yaw naming, `rangeweave_geometry.py` stores the derived producer-ZoneID slope pair:

```text
(x_per_z, y_per_z)
```

and defines the projection vector:

```text
(x_per_z, y_per_z, 1)
```

The raw ZoneID is never reordered. The previously validated upright grid mapping remains:

```text
physical_row = 7 - producer_row
physical_col = 7 - producer_col
```

## Golden examples

For a synthetic perpendicular wall where every zone reports `distance_mm = 1000`:

```text
ZoneID 0  / physical lower-right -> (+362.624, +362.624, 1000) mm
ZoneID 7  / physical lower-left  -> (-362.624, +362.624, 1000) mm
ZoneID 56 / physical upper-right -> (+362.624, -362.624, 1000) mm
ZoneID 63 / physical upper-left  -> (-362.624, -362.624, 1000) mm
```

The four central zones are approximately:

```text
ZoneID 27 -> (+49.446, +49.446, 1000) mm
ZoneID 28 -> (-49.446, +49.446, 1000) mm
ZoneID 35 -> (+49.446, -49.446, 1000) mm
ZoneID 36 -> (-49.446, -49.446, 1000) mm
```

All 64 points retain exactly `Z = 1000 mm`. This flat-Z invariant is covered by CI.

## Physical validation performed for PR #8

The nominal fallback has now been exercised against real Rangeweave captures. These tests validate the projection pipeline and expose the limits of the nominal LUT; they do **not** constitute per-device ray calibration.

### Flat wall

Capture:

```text
capture_20260821_004124Z_flat-wall-1000mm
```

The final frame projected 64/64 valid points with:

```text
X: -331.1 .. +344.9 mm
Y: -344.9 .. +333.3 mm
Z:  871.0 .. 951.0 mm
```

The 3D projection formed a coherent tilted sheet rather than a spherical/bowled depth surface. That supports the axial-Z interpretation of `distance_mm`, the producer-zone ordering, and the basic projection pipeline. The Z gradient is consistent with the wall/sensor not being perfectly fronto-parallel in this informal capture.

### Thin diagonal plywood foreground

Capture:

```text
capture_20260822_234124Z_pointcloud-close-object
```

The final frame projected 64/64 valid points with:

```text
X: -320.6 .. +316.6 mm
Y: -320.6 .. +316.6 mm
Z:  291.0 .. 885.0 mm
```

A long, straight, thin plywood strip held diagonally at roughly 300 mm was clearly separated from a wall around 850-900 mm. The test exercised a hard foreground/background depth discontinuity and exposed fictitious long mesh connections in the diagnostic viewer; the viewer now suppresses row/column links when neighbouring points exceed a configurable Z discontinuity threshold.

The front-on view also made the nominal ST lattice curvature visible, especially around the wall points near the image edges. That lateral X/Y curvature is generated by the fallback `(X/Z, Y/Z)` LUT. It is therefore **not evidence that every physical Rangeweave sensor has that exact inward bow**. The observation is one reason Rangeweave will support per-device geometry calibration rather than treating the ST table as canonical calibrated geometry.

## Invalid and low-confidence samples

The current host geometry follows the existing analysis convention:

- finite `distance_mm > 0`: projectable;
- zero, negative or non-finite distance: invalid and represented as no point.

This is not yet the full VL53L5CX quality policy.

ST recommends checking `target_status`; for clean object edges, the ST guidance in the cited discussion is to accept statuses 5, 6 or 9 and reject status 12, which can represent weak lens/glare returns. Protocol v0.1 already reserves the `TOF_FIELD_TARGET_STATUS` field, but the current reference producer captures only distance and reflectance (`field_mask = 0x0003`).

Consequently the first point-cloud projection is geometrically useful but cannot yet perform ST status-based quality filtering. Adding target status to the reference producer is a later acquisition refinement.

## Point-cloud viewer policy

[`../host/python/view_point_cloud.py`](../host/python/view_point_cloud.py) is a diagnostic frontend, not a meshing algorithm.

A real foreground/background boundary can put neighbouring 8x8 zones hundreds of millimetres apart in Z. Connecting those neighbours unconditionally draws fictitious surfaces across empty space. The viewer therefore only draws a row/column mesh edge while the neighbouring projected points differ by no more than a configurable axial-depth threshold.

The default is:

```text
max_link_dz_mm = 150
```

Override it for a particular scene with:

```powershell
py host/python/view_point_cloud.py <capture> --max-link-dz-mm 100
```

This threshold affects presentation only. It does not remove, alter, interpolate or reclassify any projected point.

The graphical figure contains two complementary views:

1. a perspective 3D XYZ view whose grid lines break at invalid zones and large Z discontinuities;
2. a front-on X/Y view coloured by axial Z.

The front-on plot deliberately displays `+Y` downward so its image orientation matches the frozen `tof_optical` scene convention and the physically validated ToF presentation.

This front-on diagnostic is useful for checking physical feature orientation and for distinguishing measured Z structure from the lateral sampling lattice implied by the active geometry profile.

## Calibration philosophy

Rangeweave defines the coordinate frame and the calibration contract; it does not define what shape a particular sensor's 64-ray field must have.

A future calibrated geometry profile should therefore store all 64 zone directions independently. Calibration must not require:

```text
left = -right
top = -bottom
inward bow
outward bow
uniform spacing
```

Symmetry and similarity to the ST fallback profile are diagnostics only. A valid calibration may disagree with the fallback in any direction if the measurements support it.

The intended architecture is:

```text
raw producer-native TOF_GRID
        ↓
geometry profile
   ├─ nominal ST fallback when uncalibrated
   └─ measured per-device profile when available
        ↓
metric tof_optical XYZ
```

Optional image rectification may later provide a visually regular grid for human viewing, but rectification must remain separate from the metric geometry used for reconstruction.

## Code boundary

[`../host/python/rangeweave_geometry.py`](../host/python/rangeweave_geometry.py) is standard-library only and currently provides:

- producer ZoneID ↔ validated physical grid mapping;
- the built-in nominal fallback `(X/Z, Y/Z, 1)` profile;
- normalized unit rays for later geometry algorithms;
- one-zone axial-distance projection;
- full 64-zone projection preserving invalid zones as `None`.

[`../host/python/view_point_cloud.py`](../host/python/view_point_cloud.py) is a thin optional-Matplotlib frontend for one frame. It does not alter capture or analysis semantics.

A later increment will turn the geometry profile into a portable per-device calibration artifact rather than making downstream code depend directly on the built-in fallback table.

## Not calibrated yet

The built-in profile is a nominal optical fallback, not a per-unit calibration. It does not yet estimate or correct:

- per-device zone direction error;
- exact optical-centre translation;
- range bias;
- cover-window/refraction effects;
- assembly extrinsics to IMU or magnetometer;
- motion during one 8x8 frame;
- world-frame pose.

Those remain later calibration/fusion layers above the same raw producer-native measurements.
