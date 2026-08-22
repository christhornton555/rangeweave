# VL53L5CX 64-zone ray geometry

**Status: nominal host geometry candidate. The producer-native wire format remains unchanged.**

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

## Geometry source

The current nominal model is named:

```text
vl53l5cx-st-plane-algo-2022-corrected-yaw
```

It is derived from the VL53L5CX pitch/yaw lookup table published by an ST employee in the ST Community thread *For VL53L5CX: what does ResultsData->distance_mm[ZoneNum] exactly mean?*. The post describes it as the code ST used for the plane/XYZ example.

Relevant source material:

- VL53L5CX datasheet DS13754 Rev 13: 8x8 ZoneID ordering and a nominal 45° horizontal × 45° vertical detection volume (65° diagonal);
- ST Community: https://community.st.com/mems-sensors-48/for-vl53l5cx-what-does-resultsdata-distance-mm-zonenum-exactly-mean-7106
- related XYZ thread: https://community.st.com/imaging-sensors-49/vl53l5cx-multi-zone-sensor-get-x-y-z-of-points-relative-to-origin-29010

The published yaw table contains one duplicated value in ZoneID 48. A later community reply identifies the symmetric value as 215.40° rather than 203.20°. A retired ST employee subsequently acknowledged that the published point-cloud numbers contained a copy error. Rangeweave uses the symmetry-corrected 215.40° value.

A later ST Community example also circulated a different VL53L5CX pitch table beginning at 59°. That post explicitly described its verification as minimal. Rangeweave does not silently choose that alternate table as the canonical model. The geometry API is kept replaceable so a better characterised or per-device model can supersede the current nominal slopes without changing capture semantics.

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

## Invalid and low-confidence samples

The current host geometry follows the existing analysis convention:

- finite `distance_mm > 0`: projectable;
- zero, negative or non-finite distance: invalid and represented as no point.

This is not yet the full VL53L5CX quality policy.

ST recommends checking `target_status`; for clean object edges, the ST guidance in the cited discussion is to accept statuses 5, 6 or 9 and reject status 12, which can represent weak lens/glare returns. Protocol v0.1 already reserves the `TOF_FIELD_TARGET_STATUS` field, but the current reference producer captures only distance and reflectance (`field_mask = 0x0003`).

Consequently the first point-cloud projection is geometrically useful but cannot yet perform ST status-based quality filtering. Adding target status to the reference producer is the natural next acquisition refinement.

## Code boundary

[`../host/python/rangeweave_geometry.py`](../host/python/rangeweave_geometry.py) is standard-library only and provides:

- producer ZoneID ↔ validated physical grid mapping;
- per-zone `(X/Z, Y/Z, 1)` projection vectors;
- normalized unit rays for later geometry algorithms;
- one-zone axial-distance projection;
- full 64-zone projection preserving invalid zones as `None`.

[`../host/python/view_point_cloud.py`](../host/python/view_point_cloud.py) is a thin optional-Matplotlib frontend for one frame. It does not alter capture or analysis semantics.

## Not calibrated yet

This model is a nominal optical mapping, not a per-unit calibration. It does not yet estimate or correct:

- per-device zone direction error;
- exact optical-centre translation;
- range bias;
- cover-window/refraction effects;
- assembly extrinsics to IMU or magnetometer;
- motion during one 8x8 frame;
- world-frame pose.

Those remain later calibration/fusion layers above the same raw producer-native measurements.
