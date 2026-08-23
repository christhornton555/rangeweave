# Coordinate frames

**Status: `tof_optical` zone orientation and axes frozen; a nominal fallback 64-zone projection profile is defined; IMU/magnetometer/body/world transform conventions remain to be frozen before cross-sensor fusion.**

Named frames planned by the architecture:

- `tof_optical` - VL53L5CX optical/ray frame.
- `imu_sensor` - LSM6DSOX package measurement frame.
- `mag_sensor` - LIS3MDL package measurement frame.
- `device_body` - rigid assembled sensing-head frame.
- `world` / `local` - estimator/map frame.

## `tof_optical`

The first physically validated frame is `tof_optical`.

It is a **right-handed** optical frame:

- origin: nominal optical centre of the VL53L5CX sensing aperture; the exact calibrated physical origin remains future calibration work;
- `+X`: right in the sensor image when standing behind the sensor and looking forward into the scene;
- `+Y`: down in that image;
- `+Z`: forward from the sensor into the scene.

Therefore `+X × +Y = +Z`.

This convention is project-owned. It is intentionally stated explicitly rather than inherited implicitly from OpenGL, Android, Open3D, ROS or any other platform API.

Geometric distances and Cartesian coordinates in the reference host layer are expressed in **millimetres** unless an interface explicitly states otherwise.

## Producer-native versus physical zone indices

Protocol v0.1 `layout_id = 0` remains producer-native flattened zone order. Raw `TOF_GRID` data is never rewritten into physical order.

A four-corner physical validation on 2026-08-22 established:

```text
physical upper-left   -> producer r07c07
physical upper-right  -> producer r07c00
physical lower-left   -> producer r00c07
physical lower-right  -> producer r00c00
```

For an 8x8 grid, define physical/image indices `p_row` and `p_col` such that:

- `p_row = 0` is the physical top and increases toward `+Y` (down);
- `p_col = 0` is the physical left and increases toward `+X` (right).

The mapping from producer-native indices `(r, c)` is:

```text
p_row = 7 - r
p_col = 7 - c
```

The inverse is identical:

```text
r = 7 - p_row
c = 7 - p_col
```

For producer-native flattened index:

```text
n = 8*r + c
```

and physical row-major flattened index:

```text
p = 8*p_row + p_col = 63 - n
```

This is a pure 180-degree rotation. It is not a transpose and not a single-axis mirror.

### Golden numeric examples

```text
producer r00c00 / n=0   -> physical row 7, col 7 / p=63  (lower-right)
producer r00c07 / n=7   -> physical row 7, col 0 / p=56  (lower-left)
producer r07c00 / n=56  -> physical row 0, col 7 / p=7   (upper-right)
producer r07c07 / n=63  -> physical row 0, col 0 / p=0   (upper-left)
producer r03c03 / n=27  -> physical row 4, col 4 / p=36
producer r04c04 / n=36  -> physical row 3, col 3 / p=27
```

Because the grid has an even number of rows and columns, no single zone lies exactly on the optical axis; the nominal centre falls between the four central zones.

The measured experiment and rationale are recorded in [`tof-zone-orientation.md`](tof-zone-orientation.md).

## Nominal 64-zone projection fallback

The first built-in VL53L5CX ray/projection profile is documented in [`tof-ray-geometry.md`](tof-ray-geometry.md) and implemented by `host/python/rangeweave_geometry.py`.

A crucial device convention is explicit: VL53L5CX `distance_mm` is the **axial/perpendicular Z distance**, not slant range. For each producer ZoneID, Rangeweave stores a projective direction `(x_per_z, y_per_z, 1)` and computes:

```text
X = x_per_z * distance_mm
Y = y_per_z * distance_mm
Z = distance_mm
```

The built-in slopes are derived from ST's published VL53L5CX plane/XYZ lookup table, with the documented duplicated-yaw copy error corrected for that profile. Their architectural role is a **nominal fallback for uncalibrated systems**. Raw captures do not depend on them, and downstream Rangeweave geometry must be able to substitute a measured per-device profile without changing capture semantics.

Rangeweave does not require calibrated zone directions to preserve the ST table's symmetry, spacing or curvature. A valid measured profile may bow inward, bow outward, be asymmetric, or differ zone-by-zone if the calibration measurements support it.

A normalized unit ray may be derived from the same vector for algorithms that need direction only. It must **not** be multiplied directly by `distance_mm`, because that would reinterpret the device's axial range as a slant range.

Golden one-metre axial-wall examples for the built-in fallback include:

```text
producer ZoneID 0  -> (+362.624, +362.624, 1000) mm
producer ZoneID 7  -> (-362.624, +362.624, 1000) mm
producer ZoneID 56 -> (+362.624, -362.624, 1000) mm
producer ZoneID 63 -> (-362.624, -362.624, 1000) mm
```

All 64 synthetic wall points retain `Z = 1000 mm` exactly.

The fallback has been exercised against both a real flat-wall capture and a thin diagonal foreground object against a distant wall. Those tests validate the projection pipeline and depth convention. They do not elevate the ST-derived X/Y lattice to per-device calibrated truth; the front-on diagonal-object test visibly exposed the fallback lattice's inward edge curvature, motivating the generic calibration layer planned next.

## Still to freeze / calibrate

Before IMU/ToF fusion or world-frame reconstruction is merged, we must still document, with diagrams and golden numeric examples:

1. `imu_sensor` package axes in the actual board mounting;
2. `mag_sensor` package axes in the actual board mounting;
3. `device_body` axes;
4. quaternion component order, active/passive interpretation and multiplication order;
5. transform notation (`T_A_B` meaning exactly what?);
6. portable per-device VL53L5CX geometry profiles, calibration procedure, fit diagnostics and exact optical origin where required;
7. rigid extrinsics between `tof_optical`, `imu_sensor`, `mag_sensor` and `device_body`.

No platform-specific API convention may silently become the project convention.
