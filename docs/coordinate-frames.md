# Coordinate frames

**Status: `tof_optical` zone orientation and axes frozen; IMU/magnetometer/body/world transform conventions remain to be frozen before cross-sensor fusion.**

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

## Still to freeze

Before IMU/ToF fusion or world-frame reconstruction is merged, we must still document, with diagrams and golden numeric examples:

1. `imu_sensor` package axes in the actual board mounting;
2. `mag_sensor` package axes in the actual board mounting;
3. `device_body` axes;
4. quaternion component order, active/passive interpretation and multiplication order;
5. transform notation (`T_A_B` meaning exactly what?);
6. units for all geometric/calibration quantities;
7. calibrated VL53L5CX zone-ray directions and the exact optical origin;
8. rigid extrinsics between `tof_optical`, `imu_sensor`, `mag_sensor` and `device_body`.

No platform-specific API convention may silently become the project convention.