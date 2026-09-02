# Coordinate frames

**Status:** `tof_optical` axes/zone orientation, nominal ToF projection geometry, `device_body` axes and the rotational `tof_optical -> device_body` contract are frozen. The LSM6DSOX package mapping and short-baseline relative-body rotation have been physically validated on the current reference rig. `mag_sensor -> device_body`, rigid translations and world/local-frame conventions remain open.

Named frames:

- `tof_optical` - VL53L5CX optical/ray frame;
- `imu_sensor` - LSM6DSOX package measurement frame;
- `mag_sensor` - LIS3MDL package measurement frame;
- `device_body` - rigid assembled sensing-head frame;
- `world` / `local` - future estimator/map frame.

## `tof_optical`

`tof_optical` is right-handed:

- origin: nominal optical centre of the VL53L5CX sensing aperture;
- `+X`: scene/image right when standing behind the sensor and looking forward;
- `+Y`: image down;
- `+Z`: forward from the sensor into the scene.

Therefore `+X x +Y = +Z`.

Geometric distances and Cartesian coordinates in the reference host layer are in **millimetres** unless an interface explicitly states otherwise.

## `device_body`

`device_body` is a separate right-handed frame attached to the completed sensing head:

- `+X`: intended device right;
- `+Y`: intended device down;
- `+Z`: intended mechanical forward direction.

The nominal design makes these axes parallel to `tof_optical`, but a real assembly is not assumed to achieve that exactly.

The v1 rotation contract is:

```text
v_body = R_body_from_tof * v_tof
```

and is implemented by `host/python/rangeweave_extrinsics.py`.

A common assembly rotation belongs in this rigid extrinsic. It must not be hidden inside the 64 independent ToF zone slopes.

## `imu_sensor -> device_body` on the reference rig

Physical axis tests on the current rigid breadboard reference assembly established:

```text
device_body +X -> imu_sensor -X
device_body +Y -> imu_sensor -Z
device_body +Z -> imu_sensor -Y
```

Therefore the current reference-host mapping is:

```text
body X = -imu X
body Y = -imu Z
body Z = -imu Y
```

The corresponding 3x3 mapping is a proper rotation with determinant +1.

This mapping is **assembly-specific**. It is valid for the physically tested reference mounting; it is not a universal property of the LSM6DSOX package or Adafruit breakout. Any builder who mounts the IMU differently must establish the correct `imu_sensor -> device_body` transform for that build.

The reference implementation uses this mapping for short-baseline calibration motions. The relative-rotation estimator has been physically checked on simple and compound movements with sub-degree independent gravity closure under the current +/-500 deg/s acquisition configuration.

See [`tof-body-extrinsics.md`](tof-body-extrinsics.md) and [`boresight-calibration.md`](boresight-calibration.md).

## Producer-native versus physical ToF zone indices

Protocol v0.1 `layout_id = 0` remains producer-native flattened zone order. Raw `TOF_GRID` records are never rewritten into physical/image order.

Four-corner physical validation established:

```text
physical upper-left   -> producer r07c07
physical upper-right  -> producer r07c00
physical lower-left   -> producer r00c07
physical lower-right  -> producer r00c00
```

For an 8x8 grid, define physical/image indices `p_row`, `p_col` with row 0 at the physical top and column 0 at physical left:

```text
p_row = 7 - r
p_col = 7 - c
```

The inverse is identical:

```text
r = 7 - p_row
c = 7 - p_col
```

With producer flattened index `n = 8*r + c`, physical row-major index is:

```text
p = 8*p_row + p_col = 63 - n
```

This is a pure 180-degree rotation, not a transpose or a single-axis mirror.

Golden examples:

```text
producer r00c00 / n=0   -> physical row 7, col 7 / p=63
producer r00c07 / n=7   -> physical row 7, col 0 / p=56
producer r07c00 / n=56  -> physical row 0, col 7 / p=7
producer r07c07 / n=63  -> physical row 0, col 0 / p=0
producer r03c03 / n=27  -> physical row 4, col 4 / p=36
producer r04c04 / n=36  -> physical row 3, col 3 / p=27
```

Because the grid is even-sized, no single zone lies exactly on the optical axis; the nominal centre falls between the four central zones.

See [`tof-zone-orientation.md`](tof-zone-orientation.md).

## Nominal 64-zone projection fallback

The built-in VL53L5CX ray profile is documented in [`tof-ray-geometry.md`](tof-ray-geometry.md) and implemented by `host/python/rangeweave_geometry.py`.

Rangeweave treats VL53L5CX `distance_mm` as **axial/perpendicular Z distance**, not slant range. For each producer zone the profile stores `(x_per_z, y_per_z, 1)` and projects:

```text
X = x_per_z * distance_mm
Y = y_per_z * distance_mm
Z = distance_mm
```

Do not normalize that vector and multiply by `distance_mm`; that would reinterpret axial range as slant range.

The built-in slopes are a **nominal fallback for uncalibrated systems** derived from ST's published plane/XYZ lookup data, with the documented duplicated-yaw table error corrected for this project profile. A measured per-device profile may legitimately be asymmetric or differ zone-by-zone.

Golden one-metre axial-wall examples for the fallback include:

```text
producer ZoneID 0  -> (+362.624, +362.624, 1000) mm
producer ZoneID 7  -> (-362.624, +362.624, 1000) mm
producer ZoneID 56 -> (+362.624, -362.624, 1000) mm
producer ZoneID 63 -> (-362.624, -362.624, 1000) mm
```

All synthetic wall points retain `Z = 1000 mm` exactly.

## ToF/body boresight

The fixed-plane solver estimates `R_body_from_tof` from ToF plane normals and relative body rotations while treating the fixed wall's absolute room normal as a nuisance parameter.

A four-pose physical sequence on the current reference rig produced a small provisional boresight with ~0.535 deg normal RMS. The result is not yet a promoted artifact because final held-out validation under the newer +/-500 deg/s gyro configuration remains to be completed.

For current builder/setup guidance, see [`boresight-calibration.md`](boresight-calibration.md). The recommended boresight target is a clear flat wall patch, P0 approximately 500 mm from the wall; that distance is setup guidance rather than a frame-definition constant.

## Still to freeze / calibrate

Before full IMU/ToF fusion or world-frame reconstruction is considered stable, Rangeweave still needs to document and validate:

1. `mag_sensor -> device_body` on the physical build;
2. quaternion component order and active/passive/multiplication conventions for the future attitude layer;
3. any broader transform notation used for rigid 6DoF transforms;
4. final held-out per-device `R_body_from_tof` calibration and provenance;
5. portable per-device VL53L5CX intrinsic/ray profiles and exact optical origin where required;
6. rigid translations between ToF, IMU, magnetometer and body origins where applications require them;
7. `world` / `local` frame initialization and update conventions.

No platform-specific API convention may silently become the project convention.
