# Coordinate frames

**Status: names accepted; axis handedness/orientation must be frozen before point-projection code is merged.**

Named frames planned by the architecture:

- `tof_optical` - VL53L5CX optical/ray frame.
- `imu_sensor` - LSM6DSOX package measurement frame.
- `mag_sensor` - LIS3MDL package measurement frame.
- `device_body` - rigid assembled sensing-head frame.
- `world` / `local` - estimator/map frame.

Before implementing the first 64-point projection we must document, with a diagram and golden numeric examples:

1. handedness;
2. +X/+Y/+Z directions for every sensor board orientation;
3. quaternion convention and multiplication order;
4. transform notation (`T_A_B` meaning exactly what?);
5. distance units and angular units;
6. zone index ordering for the 8x8 depth array.

No platform-specific API convention (OpenGL, Android, Open3D, etc.) may silently become the project convention.
