# Calibration model

Calibration data should be explicit, versioned and kept separate from runtime estimator state. Rangeweave also keeps **sensor intrinsics** separate from **assembly extrinsics**: a common ToF/package rotation must not be hidden inside the 64 per-zone ray directions.

## Categories

### Factory / per-unit timing

Discovered automatically:

- LSM6DSOX `INTERNAL_FREQ_FINE`;
- factory-estimated timestamp tick / ODR;
- observed FIFO timestamp spacing;
- measured sensor-clock <-> MCU-clock model.

### Sensor intrinsics

Planned/partially characterized:

- accelerometer bias/scale/cross-axis behaviour;
- gyroscope zero-rate bias and temperature behaviour;
- LIS3MDL hard-iron and soft-iron calibration;
- VL53L5CX per-zone ray directions, range bias, noise/invalid-rate characteristics and any cover-window crosstalk calibration.

The optional VL53L5CX per-zone ray-calibration path uses known physical planes and is documented in [`tof-calibration-plane-workflow.md`](tof-calibration-plane-workflow.md) and [`tof-geometry-calibration.md`](tof-geometry-calibration.md). That workflow may use a measured calibration board or another known plane because its purpose is to estimate intrinsic zone geometry.

### Assembly extrinsics

Once the sensing head is rigidly mounted:

- rotational transform `tof_optical -> device_body`;
- physical mapping/rotation `imu_sensor -> device_body`;
- later `mag_sensor -> device_body`;
- later physical sensor-origin offsets / rigid translations.

The current reference rig has physically validated its LSM6DSOX package-axis mapping and a short-baseline relative-body rotation estimator. The ToF/body rotational boresight path has also produced a plausible provisional physical solve.

For **ToF/body boresight**, the recommended target is now a fixed flat wall: P0 approximately 500 mm from the centre of a clear ~1 m x 1 m wall area. Exact wall distance and exact square alignment are not calibration measurements; they simply provide field-of-view margin for later poses. See [`boresight-calibration.md`](boresight-calibration.md).

The current acquisition firmware uses LSM6DSOX gyro `CTRL2_G = 0x44` (104 Hz, +/-500 deg/s). The previous +/-250 deg/s setting was exceeded during a real mixed-axis calibration motion. Host tooling now checks gyro full-scale utilisation directly.

Current boresight-specific empirical gates are:

```text
relative motion >= 5 deg
gyro warning at 80% full scale
gyro rejection at 90% full scale
gyro/gravity closure <= 2 deg
ToF plane RMS <= 10 mm
ToF maximum plane residual <= 30 mm
ToF maximum half-capture drift <= 10 mm
```

These are workflow quality gates derived from reference-rig testing, not universal sensor specifications.

### Runtime estimator state

Must not be stored as permanent calibration merely because it changes slowly:

- current pose;
- pose covariance/uncertainty;
- online gyro-bias estimate;
- current magnetic confidence;
- local map/submap state.

## Reference bench work

The [build guide](build-guide.md) remains the hardware reproduction/self-test starting point. For current alignment work, use the dedicated [boresight calibration guide](boresight-calibration.md).

Raw calibration observations should be retained so algorithms can be rerun later. A promoted calibration artifact should carry provenance identifying the capture set, firmware/configuration, geometry profile and fit diagnostics used to produce it.

Physical reference-rig evidence is summarized in [`validation/boresight-reference-rig-2026-09.md`](validation/boresight-reference-rig-2026-09.md).
