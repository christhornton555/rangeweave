# Calibration model

Calibration data should be explicit, versioned and kept separate from runtime estimator state.

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

### Assembly extrinsics

Once the sensing head is rigidly mounted:

- rigid transform `tof_optical -> device_body`;
- rigid transform `imu_sensor -> device_body`;
- rigid transform `mag_sensor -> device_body`;
- physical sensor origin offsets.

### Runtime estimator state

Must not be stored as permanent calibration merely because it changes slowly:

- current pose;
- pose covariance/uncertainty;
- online gyro-bias estimate;
- current magnetic confidence;
- local map/submap state.

## Reference bench work

The build guide contains the current stationary IMU, magnetometer rotation and flat-wall ToF characterization procedures. Raw calibration observations should be retained so algorithms can be rerun later.
