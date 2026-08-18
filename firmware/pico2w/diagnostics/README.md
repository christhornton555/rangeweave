# Pico 2 W diagnostics

Run these in order when assembling or debugging the reference hardware:

1. `i2c_scan.py` - proves the devices ACK on the intended split buses and catches the LIS3MDL-at-0x1C wiring state.
2. `imu_bringup.py` - proves LSM6DSOX/LIS3MDL identity and readable physical values without the ToF workload.
3. `tof_bringup.py` - proves the firmware blob, VL53L5CX driver and 8x8 15-Hz mode independently.
4. `reproducible_sensor_stack.py` - full split-bus/FIFO/timestamp/clock/magnetometer/ToF self-test. A reference assembly should reach `SYSTEM READY: PASS` before higher-level capture or mapping is debugged.

The full diagnostic intentionally remains separate from future acquisition firmware. It is a reproducibility and hardware-isolation tool, not the high-throughput packet stream.

See [`docs/build-guide.md`](../../../docs/build-guide.md) and the [reference validation result](../../../docs/validation/reference-unit-v0.5.md).
