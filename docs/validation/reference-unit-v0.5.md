# Reference unit validation - diagnostic baseline v0.5

**Status: VALIDATED on one physical reference unit.**

The complete raw console output is checked in as [`pico2w-reference-run-v0.5.txt`](pico2w-reference-run-v0.5.txt).

## Startup self-test result

| Metric | Reference result |
|---|---:|
| LSM6DSOX address / ID | `0x6A` / `0x6C` |
| LIS3MDL address / ID | `0x1E` / `0x3D` |
| VL53L5CX address | `0x29` |
| `INTERNAL_FREQ_FINE` | `-20` (`0xEC`) |
| Factory-estimated real IMU ODR | `101.046720 Hz` |
| Measured clock-model ODR | `101.057830 Hz` |
| Measured clock-model tick | `25.769074 us` |
| Clock-fit RMS at self-test | `2.09 us` |
| Accel / gyro / timestamp records | `1213 / 1213 / 1213` |
| Timestamp delta | `384 / 384.000 / 384` ticks min/avg/max |
| Max FIFO backlog in startup window | `12` words |
| Magnetometer | `120 / 120`, 0 recovered retries, 0 drops |
| VL53L5CX | `179` frames, `14.907 fps`, 0 errors |
| Max ToF `get_data()` time | `28770 us` |
| Overall | **`SYSTEM READY: PASS`** |

After the startup window, the rolling clock model remained close to `25.769 us/tick` and `101.058 Hz`; accel/gyro/timestamp record counts remained aligned, magnetometer reports remained successful, and ToF acquisition stayed near 15 fps with no reported I/O errors during the supplied run.

## What another unit must reproduce

Another healthy build is **not expected to have `FREQ_FINE=-20` or run at ~101.058 Hz**. It should reproduce the invariants checked by the diagnostic:

- intended devices/identities on intended buses;
- no FIFO structural errors/overruns/full condition;
- aligned accel/gyro/timestamp streams;
- internally consistent timestamp spacing for the selected BDR relationship;
- usable positive Pico<->LSM clock model;
- reliable magnetometer acquisition;
- depth frame rate within the diagnostic pass threshold with no ToF I/O errors;
- final `SYSTEM READY: PASS`.

A second independently assembled unit should be added here when available; that will be stronger evidence of hardware reproducibility than repeating this exact unit's numeric values.
