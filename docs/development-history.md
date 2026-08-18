# Development history: how the reference architecture emerged

This is a curated engineering history, not a dump of every scratch script.

## 1. Individual sensor bring-up

The LSM6DSOX/LIS3MDL and VL53L5CX were first verified independently. Important corrections included:

- the actual magnetometer address state observed during early bring-up;
- LIS3MDL multi-byte reads requiring the sub-address auto-increment bit;
- VL53L5CX internal firmware upload before ranging;
- 8x8 operation targeting 15 Hz rather than incorrectly assuming 60 Hz for 64 zones.

## 2. Shared I2C bus rejected

Combining the full sensor workload on one physical bus produced poor timing/reliability. Full 8x8 VL53L5CX frame retrieval blocked MicroPython for roughly tens of milliseconds, while magnetometer communication became unreliable under the combined traffic.

The validated reference architecture therefore uses:

```text
I2C0 @ 400 kHz: LSM6DSOX + LIS3MDL
I2C1 @ 1 MHz:   VL53L5CX
```

## 3. Direct IMU polling rejected for time-critical acquisition

Even after splitting buses, a long ToF `get_data()` call could occupy the interpreter for roughly 19 ms and occasionally close to 29 ms. Polling accel/gyro registers from the host would therefore miss or duplicate physical samples even if the I2C transactions themselves succeeded.

## 4. LSM6DSOX FIFO adopted

Accel/gyro were moved into the LSM6DSOX hardware FIFO. This lets the sensor continue sampling while MicroPython is busy with a ToF transfer; the Pico drains the backlog later. Reference stress tests showed no FIFO overrun/full/error condition under the tested workload.

## 5. Hardware timestamps adopted

FIFO timestamp batching was enabled so time is defined by the sensor clock rather than by when Python happens to read a record. Accel, gyro and timestamp records were verified to remain slot-aligned.

## 6. The apparent “104 Hz vs 101 Hz” problem was explained

The nominal 104-Hz configuration did not produce 104 physical samples per wall-clock second on the first sensor. Reading `INTERNAL_FREQ_FINE` gave `-20`; the factory timing formula predicted approximately 101.047 Hz and FIFO timestamps independently measured approximately 101.04-101.06 Hz.

The architecture therefore **never hard-codes the real ODR**. It derives a unit-specific factory estimate and fits the LSM timestamp clock against Pico monotonic time.

## 7. LIS3MDL address made deterministic

The magnetometer had shown intermittent communication trouble during development. The Adafruit `ADM` address-select input was subsequently tied directly to 3.3 V, making LIS3MDL consistently appear at `0x1E`. The full timestamp/FIFO workload then produced stable 10-Hz host acquisition in the tested runs.

This does not prove every historical NACK had one root cause, but deterministic address wiring became part of the reference build because it removed a variable and produced a stable practical result.

## 8. v0.5 reproducibility self-test

The final diagnostic refactor stopped treating the reference unit's oscillator as universal. It discovers each unit's timing, fits a rolling clock model and checks structural invariants rather than exact reference numbers.

The old `test04`-`test08` scratch scripts were useful for diagnosis but represent rejected or superseded architectures. They are intentionally not committed to the builder-facing tree. From the first public repository commit onward, Git/ADRs/issues should preserve this history instead.
