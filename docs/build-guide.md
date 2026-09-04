# Rangeweave reference build guide

**Status: current builder/reproduction guide for the Pico 2 W breadboard reference stack.**

This guide is the shortest supported path from loose parts to a validated Rangeweave sensor stream. It deliberately separates **hardware self-test** from **acquisition/calibration** so that changes made for motion capture do not silently alter the reproduction baseline.

Git history and the files in `docs/validation/` are the validation record; old numbered prototype-guide revisions are historical only.

## 1. Reference hardware

| Item | Reference part | Notes |
| --- | --- | --- |
| MCU | Raspberry Pi Pico 2 W (RP2350) | MicroPython reference controller |
| ToF | Pimoroni VL53L5CX 8x8 breakout | 64 zones, reference mode 15 Hz |
| IMU + magnetometer | Adafruit LSM6DSOX + LIS3MDL Precision 9 DoF breakout | accel/gyro + magnetometer |
| Prototype mounting | rigid breadboard/plate | ToF and IMU must not move relative to each other during motion/calibration |
| Host | PC with Python 3 + Thonny | flashing, capture, replay and analysis |

The project does not require an RGB camera.

## 2. Mechanical requirements

Before motion datasets or calibration:

- make the sensing head mechanically rigid;
- keep the VL53L5CX aperture unobstructed;
- do not add a cover window until its optical effects can be calibrated;
- mark the intended `device_body` axes on the assembly;
- keep magnets, speakers, motors and large steel/current-carrying parts away from the LIS3MDL where practical.

`device_body` is right-handed:

```text
+X = device right
+Y = device down
+Z = intended mechanical/ToF-forward direction
```

The reference rig's measured LSM6DSOX mapping is documented in [`coordinate-frames.md`](coordinate-frames.md). Do **not** copy that mapping blindly if your IMU is mounted differently.

## 3. Wiring

Use two physical I2C buses. The split-bus design is part of the validated reference architecture.

### Power

- Pico `3V3(OUT)` -> sensor 3.3 V rail;
- Pico GND -> common sensor ground;
- 3.3 V rail -> Adafruit VIN and VL53L5CX VCC;
- common GND -> both breakouts.

Do not power motors/servos from the Pico 3.3 V rail.

### I2C0: LSM6DSOX + LIS3MDL

| Pico | Adafruit board | Reference use |
| --- | --- | --- |
| GP4 | SDA | I2C0 SDA |
| GP5 | SCL | I2C0 SCL |
| 3V3 | ADM | required: forces LIS3MDL to `0x1E` |
| leave default/open | ADAG | reference LSM6DSOX address `0x6A` |
| unconnected | INT1/INT2/DRDY/INTM | not required by current reference firmware |

I2C0 speed: **400 kHz**.

Expected identities:

```text
LSM6DSOX address 0x6A, WHO_AM_I 0x6C
LIS3MDL  address 0x1E, WHO_AM_I 0x3D
```

If LIS3MDL appears at `0x1C`, fix `ADM -> 3V3` before continuing.

### I2C1: VL53L5CX

| Pico | VL53L5CX | Reference use |
| --- | --- | --- |
| GP2 | SDA | I2C1 SDA |
| GP3 | SCL | I2C1 SCL |
| 3V3 | VCC | power |
| GND | GND | common ground |

I2C1 speed: **1 MHz**.

Expected address: `0x29`.

See [`../hardware/wiring/pico2w-reference.md`](../hardware/wiring/pico2w-reference.md) for the compact wiring sheet.

## 4. MicroPython and VL53L5CX firmware

Use a Pimoroni RP2350/Pico 2 W MicroPython build that includes the VL53L5CX binding used by the project. Record the exact runtime for reproduction reports:

```python
import os, sys
print(os.uname())
print(sys.implementation)
```

The VL53L5CX internal firmware blob must exist on the Pico as:

```text
/vl53l5cx_firmware.bin
```

The repository does not vendor the third-party binary. See [`../tools/fetch_vl53l5cx_firmware.py`](../tools/fetch_vl53l5cx_firmware.py) and [`../THIRD_PARTY_NOTICES.md`](../THIRD_PARTY_NOTICES.md).

Initial VL53L5CX construction uploads this firmware over I2C and can take several seconds.

## 5. Bring-up order

Do not begin with the acquisition streamer. Bring up one layer at a time:

1. `firmware/pico2w/diagnostics/i2c_scan.py`
2. `firmware/pico2w/diagnostics/imu_bringup.py`
3. `firmware/pico2w/diagnostics/tof_bringup.py`
4. `firmware/pico2w/diagnostics/reproducible_sensor_stack.py`

The first pass condition is:

```text
SYSTEM READY: PASS
```

The diagnostic verifies the known device identities, FIFO/timestamp structure, per-unit timing behaviour, magnetometer acquisition and ToF operation. A second healthy LSM6DSOX does **not** need to have exactly the same measured ODR as the first reference unit; the architecture measures timing per device.

The published reference self-test evidence is in [`validation/reference-unit-v0.5.md`](validation/reference-unit-v0.5.md).

## 6. Switch from diagnostic to acquisition firmware

Once the self-test passes, use the binary producer in:

```text
firmware/pico2w/acquisition/
```

Copy these files to the Pico root:

```text
rw_protocol.py
rw_sensors.py
rw_timing.py
rw_transport_usb.py
main.py
```

Copy `main.py` **last**. Once it starts, stdout becomes the binary Rangeweave stream, so readable Thonny output is no longer expected.

The acquisition firmware and frozen diagnostic serve different purposes and need not use every identical sensor register value. In particular, the current acquisition stream uses:

```text
LSM6DSOX accel: 104 Hz, +/-4 g     (CTRL1_XL = 0x48)
LSM6DSOX gyro:  104 Hz, +/-500 dps (CTRL2_G = 0x44)
VL53L5CX:       8x8, 15 Hz
```

The acquisition gyro was increased from +/-250 to +/-500 dps after a real mixed-axis calibration movement exceeded the old full scale. `STREAM_INFO` records the actual runtime configuration; host tools read it rather than assuming scale.

See [`../firmware/pico2w/acquisition/README.md`](../firmware/pico2w/acquisition/README.md).

## 7. PC setup

From a normal Python environment:

```powershell
py -m pip install pyserial
```

Use the repository root as the working directory.

### Smoke probe

Replace `COM5` as required:

```powershell
py host/python/probe_serial.py COM5 --warmup 3 --seconds 15 --output packets.bin
```

Healthy measured-window expectations:

- valid frames received;
- `bad frames = 0`;
- `seq gaps = 0`;
- `IMU_BATCH`, `MAG`, `TOF_GRID`, `CLOCK_SYNC`, `STATUS`, `STREAM_INFO` all present;
- drop/FIFO/sensor/clock-sync health-counter deltas remain zero.

The exact record counts are sanity checks rather than protocol requirements.

### Canonical capture

```powershell
py host/python/capture.py COM5 --seconds 10 --name first-capture
```

A capture directory contains the raw stream plus metadata/provenance information. Keep raw captures when they support calibration or validation so later code can replay them.

`capture.py` defaults to a 3-second warm-up that is **discarded before recording begins**. This matters for manual motion experiments; do not start moving during that warm-up if the recorded capture must contain an initial stationary pose.

## 8. Inspect the captured data

Useful tools include:

```powershell
py host/python/analyze_capture.py <packets.bin>
py host/python/inspect_tof_calibration_capture.py <capture-dir>
py host/python/inspect_relative_rotation.py <capture-dir>
```

The raw/temporal viewers and point-cloud tools are documented under `docs/`.

## 9. Calibration after hardware reproduction

Do not treat "calibration" as one monolithic procedure.

### Optional ToF intrinsic/ray calibration

Refines per-zone geometry inside `tof_optical`. It uses known measured planes and is documented in [`tof-calibration-plane-workflow.md`](tof-calibration-plane-workflow.md).

### ToF/body boresight calibration

Estimates the rigid rotation between the ToF optical frame and the assembled `device_body` frame.

Recommended physical setup:

- a flat, clear, preferably matt wall;
- P0 approximately **500 mm** from the centre of at least a **1 m x 1 m** unobstructed wall patch;
- sensing head held on a stable 3-axis mount;
- moderate multi-axis movements with clean stationary endpoints;
- enough cable slack that USB/sensor leads do not pull the head as it settles.

A low-cost reference-style fixture can be made with a 3-way photographic head, an upper MDF carrier screwed to the camera screw, the breadboard clamped to that carrier, and a lower MDF/base support attached using the mount's own threaded connector and a support/selfie-stick clamp. This exact arrangement is not required: the important properties are rigid ToF-to-IMU mounting, useful multi-axis adjustment and stable hands-off holds.

Exact wall distance and exact commanded pose angles are not measured solver inputs.

Run the standard full P0-P5 workflow, replacing `COM5` as needed:

```powershell
py host/python/guided_boresight.py COM5
```

The guided command owns the 3 s discarded warm-up, 5 s recorded initial hold, 10 s movement allowance and 12 s hands-off final hold. It displays explicit HOLD / MOVE NOW / STOP MOVING cues and automatically applies minimum-motion, gyro full-scale, gravity-closure, plane-residual and temporal-drift gates.

P5 is the validation pose. After the successful session completes, generate the per-device artifact:

```powershell
py host/python/generate_boresight_artifact.py <session-prefix>
```

The generator fits P0-P4 first, predicts P5 using M5's independently measured IMU orientation, compares that prediction with the held-out P5 ToF plane, refits all six accepted poses, and writes:

```text
calibration/tof-body-rotation-<session-prefix>.json
```

The artifact embeds the exact rotation matrix, validation/fit diagnostics, capture SHA-256 hashes, `STREAM_INFO`, IMU mapping role, gyro-range provenance, quality gates and ToF geometry-profile role.

See [`boresight-calibration.md`](boresight-calibration.md) for the detailed procedure and [`validation/boresight-reference-rig-2026-09.md`](validation/boresight-reference-rig-2026-09.md) for the physical reference result.

## 10. Reference-rig calibration facts versus universal requirements

The current breadboard reference rig has physically established:

```text
body X = -imu X
body Y = -imu Z
body Z = -imu Y
```

That is **not** a universal wiring/property of the LSM6DSOX. It depends on how the board is mounted.

The September 2026 reference assembly also completed P0-P5 boresight validation. Its six-pose result was approximately:

```text
R_body_from_tof: Rx +5.880  Ry +3.930  Rz +0.160 deg
normal RMS:      0.797 deg
held-out P5 error before refit: 0.644 deg
```

Those values are evidence that the calibration method works on that assembly, **not** calibration constants to copy into another build. Reproduce the physical mapping/calibration for a differently assembled unit.

## 11. Recovery / returning to Thonny

Because acquisition `main.py` starts automatically and emits binary data, interrupt it with Ctrl-C from Thonny/serial when practical. If that becomes awkward, BOOTSEL/reflash the known-good MicroPython image and restore the firmware files deliberately.

Keep a local copy of:

- the exact MicroPython/UF2 version used;
- `vl53l5cx_firmware.bin` provenance/source;
- the repository commit/tag;
- raw validation/calibration captures;
- generated calibration artifacts for that physical unit.

## 12. What this build does not yet prove

A passing sensor stack/acquisition stream does **not** by itself prove:

- correct per-device ToF/body alignment without running boresight calibration;
- magnetometer/world-frame calibration;
- reliable freehand 6DoF tracking;
- odometry/loop closure;
- dense or metrically accurate 3D reconstruction;
- identical calibration across arbitrary third-party assemblies.

Those capabilities must each earn their own physical validation evidence.
