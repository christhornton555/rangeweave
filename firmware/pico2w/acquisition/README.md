# Pico 2 W acquisition firmware

**Status: hardware-validated Rangeweave v0.1 acquisition producer on the current reference stack.**

This directory is separate from [`../diagnostics/reproducible_sensor_stack.py`](../diagnostics/reproducible_sensor_stack.py). The diagnostic is the builder/reproduction self-test; this acquisition firmware emits the binary [Rangeweave protocol v0.1](../../../protocol/spec-v0.1.md) stream consumed by host capture/replay tools.

The protocol/packetizer path passes host-side conformance tests, and physical reference runs have demonstrated lossless steady-state USB streaming with zero measured sensor/FIFO/sequence/drop errors and VL53L5CX acquisition close to the configured 15 Hz.

## Files

```text
main.py                 acquisition scheduler / record production
rw_protocol.py          MicroPython protocol-v0.1 encoder
rw_sensors.py           sensor configuration + raw acquisition
rw_timing.py            32->64-bit LSM timestamp extension
rw_transport_usb.py     USB CDC transport adapter
```

Copy these files to the Pico filesystem root using the same names. `/vl53l5cx_firmware.bin` must already be present as described in the [build guide](../../../docs/build-guide.md).

Copy `main.py` last: once it starts, stdout becomes a binary stream and readable REPL text is no longer expected.

## Current acquisition source profile

`SOURCE_PROFILE`:

```text
pico2w-lsm6dsox-lis3mdl-vl53l5cx-8x8-15hz
```

Current acquisition configuration:

- LSM6DSOX + LIS3MDL on I2C0, GP4/GP5, 400 kHz;
- VL53L5CX on I2C1, GP2/GP3, 1 MHz;
- LSM6DSOX accel `CTRL1_XL = 0x48` (104 Hz, +/-4 g);
- LSM6DSOX gyro `CTRL2_G = 0x44` (**104 Hz, +/-500 deg/s**);
- BDU/IF_INC `CTRL3_C = 0x44`;
- FIFO accel/gyro BDR `0x44`, FIFO/timestamp mode `0x46`;
- FIFO serviced on the validated 20 ms host cadence;
- LIS3MDL control bytes `74 00 00 0c 40`;
- VL53L5CX 8x8 at 15 Hz;
- LIS3MDL address fixed at `0x1E` by `ADM -> 3V3`.

`STREAM_INFO` reports the actual runtime register/configuration bytes. Host analysis must read those bytes rather than assuming a scale.

### Why the gyro is now +/-500 deg/s

The earlier acquisition configuration used `CTRL2_G = 0x40` (+/-250 deg/s), matching the original diagnostic baseline. During physical ToF/body boresight testing, one legitimate mixed-axis motion reached about 267 deg/s on the mapped body-X axis and produced a large gyro/gravity closure failure.

The acquisition firmware was therefore changed to `0x44`, keeping the same nominal 104 Hz ODR while doubling rate headroom. Follow-up physical tests recovered simple and compound rotations with sub-degree gravity closure while using only about 14% of the new full scale.

The host boresight path now also checks full-scale utilisation directly: warning at 80%, rejection at 90% of the range reported by `STREAM_INFO`.

The frozen diagnostic/self-test firmware may retain the older diagnostic configuration because its role is hardware reproduction, not calibration-motion capture. Do not infer acquisition configuration from the diagnostic source; use `STREAM_INFO` from the actual acquisition stream.

## Timestamp handling

The producer preserves source clock domains:

- each complete IMU slot carries an extended native LSM timestamp tick;
- MAG records carry Pico before/after read brackets;
- ToF records carry Pico data-ready-observation/read-complete timestamps;
- `CLOCK_SYNC` records carry Pico-before / LSM-tick / Pico-after observations.

No firmware-derived common-time estimate is written into sensor records. Replay/host code can therefore refit the sensor-clock relationship later.

A direct LSM timestamp read can be newer than FIFO records waiting in backlog. `LsmTickExtender` maps nearby 32-bit observations into one 64-bit epoch without moving its anchor backwards when an older FIFO timestamp arrives.

## IMU batching, queueing and loss visibility

Timestamped IMU slots are batched four at a time, with partial batches flushed after 50 ms.

Complete encoded frames enter a fixed queue before transport. Sequence numbers are allocated before queue admission. If transport cannot keep up, loss becomes visible as both sequence gaps and cumulative `STATUS` counter changes rather than hidden timing corruption.

The USB adapter writes bounded bursts rather than one tiny chunk per scheduler pass. The packet CRC implementation uses a nibble lookup table; both changes were required to sustain the validated producer rate under MicroPython.

## VL53L5CX binding limitation

The current Pimoroni MicroPython binding exposes distance and reflectance arrays but not ST's per-zone `target_status`. `TOF_GRID` therefore uses field mask `0x0003` (distance + reflectance) rather than inventing validity metadata.

Negative/out-of-range driver distances are rejected rather than silently wrapping into protocol uint16 values; such a frame increments `tof_errors`.

## Hardware-validation procedure

1. Run the frozen diagnostic firmware first and require `SYSTEM READY: PASS`.
2. Copy `rw_protocol.py`, `rw_sensors.py`, `rw_timing.py`, `rw_transport_usb.py` to the Pico root.
3. Copy acquisition `main.py` last.
4. Confirm `/vl53l5cx_firmware.bin` remains present.
5. Close/disconnect Thonny so the COM port is free.
6. Install pyserial on the PC if required:

   ```powershell
   py -m pip install pyserial
   ```

7. From the repository root, run a smoke probe, replacing the port as needed:

   ```powershell
   py host/python/probe_serial.py COM5 --warmup 3 --seconds 15 --output packets.bin
   ```

Pass criteria for the measured interval:

- valid Rangeweave frames received;
- `bad frames = 0`;
- `seq gaps = 0`;
- expected record families (`IMU_BATCH`, `MAG`, `TOF_GRID`, `CLOCK_SYNC`, `STATUS`, `STREAM_INFO`) observed;
- deltas for drops, FIFO structural/overrun errors and sensor/clock-sync errors remain zero.

Approximate healthy 15-second counts on the reference unit were ~380 `IMU_BATCH`, ~150 `MAG`, ~225 `TOF_GRID`, ~15 `CLOCK_SYNC`, ~15 `STATUS` and one or two `STREAM_INFO` records. Treat these as sanity ranges, not protocol requirements.

For offline timing inspection:

```powershell
py host/python/analyze_capture.py packets.bin
```

For canonical capture sessions:

```powershell
py host/python/capture.py COM5 --seconds 10 --name example
```

`capture.py` defaults to a 3-second warm-up before recorded data begins. Developer motion tests must account for that; the planned guided boresight command will own this timing and provide explicit movement cues.

## Calibration-motion inspection

Current host tooling can inspect the acquisition gyro configuration and physical motion quality:

```powershell
py host/python/inspect_relative_rotation.py <capture>
```

For the current reference rig, the physically validated mapping is:

```text
body X = -imu X
body Y = -imu Z
body Z = -imu Y
```

That mapping is assembly-specific and must not be copied to a differently mounted IMU without physical validation.

See [`../../../docs/boresight-calibration.md`](../../../docs/boresight-calibration.md) and [`../../../docs/validation/boresight-reference-rig-2026-09.md`](../../../docs/validation/boresight-reference-rig-2026-09.md).

## Getting back to development mode

Because `main.py` starts automatically and emits binary data, interrupt it with Ctrl-C from Thonny/serial when practical. If necessary, use BOOTSEL/reflash recovery and restore the validated MicroPython image. Keep the diagnostic source and VL53L5CX firmware blob available on the PC.

## Not implemented in this firmware layer

The MCU producer intentionally does not perform point projection, world-attitude fusion, SLAM, mapping, compression, host command/control or BLE/Wi-Fi transport. Those belong above/beside the transport-independent record layer.
