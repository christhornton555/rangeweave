# Pico 2 W acquisition firmware

**Status: EXPERIMENTAL / first hardware-validation candidate.**

This directory contains the first Rangeweave acquisition producer for the validated Pico 2 W reference sensor stack. It is deliberately separate from [`../diagnostics/reproducible_sensor_stack.py`](../diagnostics/reproducible_sensor_stack.py): the diagnostic remains the reproduction/self-test tool, while this firmware emits the binary [Rangeweave protocol v0.1](../../../protocol/spec-v0.1.md) stream used by capture/replay software.

This revision has passed protocol/logic tests under CPython, but **has not yet been validated on the physical Pico + sensor stack**. Do not treat it as a replacement for the v0.5 diagnostic baseline until the live validation procedure below passes.

## Files

```text
main.py                 acquisition scheduler / record production
rw_protocol.py          MicroPython protocol-v0.1 encoder
rw_sensors.py           validated sensor configuration + raw acquisition
rw_timing.py            hardware-independent 32->64-bit LSM timestamp extension
rw_transport_usb.py     first transport adapter: USB CDC stdout
```

On the Pico, copy all five files to the filesystem root using the filenames above. The VL53L5CX firmware blob must already be present as `/vl53l5cx_firmware.bin` as described in the build guide.

The `rw_` prefixes are intentional: these modules are copied to the Pico root for now, so generic names such as `protocol.py` or `sensors.py` would create avoidable collision risk. A later packaged MCU implementation may use a different filesystem layout without changing protocol semantics.

## Architecture boundary

```text
validated sensors/FIFO
        |
  raw acquisition
        |
 record creation
        |
 Rangeweave packetizer
        |
 complete-frame queue
        |
 transport adapter
        |
     USB CDC now
```

`rw_protocol.py`, record semantics and timestamps know nothing about USB. `rw_transport_usb.py` knows nothing about the sensors. This is the same boundary a future ESP32/BLE/Wi-Fi/local-storage producer must preserve.

## Current source profile

`SOURCE_PROFILE` is:

```text
pico2w-lsm6dsox-lis3mdl-vl53l5cx-8x8-15hz
```

The reference configuration mirrors the validated diagnostic baseline:

- LSM6DSOX + LIS3MDL on I2C0, GP4/GP5, 400 kHz;
- VL53L5CX on I2C1, GP2/GP3, 1 MHz;
- LSM6DSOX accel `0x48`, gyro `0x40`, BDU/IF_INC `0x44`;
- FIFO accel/gyro BDR `0x44` and FIFO/timestamp mode `0x46`;
- LIS3MDL control bytes `74 00 00 0c 40`;
- VL53L5CX 8x8 at 15 Hz;
- LIS3MDL address fixed at `0x1E` by `ADM -> 3V3`.

`STREAM_INFO` reports the observed WHO_AM_I values, `INTERNAL_FREQ_FINE`, actual programmed control bytes and ToF profile at runtime rather than making the host infer them.

### Current VL53L5CX binding limitation

The validated Pimoroni MicroPython binding exposes full `distance` and `reflectance` arrays, but does not expose ST's per-zone `target_status` array. The current producer therefore emits `TOF_GRID` with field mask `0x0003` (distance + reflectance) rather than fabricating a validity field.

The upstream ST result structure represents `distance_mm` as signed 16-bit values, while protocol v0.1 currently specifies its `DISTANCE_MM` field as uint16. This producer **does not silently wrap negative driver values**: if a negative/out-of-range distance is observed, that ToF frame is rejected and `tof_errors` increments. Live hardware validation will tell us whether this candidate-protocol edge case needs a v0.1 correction before the protocol is tagged stable.

## Timestamp handling

The acquisition firmware preserves source clock domains exactly as protocol v0.1 requires.

- Each complete IMU slot carries an extended native LSM timestamp tick.
- MAG carries MCU-before/after timestamps bracketing the successful raw XYZ read.
- ToF carries the MCU time at which software observed data-ready and the time after the complete frame read.
- `CLOCK_SYNC` carries MCU-before / extended-LSM-tick / MCU-after for a direct timestamp-register read.

A direct LSM timestamp read can be newer than FIFO records still waiting in backlog. `LsmTickExtender` therefore maps nearby 32-bit observations into one 64-bit epoch without moving its anchor backwards when an older FIFO timestamp arrives. This behaviour is regression-tested on the host.

No firmware-derived common-time estimate is written into sensor records.

## IMU batching and queueing

Complete timestamped IMU slots are batched four at a time. A partial batch is flushed after 50 ms, so batching reduces frame overhead without inferring sample timing from batch position.

Complete encoded frames enter a fixed 32-frame queue before transport. Sequence numbers are allocated **before** queue admission. If the transport cannot keep up, a dropped frame therefore creates both:

- a detectable sequence gap on the receiver; and
- an increment in the appropriate `STATUS` counters.

This is deliberate: transport backpressure must become observable data loss, not hidden sensor-timing damage.

## USB transport behaviour

The first adapter writes binary frames to MicroPython's built-in USB CDC stdout stream in bounded chunks. It attempts non-blocking/polling-first writes when the runtime exposes `select.poll()` support.

The first byte emitted by the adapter is a standalone `0x00` delimiter. This gives a Rangeweave receiver a clean framing boundary after any textual MicroPython soft-reset/startup bytes that may have appeared before `main.py` took over stdout.

After successful initialization, **do not expect readable text in the Thonny Shell**. The stream is binary by design, and acquisition code must not mix `print()` diagnostics into it.

## First hardware-validation procedure

1. First run the frozen diagnostic firmware and confirm the reference stack still reaches `SYSTEM READY: PASS`.
2. In Thonny, copy these four support modules to the Pico root:
   - `rw_protocol.py`
   - `rw_sensors.py`
   - `rw_timing.py`
   - `rw_transport_usb.py`
3. Copy this acquisition `main.py` to the Pico **last**. Once it starts, stdout becomes a binary stream.
4. Ensure `/vl53l5cx_firmware.bin` is still present.
5. Close/disconnect Thonny so it releases the Pico COM port.
6. On the PC, install pyserial if needed:

   ```powershell
   py -m pip install pyserial
   ```

7. From the repository root, run the smoke probe, replacing `COM7` with the Pico port:

   ```powershell
   py host/python/probe_serial.py COM7 --warmup 3 --seconds 15 --output packets.bin
   ```

The three-second warm-up deliberately ignores any queue backlog/drop counters accumulated while no host had the USB stream open. The measured window is what we use for the first steady-state judgement.

### Initial pass criteria

For the measured window, the first target is:

- valid Rangeweave frames are received;
- `bad frames = 0`;
- `seq gaps = 0`;
- `IMU_BATCH`, `MAG`, `TOF_GRID`, `CLOCK_SYNC`, `STATUS` and `STREAM_INFO` are all observed as expected for their rates;
- deltas for `frames_dropped`, `imu_samples_dropped`, `fifo_overruns`, `fifo_structural_errors`, `mag_errors`, `tof_errors` and `clock_sync_errors` are all zero.

Approximate healthy 15-second counts are expected to be on the order of ~375 `IMU_BATCH`, ~150 `MAG`, ~225 `TOF_GRID`, ~15 `CLOCK_SYNC`, ~15 `STATUS` and one or two `STREAM_INFO` records. These are sanity ranges, **not protocol requirements**; timing on the physical unit is the source of truth.

If the probe exits non-zero, keep its summary plus the resulting `packets.bin` and return to the frozen diagnostic before changing sensor registers.

## Getting the Pico back into development mode

Because `main.py` starts automatically and emits binary data, reconnecting a REPL can be less pleasant than with the diagnostic scripts. If needed, interrupt the running script with Ctrl-C from a serial/Thonny connection; if that is not practical, use the normal BOOTSEL/reflash recovery path and restore the validated MicroPython image. Keep the diagnostic source and firmware blob on the PC so recovery is deterministic.

## What this revision intentionally does not do

- no point projection, orientation fusion, SLAM or mapping;
- no conversion of raw IMU/magnetometer counts into physical units on the MCU;
- no host-to-device command channel;
- no compression;
- no BLE/Wi-Fi transport;
- no stable hardware/device identifier;
- no target-status synthesis when the current driver does not expose it.

Those are later layers. The purpose of this revision is to prove that the validated sensor stack can produce a loss-detectable, replayable Rangeweave byte stream without reintroducing the timing failures that motivated the FIFO/split-bus architecture.
