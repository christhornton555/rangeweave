# Rangeweave wire protocol v0.1

**Status: EXPERIMENTAL / implementation candidate.**

This document is the normative byte-level specification for Rangeweave protocol **0.1**. It is intentionally small enough to implement in MicroPython, Kotlin and other embedded/host environments without a serialization framework.

Protocol 0.1 defines stream framing and five sensor/health record types. It does **not** yet define a device/configuration metadata record; that will be added before the first acquisition-firmware release once the metadata/privacy requirements are settled.

## 1. Design rules

- Wire semantics are independent of Pico, MicroPython, USB and Python.
- All multi-byte integers are **little-endian**.
- No structure has implicit padding or alignment bytes.
- Raw sensor integer values are preserved where practical.
- Time is never inferred from packet arrival time, packet sequence or nominal sample rate.
- Sensor/source clock domains remain explicit. A host may derive a common time line from `CLOCK_SYNC` records, but derived mapped timestamps are not written back into sensor records.
- A recording may store the exact live wire frames unchanged.
- Unknown record types may be retained or ignored after their frame has passed framing/CRC validation.
- A parser must be able to attach to the middle of a byte stream and recover after corruption.

## 2. Stream framing

The stream is a sequence of:

```text
COBS(decoded_frame) 0x00
COBS(decoded_frame) 0x00
...
```

`0x00` is the frame delimiter. COBS encoding guarantees that a valid encoded frame contains no zero byte.

Consecutive zero delimiters are harmless and may be ignored.

If COBS decoding, frame validation or CRC validation fails, discard that delimited frame and resume at the next `0x00`. Do not search for magic bytes inside a corrupt encoded frame.

### 2.1 Decoded frame

```text
offset  size  field
0       2     magic = ASCII "RW" (52 57)
2       1     protocol_major = 0
3       1     protocol_minor = 1
4       1     record_type
5       1     flags
6       2     payload_length
8       4     sequence
12      N     payload
12+N    2     CRC-16/CCITT-FALSE
```

The fixed header is 12 bytes. The CRC is stored little-endian.

`payload_length` is the number of payload bytes only; it excludes the header and CRC.

Protocol v0.1 limits the complete **decoded** frame, including header and CRC, to 1024 bytes.

### 2.2 CRC

Use CRC-16/CCITT-FALSE:

```text
width    = 16
poly     = 0x1021
init     = 0xFFFF
refin    = false
refout   = false
xorout   = 0x0000
check    = CRC("123456789") = 0x29B1
```

The CRC covers the 12-byte header followed by the payload. It does not include the CRC field itself, COBS encoding, or the trailing zero delimiter.

### 2.3 Sequence number

`sequence` is an unsigned 32-bit packet sequence assigned before a frame is queued for transport.

- It increments for every created protocol frame, regardless of record type.
- A gap therefore makes packet loss/drop detectable.
- It wraps modulo `2^32`.
- A device reset may restart the sequence. A transport reconnect alone should not reset it.
- A future stream/session metadata record will make reset/session boundaries explicit; v0.1 consumers should treat a large backwards sequence discontinuity as a probable new device session.

### 2.4 Header flags

All v0.1 header flag bits are reserved and producers must write `0`.

Consumers must preserve the value but must not infer meaning from it until a later protocol revision defines a flag.

## 3. Time domains

Rangeweave v0.1 uses two time domains in the reference implementation.

### 3.1 MCU monotonic microseconds

`mcu_*_us` fields are unsigned 64-bit monotonic microseconds since the current MCU boot/session.

They are not wall-clock UTC timestamps.

An MCU whose native tick API wraps must extend that counter so emitted protocol timestamps are monotonic 64-bit values.

### 3.2 LSM timestamp ticks

`lsm_tick` is an unsigned 64-bit extension of the LSM6DSOX native 32-bit timestamp counter.

The lower 32 bits therefore preserve the native hardware counter exactly. Extending wrap into the upper bits is lossless bookkeeping; it does not convert the value into seconds.

Consumers must use `CLOCK_SYNC` observations to map LSM ticks to MCU time rather than assuming a nominal IMU sample rate.

## 4. Record types

```text
0x01  IMU_BATCH
0x02  MAG
0x03  TOF_GRID
0x04  CLOCK_SYNC
0x05  STATUS
```

Existing record layouts are immutable within protocol major version 0. Later minor revisions may add new record types.

## 5. IMU_BATCH — 0x01

IMU FIFO samples are batched to amortize framing/transport overhead without changing their individual timestamps.

```text
offset  type       field
0       uint8      sample_count (1..16)
1       repeated   sample_count × IMU sample
```

Each IMU sample is exactly 20 bytes:

```text
type       field
uint64     lsm_tick
int16      accel_x_raw
int16      accel_y_raw
int16      accel_z_raw
int16      gyro_x_raw
int16      gyro_y_raw
int16      gyro_z_raw
```

The six signed values are raw LSM6DSOX output counts in the sensor register/FIFO axes. Scaling and calibration are host-side concerns.

No time spacing is implied by sample order. Each sample carries its own timestamp.

## 6. MAG — 0x02

MAG preserves one LIS3MDL XYZ sample plus a bracket around the host-side I2C read.

The payload is exactly 24 bytes:

```text
type       field
uint64     mcu_before_us
uint64     mcu_after_us
int16      mag_x_raw
int16      mag_y_raw
int16      mag_z_raw
uint8      status_reg
uint8      read_flags
```

`mcu_before_us` is sampled immediately before the successful raw-data read transaction; `mcu_after_us` is sampled immediately after it.

These values bracket **software acquisition**, not the unknown exact instant at which the LIS3MDL internally sampled the field.

`status_reg` is the raw LIS3MDL status register associated with the acquisition.

`read_flags`:

```text
bit 0  RETRY_USED
bits 1..7 reserved
```

A producer writes a MAG record only for a successfully obtained XYZ sample.

## 7. TOF_GRID — 0x03

TOF_GRID carries a sparse rectangular range grid and optional per-target companion arrays.

### 7.1 Prefix

The fixed prefix is 22 bytes:

```text
type       field
uint64     mcu_ready_us
uint64     mcu_read_complete_us
uint8      rows
uint8      cols
uint8      targets_per_zone
uint8      layout_id
uint16     field_mask
```

For v0.1:

- `1 <= rows * cols <= 64`;
- `targets_per_zone` must be `1`;
- `layout_id = 0` means **producer-native flattened zone order**.

`layout_id = 0` deliberately does not define a 3D axis convention. The protocol preserves the producer's flat sensor order; `docs/coordinate-frames.md` will define the geometric mapping before point-projection code is merged.

`mcu_ready_us` is captured when software observes the sensor's data-ready condition, before the potentially long frame read.

`mcu_read_complete_us` is captured after the complete grid has been obtained. The physical ranging result was available no later than `mcu_ready_us`; the second timestamp exposes read/transport latency rather than pretending that latency is sensor time.

### 7.2 Field mask

v0.1 defines:

```text
0x0001  DISTANCE_MM
0x0002  REFLECTANCE_PERCENT
0x0004  TARGET_STATUS
0xFFF8  reserved in v0.1
```

A producer may omit an array by clearing its bit.

For `zones = rows * cols` and v0.1 `targets_per_zone = 1`, target-specific arrays contain `zones` entries.

Arrays, when present, are appended after the prefix in this exact order:

1. `DISTANCE_MM`: `uint16[zones]`
2. `REFLECTANCE_PERCENT`: `uint8[zones]`
3. `TARGET_STATUS`: `uint8[zones]`

There is no padding between arrays.

`DISTANCE_MM` contains integer millimetres as emitted by the depth-sensor driver.

`REFLECTANCE_PERCENT` contains the driver's integer reflectance percentage values.

`TARGET_STATUS`, when available from the producer, contains the raw per-zone target-status code. Status meaning is sensor/profile-specific; protocol v0.1 transports the raw code rather than converting it to a Boolean validity decision.

A v0.1 semantic decoder must reject a TOF_GRID payload that sets unknown field-mask bits, but the outer frame can still be retained as valid opaque protocol data.

## 8. CLOCK_SYNC — 0x04

CLOCK_SYNC preserves the raw observation used to correlate the LSM timestamp domain with the MCU monotonic clock.

The payload is exactly 24 bytes:

```text
type       field
uint64     mcu_before_us
uint64     lsm_tick
uint64     mcu_after_us
```

The MCU timestamps bracket one direct read of the LSM timestamp register.

A host may use the bracket midpoint as one regression observation and half the bracket width as a timing-uncertainty estimate, but the protocol stores the bracket itself so improved clock models can be applied during replay.

## 9. STATUS — 0x05

STATUS is an acquisition-health snapshot. It is not a substitute for sensor data.

The payload is exactly 48 bytes:

```text
type       field
uint64     mcu_time_us
uint32     frames_created
uint32     frames_dropped
uint32     imu_samples_dropped
uint32     fifo_overruns
uint32     fifo_structural_errors
uint32     mag_retries
uint32     mag_errors
uint32     tof_errors
uint32     clock_sync_errors
uint16     queue_high_water
uint16     status_flags
```

Counters are monotonically increasing within one MCU session and wrap modulo their integer width.

`frames_created` counts protocol frames assigned a sequence number.

`frames_dropped` counts frames that the acquisition system could not deliver after they were assigned a sequence number.

`imu_samples_dropped` counts known missing IMU samples/slots at the acquisition layer.

`queue_high_water` is the maximum queued-frame/record count observed since boot. Its exact queue implementation is firmware-specific; the number exists as a backpressure diagnostic, not as protocol timing.

`status_flags`:

```text
0x0001  ACQUISITION_READY
0x0002  CLOCK_SYNC_ACTIVE
0x0004  FIFO_OVERRUN_LATCHED
0x0008  TRANSPORT_BACKPRESSURE
0xFFF0  reserved
```

## 10. Recording rule

The initial host recorder should write complete wire frames, including each trailing `0x00`, to `packets.bin` without translating them into a second binary representation.

A replay source should feed those bytes through the same stream decoder used for live transport.

Human-readable metadata and calibration are separate files; they do not replace the sensor packets.

## 11. Error handling

A stream consumer should distinguish:

- **framing error:** invalid COBS or impossible frame length;
- **CRC error:** COBS/frame length valid but checksum wrong;
- **unsupported protocol major:** framing is valid but semantics are incompatible;
- **unsupported record:** frame is valid but record type/field mask is unknown;
- **sequence gap:** one or more created frames were not observed;
- **semantic payload error:** known record type has an impossible payload length/count.

A corrupt frame must not poison later frames. Recovery boundary is the next zero delimiter.

## 12. Cross-language conformance

Canonical fixtures live in `protocol/test-vectors/`.

For protocol 0.1 the same fixture bytes must be consumed by:

1. the dependency-light Python reference decoder;
2. the future Kotlin/Android decoder;
3. any later MCU/host implementation that claims protocol 0.1 compatibility.

Tests must include at least one corruption/resynchronisation case.

## 13. Deferred from v0.1

Before acquisition firmware is declared feature-complete, a later protocol work item must define device/configuration metadata sufficient to describe the actual sensor profile and scaling without introducing an unwanted globally stable device identifier.

Also deferred:

- command/control packets;
- host-to-device negotiation;
- multiple targets per ToF zone;
- compression;
- encryption/authentication;
- wall-clock time;
- geometry/coordinate-frame transforms.

Those concerns must not be smuggled into transport-specific code.

## 14. Reference implementation

The standard-library-only reference implementation is:

`host/python/rangeweave_protocol.py`

Golden fixtures are:

`protocol/test-vectors/v0.1.json`

Run:

```bash
python -m unittest discover -s tests -v
```

before changing framing or record semantics.
