# Packet protocol - design requirements

**Status: PLANNED. No byte-level protocol is frozen yet.**

The next engineering task is to design a small binary protocol before writing the acquisition firmware.

## Required properties

- versioned independently of firmware and calibration schemas;
- language-neutral;
- explicit endianness and integer widths;
- framed and resynchronisable after corruption or mid-stream connection;
- sequence-numbered so packet loss is detectable;
- explicit units/scale metadata or fixed units defined in the spec;
- transport-agnostic;
- raw-first: retain raw sensor values where practical;
- live and recorded byte streams should use the same packet frames.

## Initial logical records

- **IMU** - sequence, native LSM timestamp, raw accel XYZ, raw gyro XYZ.
- **MAG** - sequence, MCU timestamp, raw magnetic XYZ, status/quality.
- **TOF** - sequence, frame-observed MCU timestamp, mapped LSM timestamp, 64 ranges, 64 reflectance values, validity/status.
- **CLOCK_SYNC** - MCU timestamp plus raw LSM timestamp and correlation quality.
- **STATUS/META** - firmware/protocol/schema versions, sensor identities/configuration, `FREQ_FINE`, FIFO/drop/error counters.

## Cross-platform gate

Before extending the protocol beyond the initial records, create byte-level golden fixtures and require both Python and Kotlin decoders to produce the same semantic objects from those bytes.

See ADR-0001 and ADR-0005.
