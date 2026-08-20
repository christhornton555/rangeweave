# Python reference host

**Status: protocol decoder and Pico live-acquisition smoke validation complete; canonical capture/replay next.**

The protocol/capture boundary is deliberately dependency-light. Scientific/3D libraries can be added later behind project-owned interfaces without becoming wire-format dependencies.

Implemented now:

1. [`rangeweave_protocol.py`](rangeweave_protocol.py) — protocol v0.1 COBS framing, CRC, stream recovery and semantic decoders;
2. [`probe_serial.py`](probe_serial.py) — pyserial-based live Pico smoke probe for framing, sequence continuity, record counts and `STATUS` deltas;
3. [`analyze_capture.py`](analyze_capture.py) — offline timing/health analysis for raw byte captures produced by the probe;
4. shared golden-vector tests in [`../../tests/test_protocol.py`](../../tests/test_protocol.py), plus acquisition-side encoder/timestamp/transport tests.

The physical Pico 2 W reference producer has now passed steady-state live validation: zero measured bad frames, sequence gaps, dropped frames, dropped IMU samples, FIFO errors or sensor errors, while the 15 Hz VL53L5CX stream measured 14.9955 Hz over the saved validation capture.

`probe_serial.py` is **not yet the canonical capture/replay layer**. It may optionally save raw bytes to `packets.bin`, but its role is validation rather than durable session management.

`analyze_capture.py` treats ToF `mcu_ready_us` as the Pico software-observation time defined by the protocol. Individual observation intervals can therefore contain scheduler jitter; the tool reports net rate/period deficit rather than claiming that every long interval represents a skipped VL53L5CX frame.

Example on Windows:

```powershell
py -m pip install pyserial
py host/python/probe_serial.py COM7 --warmup 3 --seconds 15 --output packets.bin
py host/python/analyze_capture.py packets.bin
```

Close Thonny or any other serial client first so the COM port is free. See [`../../firmware/pico2w/acquisition/README.md`](../../firmware/pico2w/acquisition/README.md) for the complete hardware-validation procedure and evidence.

Next:

1. turn the byte source + raw-file writer into the canonical lossless capture layer;
2. add replay through the exact same `StreamDecoder` API;
3. add capture metadata/session directory handling;
4. raw health/depth viewer;
5. calibrated 64-point projection;
6. orientation;
7. odometry/mapping baselines.

The protocol and calibration semantics must remain reproducible in Kotlin/Android without importing Python-specific concepts.
