# Python reference host

**Status: protocol decoder implemented; live USB smoke validation started; full capture/replay next.**

The protocol/capture boundary is deliberately dependency-light. Scientific/3D libraries can be added later behind project-owned interfaces without becoming wire-format dependencies.

Implemented now:

1. [`rangeweave_protocol.py`](rangeweave_protocol.py) — protocol v0.1 COBS framing, CRC, stream recovery and semantic decoders;
2. [`probe_serial.py`](probe_serial.py) — small pyserial-based live Pico smoke probe used to validate framing, sequence continuity, record counts and `STATUS` deltas;
3. shared golden-vector tests in [`../../tests/test_protocol.py`](../../tests/test_protocol.py), plus acquisition-side encoder/timestamp tests.

`probe_serial.py` is **not yet the canonical capture/replay layer**. It may optionally save raw bytes to `packets.bin`, but its immediate job is to tell us whether the first Pico producer survives real USB acquisition without framing corruption, sequence loss or sensor-health regressions.

Example on Windows:

```powershell
py -m pip install pyserial
py host/python/probe_serial.py COM7 --warmup 3 --seconds 15 --output packets.bin
```

Close Thonny or any other serial client first so the COM port is free. See [`../../firmware/pico2w/acquisition/README.md`](../../firmware/pico2w/acquisition/README.md) for the complete hardware-validation procedure.

Next:

1. hardware-validate the Pico v0.1 acquisition producer;
2. turn the byte source + raw-file writer into the canonical lossless capture layer;
3. add replay through the exact same `StreamDecoder` API;
4. add capture metadata/session directory handling;
5. raw health/depth viewer;
6. calibrated 64-point projection;
7. orientation;
8. odometry/mapping baselines.

The protocol and calibration semantics must remain reproducible in Kotlin/Android without importing Python-specific concepts.
