# Python reference host

**Status: protocol decoder started; capture/replay next.**

The protocol/capture boundary is deliberately dependency-light. Scientific/3D libraries can be added later behind project-owned interfaces without becoming wire-format dependencies.

Implemented now:

1. [`rangeweave_protocol.py`](rangeweave_protocol.py) — protocol v0.1 COBS framing, CRC, stream recovery and semantic decoders;
2. shared golden-vector tests in [`../../tests/test_protocol.py`](../../tests/test_protocol.py).

Next:

1. USB byte-source adapter;
2. lossless `packets.bin` recorder;
3. replay adapter using the same stream decoder;
4. raw health/depth viewer;
5. calibrated 64-point projection;
6. orientation;
7. odometry/mapping baselines.

The protocol and calibration semantics must remain reproducible in Kotlin/Android without importing Python-specific concepts.
