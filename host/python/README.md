# Python reference host

**Status: protocol, Pico acquisition, canonical capture/replay and first reference dataset suite validated; raw depth/health viewer candidate added.**

The protocol/capture boundary is deliberately dependency-light. Scientific/3D libraries can be added later behind project-owned interfaces without becoming wire-format dependencies.

Implemented now:

1. [`rangeweave_protocol.py`](rangeweave_protocol.py) — protocol v0.1 COBS framing, CRC, stream recovery and semantic decoders;
2. [`rangeweave_capture.py`](rangeweave_capture.py) — common byte-source/stream-summary, metadata, hash and replay-parity helpers;
3. [`capture.py`](capture.py) — canonical pyserial recorder producing a session directory with aligned `packets.bin`, `metadata.json` and `notes.txt`;
4. [`replay.py`](replay.py) — replay/integrity tool feeding recorded bytes back through the exact same `StreamDecoder`;
5. [`rangeweave_depth.py`](rangeweave_depth.py) — standard-library raw ToF analysis and per-zone statistics;
6. [`view_depth.py`](view_depth.py) — optional-Matplotlib 8x8 depth/reflectance and health viewer;
7. [`probe_serial.py`](probe_serial.py) — live Pico smoke probe retained for firmware/framing validation;
8. [`analyze_capture.py`](analyze_capture.py) — offline timing/health analysis for raw Rangeweave byte captures;
9. shared protocol/acquisition/capture/depth tests under [`../../tests/`](../../tests/).

The physical Pico 2 W reference producer and canonical capture/replay path have both passed live validation with zero measured framing, sequence, drop, FIFO or sensor errors. A first named dataset suite now covers stationary, yaw/pitch/roll rotation, a flat wall at approximately 1000 mm, and simple forward/back translation.

## Canonical capture

Install pyserial once if needed:

```powershell
py -m pip install pyserial
```

Close Thonny or any other serial client so the Pico COM port is free, then record a session:

```powershell
py host/python/capture.py COM5 --seconds 30 --name stationary --notes "Sensor stationary on desk"
```

By default this creates:

```text
captures/
  capture_YYYYMMDD_HHMMSSZ_stationary/
    metadata.json
    packets.bin
    notes.txt
```

The recorder discards a configurable startup/backlog warm-up, aligns the beginning of `packets.bin` to a known `0x00` frame delimiter, and after the requested duration runs only long enough to close the current frame. Completed captures therefore contain complete Rangeweave wire frames rather than arbitrary serial-read fragments.

`metadata.json` records file SHA-256/size, protocol versions, sequence range/gaps, record counts, `STREAM_INFO`, and first/last `STATUS` health plus counter deltas. Raw `STREAM_INFO` TLVs are retained as hex alongside the decoded view of currently known fields.

The capture directory format is documented in [`../../docs/capture-format.md`](../../docs/capture-format.md).

## Replay

Replay a complete session with:

```powershell
py host/python/replay.py captures/capture_YYYYMMDD_HHMMSSZ_stationary
```

Replay verifies the `packets.bin` SHA-256 and byte count and checks that decoding reproduces the capture-time summary stored in metadata. The file source feeds the same `rangeweave_protocol.StreamDecoder` used during live acquisition; there is no separate replay data model.

For regression/CI-style use, add `--strict` to return non-zero when the recorded stream itself contains bad/semantic frames, sequence gaps or acquisition-health counter increases:

```powershell
py host/python/replay.py captures/capture_YYYYMMDD_HHMMSSZ_stationary --strict
```

`replay.py` may also be pointed directly at a raw `packets.bin`; in that case it replays and summarizes the stream but cannot perform metadata parity checks.

## Raw depth / health viewer

The first viewer remains in protocol-native 2D zone space. It does **not** yet project points into `tof_optical` or assume that producer-native row/column indices correspond to physical top/left.

Install Matplotlib for the graphical frontend:

```powershell
py -m pip install matplotlib
```

Then inspect a canonical capture:

```powershell
py host/python/view_depth.py captures/capture_20260821_004124Z_flat-wall-1000mm
```

The CLI prints numeric 8x8 grids for mean distance, distance population standard deviation, mean reflectance and the selected raw frame. The graphical view shows the same four diagnostics as heatmaps.

Useful variants:

```powershell
# Text-only; rangeweave_depth.py itself remains standard-library only.
py host/python/view_depth.py <capture> --summary-only

# Inspect a particular ToF record.
py host/python/view_depth.py <capture> --frame 100

# Save the figure without opening a window.
py host/python/view_depth.py <capture> --save wall.png --no-show
```

The viewer verifies capture metadata parity when given a session directory and reports normal stream-health deltas alongside the depth data.

See [`../../docs/raw-depth-viewer.md`](../../docs/raw-depth-viewer.md) for the important producer-native layout semantics and current non-goals.

## Validation utilities

The original smoke tools remain useful:

```powershell
py host/python/probe_serial.py COM5 --warmup 3 --seconds 15 --output packets.bin
py host/python/analyze_capture.py packets.bin
```

`probe_serial.py` remains the short firmware/transport acceptance test rather than the durable recording interface.

`analyze_capture.py` treats ToF `mcu_ready_us` as the Pico software-observation time defined by the protocol. Individual observation intervals can contain scheduler jitter; the tool reports net rate/period deficit rather than claiming that every long interval represents a skipped VL53L5CX frame.

## Next

1. validate the raw depth viewer against the 1000 mm flat-wall capture and inspect actual per-zone range/noise/reflectance patterns;
2. freeze coordinate-frame and zone-index conventions before merging any 64-point projection code;
3. add the Kotlin/Android protocol-parity smoke test using the shared golden fixtures;
4. implement calibrated 8x8-to-64-point projection;
5. proceed to orientation, controlled-pose scanning and later odometry/mapping.

Python is the reference implementation, not the architecture authority. Capture, protocol and calibration semantics must remain reproducible in Kotlin/Android without importing Python-specific concepts.
