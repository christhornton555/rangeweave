# Python reference host

**Status: protocol, Pico acquisition, canonical capture/replay, raw depth/health analysis and live/temporal depth viewing validated; ToF zone orientation and optical axes frozen.**

The protocol/capture boundary is deliberately dependency-light. Scientific/3D libraries can be added later behind project-owned interfaces without becoming wire-format dependencies.

Implemented now:

1. [`rangeweave_protocol.py`](rangeweave_protocol.py) — protocol v0.1 COBS framing, CRC, stream recovery and semantic decoders;
2. [`rangeweave_capture.py`](rangeweave_capture.py) — common byte-source/stream-summary, metadata, hash and replay-parity helpers;
3. [`capture.py`](capture.py) — canonical pyserial recorder producing a session directory with aligned `packets.bin`, `metadata.json` and `notes.txt`, with optional non-blocking live ToF viewing;
4. [`replay.py`](replay.py) — replay/integrity tool feeding recorded bytes back through the exact same `StreamDecoder`;
5. [`rangeweave_depth.py`](rangeweave_depth.py) — standard-library raw ToF analysis, validity accounting, per-zone statistics and mean-plane diagnostics;
6. [`rangeweave_temporal.py`](rangeweave_temporal.py) — standard-library recorded-timestamp and constant-frame-rate export timing helpers;
7. [`rangeweave_live_view.py`](rangeweave_live_view.py) — optional separate-process latest-frame ToF display;
8. [`view_depth.py`](view_depth.py) — optional-Matplotlib static, temporal playback and MP4 depth viewer;
9. [`probe_serial.py`](probe_serial.py) — live Pico smoke probe retained for firmware/framing validation;
10. [`analyze_capture.py`](analyze_capture.py) — offline timing/health analysis for raw Rangeweave byte captures;
11. shared protocol/acquisition/capture/depth/temporal tests under [`../../tests/`](../../tests/).

The physical Pico 2 W reference producer and canonical capture/replay path have both passed live validation with zero measured framing, sequence, drop, FIFO or sensor errors. A first named dataset suite covers stationary, yaw/pitch/roll rotation, a flat wall, simple forward/back translation and moving-hand depth resolution.

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

The raw depth path remains in protocol-native 2D zone space and does not yet project ranges into 3D points. Producer-native row/column ordering is retained in capture, analysis and terminal output.

The standard-library analysis core treats `distance_mm == 0` as an invalid/sentinel distance for current v0.1 captures. It reports per-zone valid/invalid counts, computes valid-only distance statistics, excludes matching reflectance samples when distance is invalid, and fits a least-squares plane to the per-zone mean distance as a spatial diagnostic. Raw protocol decoding remains unchanged.

Install Matplotlib for the graphical frontend:

```powershell
py -m pip install matplotlib
```

Then inspect a canonical capture:

```powershell
py host/python/view_depth.py captures/capture_20260821_004124Z_flat-wall-1000mm
```

The CLI prints numeric 8x8 grids for valid-only mean/stddev, valid counts, invalid percentages, valid-distance reflectance mean, plane residuals and the selected raw frame. The graphical view shows six corresponding diagnostics.

Useful variants:

```powershell
# Text-only; does not import Matplotlib.
py host/python/view_depth.py <capture> --summary-only

# Inspect a particular ToF record; negative indices count from the end.
py host/python/view_depth.py <capture> --frame 100

# Save the figure without opening a window.
py host/python/view_depth.py <capture> --save wall.png --no-show
```

The viewer verifies capture metadata parity when given a session directory and reports normal stream-health deltas alongside the depth data.

See [`../../docs/raw-depth-viewer.md`](../../docs/raw-depth-viewer.md) for producer-native layout semantics, the flat-wall validation result and current non-goals.

## Live and temporal depth

The live viewer is optional instrumentation layered on the canonical recorder. It runs in a separate process and receives only a non-blocking latest-frame copy; `packets.bin` remains authoritative even if the display cannot keep up.

```powershell
py host/python/capture.py COM5 --seconds 30 --name live-test --live-view
```

Use a narrower fixed live depth scale when useful:

```powershell
py host/python/capture.py COM5 --seconds 30 --name live-test --live-view `
  --live-min-mm 100 --live-max-mm 700
```

Recorded captures can be played back using their actual ToF `mcu_ready_us` intervals:

```powershell
py host/python/view_depth.py <capture> --play
```

Playback controls are Space for pause/resume, Left/Right for frame stepping, Home/End and Esc. The depth colour scale remains fixed for the entire playback and can be overridden with `--min-mm` / `--max-mm`.

Physical validation established a pure 180-degree mapping between producer-native zone ordering and the physically upright image. Static plots, live view, playback and MP4 output apply that presentation transform while raw packet order, terminal matrices and analysis remain producer-native.

The validated mapping is:

```text
physical upper-left   -> producer r07c07
physical upper-right  -> producer r07c00
physical lower-left   -> producer r00c07
physical lower-right  -> producer r00c00
```

`tof_optical` is now frozen as right-handed with `+X` image-right, `+Y` image-down and `+Z` forward into the scene. See [`../../docs/tof-zone-orientation.md`](../../docs/tof-zone-orientation.md) and [`../../docs/coordinate-frames.md`](../../docs/coordinate-frames.md).

MP4 export uses nearest-neighbour depth rendering and maps the recorded timestamps onto a constant-frame-rate video. Real acquisition gaps become held frames rather than being silently compressed. For normal test recordings, omit the path and the viewer creates a timestamped file under the Git-ignored `recordings/` directory:

```powershell
py host/python/view_depth.py <capture> --export-mp4 --fps 30
```

The automatic filename convention is:

```text
recordings/YYYYMMDD_HHMMSS_<capture-label>-<fps>fps.mp4
```

An explicit path remains supported when needed:

```powershell
py host/python/view_depth.py <capture> --export-mp4 other/path/depth.mp4 --fps 30
```

MP4 export requires an `ffmpeg` executable on `PATH`. The default FPS is derived from the median recorded ToF interval; use `--fps` to override it.

See [`../../docs/temporal-depth-viewer.md`](../../docs/temporal-depth-viewer.md) for the timing, display-orientation and non-blocking acquisition contract.

## Validation utilities

The original smoke tools remain useful:

```powershell
py host/python/probe_serial.py COM5 --warmup 3 --seconds 15 --output packets.bin
py host/python/analyze_capture.py packets.bin
```

`probe_serial.py` remains the short firmware/transport acceptance test rather than the durable recording interface.

`analyze_capture.py` treats ToF `mcu_ready_us` as the Pico software-observation time defined by the protocol. Individual observation intervals can contain scheduler jitter; the tool reports net rate/period deficit rather than claiming that every long interval represents a skipped VL53L5CX frame.

## Next

1. define nominal/calibrated direction vectors for all 64 VL53L5CX zones and implement `distance + ray -> (x, y, z)` in `tof_optical`;
2. add golden geometry examples and Kotlin/Android parity coverage for the zone/ray projection;
3. freeze `imu_sensor`, `mag_sensor`, `device_body`, quaternion and transform conventions;
4. calibrate rigid ToF/IMU/magnetometer assembly extrinsics;
5. proceed to orientation-aware scanning, controlled-pose reconstruction and later odometry/mapping.

Python is the reference implementation, not the architecture authority. Capture, protocol and calibration semantics must remain reproducible in Kotlin/Android without importing Python-specific concepts.
