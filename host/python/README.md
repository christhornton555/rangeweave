# Python reference host

**Status: protocol, Pico acquisition, canonical capture/replay, raw depth/health analysis, live/temporal depth viewing, ToF geometry, and the reference-rig guided P0-P5 ToF/body boresight path are implemented.**

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
9. [`rangeweave_geometry.py`](rangeweave_geometry.py) — nominal/profile-aware 64-zone projection into `tof_optical`;
10. [`rangeweave_tof_calibration.py`](rangeweave_tof_calibration.py) — optional independent-zone known-plane geometry calibration solver;
11. [`rangeweave_tof_calibration_workflow.py`](rangeweave_tof_calibration_workflow.py) — measured known-plane pose convention for optional physical calibration;
12. [`rangeweave_tof_calibration_capture.py`](rangeweave_tof_calibration_capture.py) — robust stationary-capture reduction to per-zone medians plus quality diagnostics;
13. [`inspect_tof_calibration_capture.py`](inspect_tof_calibration_capture.py) — CLI for checking one capture before it is admitted to an optional calibration set;
14. [`rangeweave_imu_relative.py`](rangeweave_imu_relative.py) — short-baseline relative body rotation with endpoint bias estimation and independent gravity closure;
15. [`rangeweave_imu_quality.py`](rangeweave_imu_quality.py) — configured gyro full-scale decoding and utilisation gates;
16. [`rangeweave_boresight_sequence.py`](rangeweave_boresight_sequence.py) and [`rangeweave_extrinsics.py`](rangeweave_extrinsics.py) — stationary-ToF quality gates, observation composition and fixed-plane ToF/body rotation solve;
17. [`guided_boresight.py`](guided_boresight.py) — builder-facing P0-P5 fixed-wall capture with explicit HOLD / MOVE NOW / HANDS OFF cues and automatic quality checking;
18. [`rangeweave_boresight_holdout.py`](rangeweave_boresight_holdout.py) — held-out ToF prediction and fit-stability evaluation;
19. [`generate_boresight_artifact.py`](generate_boresight_artifact.py) — P0-P4 -> held-out P5 validation, six-pose refit and provenance-rich `rangeweave.tof-body-rotation` artifact generation;
20. [`probe_serial.py`](probe_serial.py) — live Pico smoke probe retained for firmware/framing validation;
21. [`analyze_capture.py`](analyze_capture.py) — offline timing/health analysis for raw Rangeweave byte captures;
22. shared protocol/acquisition/capture/depth/temporal/geometry/calibration tests under [`../../tests/`](../../tests/).

The physical Pico 2 W reference producer and canonical capture/replay path have passed live validation with zero measured framing, sequence, drop, FIFO or sensor errors in the published reference runs. The September 2026 reference-rig boresight sequence also completed held-out P5 validation before its final six-pose refit.

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

The raw depth path remains in protocol-native 2D zone space. Producer-native row/column ordering is retained in capture, analysis and terminal output.

The standard-library analysis core treats `distance_mm == 0` as an invalid/sentinel distance for current v0.1 captures. It reports per-zone valid/invalid counts, computes valid-only distance statistics, excludes matching reflectance samples when distance is invalid, and fits a least-squares plane to the per-zone mean distance as a spatial diagnostic. Raw protocol decoding remains unchanged.

Install Matplotlib for the graphical frontend:

```powershell
py -m pip install matplotlib
```

Then inspect a canonical capture:

```powershell
py host/python/view_depth.py captures/capture_20260821_004124Z_flat-wall-1000mm
```

The CLI prints numeric 8x8 grids for valid-only mean/stddev, valid counts, invalid percentages, valid-distance reflectance mean, plane residuals and the selected raw frame. The graphical view shows corresponding diagnostics.

See [`../../docs/raw-depth-viewer.md`](../../docs/raw-depth-viewer.md) for producer-native layout semantics and the flat-wall validation result.

## Live and temporal depth

The live viewer is optional instrumentation layered on the canonical recorder. It runs in a separate process and receives only a non-blocking latest-frame copy; `packets.bin` remains authoritative even if the display cannot keep up.

```powershell
py host/python/capture.py COM5 --seconds 30 --name live-test --live-view
```

Recorded captures can be played back using their actual ToF `mcu_ready_us` intervals:

```powershell
py host/python/view_depth.py <capture> --play
```

Physical validation established a pure 180-degree mapping between producer-native zone ordering and the physically upright image. Static plots, live view, playback and MP4 output apply that presentation transform while raw packet order, terminal matrices and analysis remain producer-native.

`tof_optical` is frozen as right-handed with `+X` image-right, `+Y` image-down and `+Z` forward into the scene. See [`../../docs/tof-zone-orientation.md`](../../docs/tof-zone-orientation.md) and [`../../docs/coordinate-frames.md`](../../docs/coordinate-frames.md).

MP4 export uses nearest-neighbour depth rendering and maps the recorded timestamps onto a constant-frame-rate video. Real acquisition gaps become held frames rather than being silently compressed.

See [`../../docs/temporal-depth-viewer.md`](../../docs/temporal-depth-viewer.md) for the timing, display-orientation and non-blocking acquisition contract.

## ToF geometry and optional per-device intrinsic calibration

Rangeweave works without a per-device geometry calibration. The built-in ST-derived geometry profile remains the normal `nominal-fallback` used by the point-cloud projection path and by the current reference-rig boresight plane fitter.

Builders who want to refine the geometry of their particular unit may optionally record several stationary captures of a known flat plane. The generic solver stores 64 independent `(X/Z, Y/Z)` pairs and does not require symmetry or any particular inward/outward bow.

Before adding a stationary capture to such a calibration set, inspect it with:

```powershell
py host/python/inspect_tof_calibration_capture.py <capture>
```

For the producer-native 8x8 diagnostic grids:

```powershell
py host/python/inspect_tof_calibration_capture.py <capture> --show-grids
```

The reducer uses per-zone median distances, requires 90% valid returns by default for a zone to contribute to that pose, reports median absolute deviation (MAD) and first-half/second-half drift, and checks the existing stream/metadata/health evidence.

See [`../../docs/tof-geometry-calibration.md`](../../docs/tof-geometry-calibration.md) and [`../../docs/tof-calibration-plane-workflow.md`](../../docs/tof-calibration-plane-workflow.md).

## Guided ToF/body boresight

Boresight is a separate assembly-extrinsic calibration. It uses a fixed wall plus relative IMU rotations; exact commanded pose angles and the wall's absolute room orientation are not solver measurements.

For the standard builder sequence, place P0 roughly 500 mm from the centre of a clear ~1 m x 1 m wall patch and use a rigid three-axis support. Then run:

```powershell
py host/python/guided_boresight.py COM5
```

The default captures P0 through P5. Each motion receives 3 s discarded warm-up, 5 s recorded initial hold, 10 s movement/adjustment allowance and 12 s hands-off final settling hold. A separate stationary ToF capture follows each movement.

After a successful P0-P5 run, create the exact per-device artifact with:

```powershell
py host/python/generate_boresight_artifact.py <session-prefix>
```

The generator verifies the capture hashes against metadata, re-runs the quality path, fits P0-P4, predicts held-out P5 using M5's IMU-derived body orientation, reveals P5, refits P0-P5, and writes:

```text
calibration/tof-body-rotation-<session-prefix>.json
```

The JSON records the exact rotation matrix plus fit/validation diagnostics, capture SHA-256 values, `STREAM_INFO`, IMU mapping role, gyro-range provenance, quality gates and ToF geometry-profile role.

The September 2026 reference run achieved a held-out P5 normal error of 0.644 deg and a 0.639 deg change in the fitted 3-D rotation after P5 was revealed. See [`../../docs/boresight-calibration.md`](../../docs/boresight-calibration.md) and [`../../docs/validation/boresight-reference-rig-2026-09.md`](../../docs/validation/boresight-reference-rig-2026-09.md).

## Validation utilities

The original smoke tools remain useful:

```powershell
py host/python/probe_serial.py COM5 --warmup 3 --seconds 15 --output packets.bin
py host/python/analyze_capture.py packets.bin
```

For developer-level boresight diagnostics, the individual relative-rotation / stationary-ToF inspectors and explicit sequence/hold-out inspectors remain available alongside the guided workflow.

`probe_serial.py` remains the short firmware/transport acceptance test rather than the durable recording interface.

## Next

1. retain/review the exact generated reference-rig `rangeweave.tof-body-rotation` artifact;
2. close the Phase 2 PR once its artifact provenance and final diff are accepted;
3. move to the orientation layer: define quaternion/attitude conventions, combine gyro + gravity, then add confidence-gated magnetometer information later;
4. reproduce boresight on independently assembled units before making cross-unit calibration claims;
5. continue the optional intrinsic-profile work separately where better-than-nominal ToF ray geometry is needed.

Python is the reference implementation, not the architecture authority. Capture, protocol and calibration semantics must remain reproducible in Kotlin/Android without importing Python-specific concepts.
