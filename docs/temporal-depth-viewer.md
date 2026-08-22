# Live and temporal depth viewer

**Status: Phase 2 implementation validated.**

This increment extends the raw 2D Rangeweave viewer with temporal playback, MP4 export and a live ToF display. Capture and analysis remain in **producer-native `TOF_GRID` row/column space**; graphical presentation uses the physically validated orientation documented in [`tof-zone-orientation.md`](tof-zone-orientation.md).

## Design rule: capture stays authoritative

Live visualization must never become part of acquisition correctness.

`capture.py --live-view` therefore keeps one serial reader and one canonical wire path:

```text
USB serial -> exact packets.bin bytes -> StreamDecoder / StreamStats
                                      \
                                       -> latest decoded TOF_GRID -> viewer process
```

The Matplotlib window runs in a separate spawned process. The capture process offers decoded ToF frames to a size-1 queue with non-blocking operations. If the viewer cannot keep up, stale **display updates** may be discarded; serial bytes are still written to `packets.bin` and decoded normally.

Closing the live window does not stop the capture.

## Presentation orientation

Physical validation established that producer-native ordering is rotated 180 degrees relative to the physically upright sensor image:

```text
physical upper-left   -> producer r07c07
physical upper-right  -> producer r07c00
physical lower-left   -> producer r00c07
physical lower-right  -> producer r00c00
```

Graphical static views, live viewing, recorded playback and MP4 export therefore apply a **180-degree presentation rotation**.

This remains a display transform only. `packets.bin`, decoded `TOF_GRID` tuples, terminal matrix output, statistics and plane fitting remain in producer-native row/column order. Rotated plots keep producer-native row/column labels so it remains possible to identify the underlying zone represented by each displayed cell.

The corresponding `tof_optical` convention is now frozen in [`coordinate-frames.md`](coordinate-frames.md): right-handed, `+X` image-right, `+Y` image-down and `+Z` forward into the scene. Per-zone ray angles and calibrated optical geometry remain separate work.

## Live view

Install Matplotlib if needed:

```powershell
py -m pip install matplotlib
```

Then record normally with the optional viewer:

```powershell
py host/python/capture.py COM5 --seconds 30 --name live-test --live-view
```

The live display uses a fixed colour scale so apparent depth does not change merely because the current frame has a different minimum/maximum. The defaults are 100..1000 mm and may be overridden:

```powershell
py host/python/capture.py COM5 --seconds 30 --name live-test --live-view `
  --live-min-mm 100 --live-max-mm 700
```

Zero distance is displayed as invalid/blank, matching the existing host analysis rule. The live view uses nearest-neighbour pixels; it does not interpolate spatial information that the 8x8 sensor did not measure.

## Recorded playback

Replay a canonical capture using the actual ToF `mcu_ready_us` timing:

```powershell
py host/python/view_depth.py captures/capture_YYYYMMDD_HHMMSSZ_name --play
```

Controls:

```text
Space       pause / resume
Left/Right  previous / next ToF frame and pause
Home/End    first / last ToF frame and pause
Esc         close playback
```

The playback colour scale is fixed for the whole run. By default it uses the capture-wide valid distance minimum and maximum. Override it when comparing multiple captures or when a narrow working range is more useful:

```powershell
py host/python/view_depth.py <capture> --play --min-mm 100 --max-mm 700
```

Playback rejects a ToF timeline whose MCU-ready timestamps move backwards rather than silently inventing timing.

## MP4 export

MP4 export requires both Matplotlib and an `ffmpeg` executable available on `PATH`.

For normal test recordings, omit the output path:

```powershell
py host/python/view_depth.py <capture> --export-mp4 --fps 30
```

The exporter creates `recordings/` if needed and writes a filename using:

```text
recordings/YYYYMMDD_HHMMSS_<capture-label>-<fps>fps.mp4
```

For example:

```text
recordings/20260821_231505_live-view-test-30fps.mp4
```

`recordings/` is intentionally ignored by Git. An explicit output path is still supported when required:

```powershell
py host/python/view_depth.py <capture> --export-mp4 other/path/example.mp4 --fps 30
```

If `--fps` is omitted, the exporter chooses a practical constant output frame rate from the median recorded ToF interval (normally 15 fps for the current producer).

MP4 itself is constant-frame-rate, but the exporter does **not** compress real acquisition gaps. At each output timestamp it uses the latest source ToF frame whose recorded `mcu_ready_us` is at or before that time. A long acquisition interval therefore appears as a held image in the video.

The export uses nearest-neighbour rendering, the same fixed depth scale as interactive playback and the same 180-degree presentation transform.

## Dependency boundary

`rangeweave_temporal.py` is standard-library only and owns replay/export timing semantics. Its timestamp and constant-frame-rate mapping are covered by normal CI tests.

`rangeweave_live_view.py` contains the optional multiprocessing/Matplotlib display path. Importing the module itself does not import Matplotlib; graphical dependencies are loaded only when live viewing actually starts.

The raw protocol and capture formats are unchanged by this work.

## Deferred

This viewer intentionally does not add:

- calibrated zone-ray projection;
- 3D point clouds;
- IMU orientation overlays or motion compensation;
- IMU/ToF assembly extrinsics;
- smoothing, interpolation or segmentation as acquisition semantics.

Those should build on the canonical raw stream, the validated zone orientation and the visualization tools established here.