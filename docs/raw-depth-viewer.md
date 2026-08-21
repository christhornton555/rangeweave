# Raw depth / health viewer

**Status: Phase 2 implementation candidate.**

The first Rangeweave viewer is deliberately a **raw 2D diagnostic view**, not yet a calibrated 3D projection. It consumes a canonical capture through the same protocol decoder used by replay and summarizes the `TOF_GRID` records without assigning physical optical axes.

## Why this comes before 3D projection

Protocol v0.1 defines `layout_id = 0` as **producer-native flattened zone order**. It intentionally does not say that row 0 is physically top, that column 0 is physically left, or which direction is +X/+Y/+Z in `tof_optical`.

Those conventions must be frozen in [`coordinate-frames.md`](coordinate-frames.md) before 64-point projection code is merged. The raw viewer therefore labels axes only as producer-native row/column indices. No display transform is written back into capture data or treated as sensor geometry.

This also keeps the current breadboard assembly extrinsics out of the raw depth path. A ToF board and IMU board may be mounted at arbitrary relative orientations; cross-sensor alignment is a later calibration/assembly-transform concern.

## Analysis core

[`../host/python/rangeweave_depth.py`](../host/python/rangeweave_depth.py) remains standard-library only. It:

- accepts a canonical capture directory or raw `packets.bin`;
- feeds exact bytes through `rangeweave_protocol.StreamDecoder`;
- accumulates normal `StreamStats` health/sequence evidence;
- rejects a capture whose ToF rows/columns/layout change mid-stream;
- treats `distance_mm == 0` as an invalid/sentinel distance sample for analysis;
- reports per-zone valid and invalid sample counts plus invalid percentages;
- computes valid-only distance count/mean/population-standard-deviation/min/max;
- excludes reflectance samples whose matching distance sample is invalid;
- fits a least-squares plane to the per-zone mean distance where at least 50% of samples are valid, and reports residual RMS/max plus the residual grid;
- reports ToF software-ready observation rate and read duration;
- verifies canonical capture metadata parity when a session directory is supplied.

The `0 mm` validity rule is an analysis-layer interpretation for the current v0.1 capture data. Raw protocol decoding remains lossless and unchanged.

The statistics preserve producer-native flattened ordering. They do not rotate, mirror or geometrically project the data.

## Viewer CLI

The graphical frontend is:

```text
host/python/view_depth.py
```

Install Matplotlib for the graphical view:

```powershell
py -m pip install matplotlib
```

Then inspect a capture, for example:

```powershell
py host/python/view_depth.py captures/capture_20260821_004124Z_flat-wall-1000mm
```

The terminal prints stream health plus numeric 8x8 grids for:

- valid-only mean distance;
- valid-only distance population standard deviation;
- valid distance sample count;
- invalid distance sample percentage;
- mean reflectance over valid distance samples;
- mean-distance plane residual;
- the selected raw ToF frame.

The graphical window shows six diagnostics:

1. selected raw distance frame;
2. valid-only mean distance;
3. valid-only distance population standard deviation;
4. invalid distance percentage;
5. mean-plane residual;
6. valid-distance reflectance mean.

Select another ToF record with:

```powershell
py host/python/view_depth.py <capture> --frame 100
```

Negative frame indices count from the end; `--frame -1` is the default.

For text-only analysis with no plotting dependency:

```powershell
py host/python/view_depth.py <capture> --summary-only
```

Save a figure without opening an interactive window with:

```powershell
py host/python/view_depth.py <capture> --save wall.png --no-show
```

## Flat-wall validation

The first physical validation dataset is:

```text
capture_20260821_004124Z_flat-wall-1000mm
```

The capture remained free of decode, sequence and acquisition-health errors. Validity diagnostics exposed one intermittently obstructed corner zone: `r07c00` contained 394 valid and 59 zero/sentinel readings (13.0% invalid), while `r00c00` contained one zero reading and all remaining zones were fully valid.

After invalid samples were excluded, the mean-distance grid fitted a plane across all 64 zones with approximately 2.36 mm RMS residual and about 7.62 mm maximum absolute residual. This is a useful spatial-quality diagnostic, not a calibration correction; the capture was an informal breadboard test and included operator obstruction plus the sensor's protective film.

A flat wall does **not** imply that all 64 raw range values should be exactly equal. The physical interpretation depends on the VL53L5CX zone ray geometry and later coordinate-frame calibration. The plane fit exists to expose spatial structure before those assumptions are introduced.

## Deferred

This PR intentionally does not add:

- calibrated zone rays;
- `tof_optical` axis conventions;
- 3D point projection;
- IMU/ToF assembly extrinsics;
- orientation fusion;
- live animated USB viewing;
- recorded temporal playback or video export;
- filtering beyond the explicitly documented zero-distance validity rule.

Those should build on the raw measurements rather than being hidden inside the viewer.
