# Raw depth / health viewer

**Status: Phase 2 implementation candidate.**

The first Rangeweave viewer is deliberately a **raw 2D diagnostic view**, not yet a calibrated 3D projection. It consumes a canonical capture through the same protocol decoder used by replay and summarizes the `TOF_GRID` records without assigning physical optical axes.

## Why this comes before 3D projection

Protocol v0.1 defines `layout_id = 0` as **producer-native flattened zone order**. It intentionally does not say that row 0 is physically top, that column 0 is physically left, or which direction is +X/+Y/+Z in `tof_optical`.

Those conventions must be frozen in [`coordinate-frames.md`](coordinate-frames.md) before 64-point projection code is merged. The raw viewer therefore labels axes only as:

```text
producer-native row index
producer-native column index
```

No display transform is written back into capture data or treated as sensor geometry.

This also keeps the current breadboard assembly extrinsics out of the raw depth path. A ToF board and IMU board may be mounted at arbitrary relative orientations; cross-sensor alignment is a later calibration/assembly-transform concern.

## Analysis core

[`../host/python/rangeweave_depth.py`](../host/python/rangeweave_depth.py) is standard-library only. It:

- accepts a canonical capture directory or raw `packets.bin`;
- feeds exact bytes through `rangeweave_protocol.StreamDecoder`;
- accumulates normal `StreamStats` health/sequence evidence;
- rejects a capture whose ToF rows/columns/layout change mid-stream;
- computes per-zone distance count, mean, population standard deviation, minimum and maximum;
- computes the same statistics for reflectance when present;
- reports ToF software-ready observation rate and read duration;
- verifies canonical capture metadata parity when a session directory is supplied.

The statistics preserve the producer-native flattened ordering. They do not rotate, mirror or geometrically project the data.

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

The terminal first prints stream health plus numeric 8x8 grids for:

- mean distance;
- distance population standard deviation;
- mean reflectance;
- the selected raw ToF frame.

The graphical window then shows four heatmaps:

1. selected raw distance frame;
2. mean distance over the capture;
3. per-zone distance standard deviation;
4. mean reflectance.

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

## Initial validation target

The first intended dataset is the stationary flat-wall capture at approximately 1000 mm. Before attempting any 3D projection, inspect:

- whether the 64 mean ranges are plausibly near the measured wall distance;
- whether there is a repeatable spatial bias pattern across zones;
- whether edge zones are noisier than central zones;
- whether reflectance varies systematically across the field;
- whether the stream remains free of decode, sequence and acquisition-health errors.

A flat wall does **not** imply that all 64 raw range values should be exactly equal. The physical interpretation depends on what the VL53L5CX distance value represents for each zone and, later, on the optical ray geometry. This viewer is intended to expose the measurements before we make those geometric assumptions.

## Deferred

This PR intentionally does not add:

- calibrated zone rays;
- `tof_optical` axis conventions;
- 3D point projection;
- IMU/ToF assembly extrinsics;
- orientation fusion;
- live animated USB viewing;
- filtering or invalid-zone heuristics not represented in the protocol.

Those should build on the raw measurements rather than being hidden inside the viewer.
