# ToF geometry profiles and plane calibration

**Status: generic host calibration framework for optional per-device VL53L5CX intrinsic/ray refinement. No producer/capture semantics are changed.**

This document defines how Rangeweave represents and estimates per-device 8x8 ToF zone geometry above the lossless producer-native capture layer.

Per-device intrinsic calibration is **optional**. The built-in ST-derived nominal profile remains the normal out-of-box geometry for a newly assembled Rangeweave system. Builders only need this workflow when they want to characterise their particular sensor and refine geometric accuracy further.

This intrinsic task is distinct from ToF/body **boresight** calibration. Intrinsic calibration estimates 64 zone rays inside `tof_optical`; boresight estimates one rigid `R_body_from_tof` rotation for the assembled head. See [`boresight-calibration.md`](boresight-calibration.md) for the latter.

The central intrinsic-calibration rule is deliberately permissive:

> Rangeweave defines the coordinate frame and calibration procedure. It does not require a physical sensor's 64-zone lattice to be symmetric, evenly spaced, monotonic, or bowed in any particular direction.

A calibrated unit may bow inward, bow outward, be asymmetric, or differ independently zone-by-zone from the built-in ST nominal fallback.

## Geometry profile

`host/python/rangeweave_geometry.py` defines `TofGeometryProfile`.

Each producer-native ZoneID stores one independent slope pair:

```text
(x_per_z, y_per_z)
```

The VL53L5CX distance contract is:

```text
X = x_per_z[zone] * distance_mm
Y = y_per_z[zone] * distance_mm
Z = distance_mm
```

The default profile remains:

```text
name: vl53l5cx-st-plane-algo-2022-corrected-yaw
role: nominal-fallback
```

It is useful without builder calibration, but it is not a regularisation target and calibrated profiles are not required to resemble it.

Projection functions accept an optional explicit profile. Omitting it uses the nominal fallback.

## Portable `tof_geometry.json`

Profiles use the versioned schema:

```text
schema:         rangeweave.tof-geometry-profile
schema_version: 1
frame:          tof_optical
distance_contract: axial-z-mm
layout_id:      0
rows:           8
cols:           8
```

A complete artifact contains all 64 producer-native ZoneIDs independently. Example structure:

```json
{
  "schema": "rangeweave.tof-geometry-profile",
  "schema_version": 1,
  "name": "my-rangeweave-tof-2026-08-24",
  "role": "calibrated",
  "sensor": "VL53L5CX",
  "rows": 8,
  "cols": 8,
  "layout_id": 0,
  "frame": "tof_optical",
  "distance_contract": "axial-z-mm",
  "zones": [
    {"zone_id": 0, "x_per_z": 0.361, "y_per_z": 0.355},
    {"zone_id": 1, "x_per_z": 0.249, "y_per_z": 0.348}
  ],
  "metadata": {}
}
```

A real v1 artifact must contain exactly 64 unique ZoneIDs `0..63`; the shortened example is illustrative only.

Use:

```python
profile = rangeweave_geometry.load_geometry_profile("tof_geometry.json")
rangeweave_geometry.save_geometry_profile(profile, "tof_geometry.json")
```

The point-cloud viewer can consume an artifact directly:

```powershell
py host/python/view_point_cloud.py <capture> `
  --geometry-profile calibration\tof_geometry.json
```

## Known-plane calibration model

`host/python/rangeweave_tof_calibration.py` estimates every ZoneID independently from observations of **known planes**.

For one plane in `tof_optical`:

```text
nx*X + ny*Y + nz*Z = d
```

where `(nx, ny, nz)` is the measured plane normal and `d` is its measured offset from the optical origin. Substituting the Rangeweave projection model gives, for each zone measurement `Z`:

```text
nx*x_per_z + ny*y_per_z = d/Z - nz
```

That is linear in the two unknown zone slopes. Multiple plane poses therefore give an ordinary two-parameter least-squares problem for each ZoneID.

The 64 zones are solved separately. There is no neighbour smoothing, symmetry constraint, ST-LUT prior, or common radial-distortion model.

## Required pose diversity

The calibration planes must constrain both X and Y. Several one-axis-only tilts cannot determine the other slope component; the solver rejects rank-deficient sets rather than inventing a value.

A practical intrinsic dataset should contain independent tilts about both image axes, redundancy, and preferably one or more mixed tilts. Reserve one independent measured pose as held-out validation.

A fronto-parallel plane is useful for range/bias diagnostics but contributes no X/Y ray-direction constraint by itself.

## Invalid measurements

For an individual zone, `None`, non-finite, zero or negative distances are skipped. Calibration can still succeed if the remaining valid planes independently constrain both slopes.

The current producer does not expose VL53L5CX `target_status`, so the physical intrinsic workflow has the same quality-information limitation as the existing point-cloud pipeline. Target-status acquisition remains a separate refinement.

## Fit diagnostics

For every zone the solver records:

- observation count;
- fitted `x_per_z` and `y_per_z`;
- RMS residual to the known planes in millimetres;
- maximum absolute plane residual in millimetres.

The calibrated profile metadata stores per-zone residual summaries plus aggregate diagnostics. Plane normals are normalized internally, so these residuals are geometric plane-distance errors in millimetres.

## Synthetic regression requirement

CI calibrates a synthetic 64-zone sensor whose lattice deliberately differs from the ST fallback: outward-bowing, asymmetric and cross-coupled. The solver must recover all 128 independent slope values and predict a separate held-out plane. A rank-deficient one-axis set must be rejected.

This protects against accidentally introducing a symmetry assumption or ST-profile prior.

## Calibration versus range correction

This geometry solver estimates only `(X/Z, Y/Z)` ray slopes. It does not fit range scale/offset, per-zone Z bias, cover-window effects or optical-centre translation. Those remain separate calibration layers; do not bend a zone ray merely to compensate for axial range bias.

## Physical capture workflow

The physical intrinsic workflow is documented in [`tof-calibration-plane-workflow.md`](tof-calibration-plane-workflow.md). It defines a general **measured known-plane pose** rather than requiring a particular fixture.

`host/python/rangeweave_tof_calibration_capture.py` reduces one stationary canonical capture to robust per-zone median distances, reports valid coverage, MAD and temporal half-drift, and preserves stream/metadata/health evidence before the observation is admitted to the plane solver.

The reducer can be exercised against any suitable stationary flat-plane capture before building a dedicated measured intrinsic-calibration fixture:

```powershell
py host/python/inspect_tof_calibration_capture.py <stationary-capture> --show-grids
```

That gives an empirical baseline for ordinary zone MAD and half-capture drift on the current hardware.

The remaining intrinsic workflow work is to define the final multi-capture manifest/artifact promotion path and perform held-out physical validation against the nominal fallback.
