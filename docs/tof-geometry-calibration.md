# ToF geometry profiles and plane calibration

**Status: generic host calibration framework. No producer/capture semantics are changed.**

This document defines how Rangeweave represents and estimates per-device 8x8 ToF zone geometry above the lossless producer-native capture layer.

Per-device calibration is **optional**. The built-in ST-derived nominal profile remains the normal out-of-box geometry for a newly assembled Rangeweave system. Builders only need this calibration workflow when they want to characterise their particular sensor and refine geometric accuracy further.

The central rule is deliberately permissive:

> Rangeweave defines the coordinate frame and calibration procedure. It does not require a physical sensor's 64-zone lattice to be symmetric, evenly spaced, monotonic, or bowed in any particular direction.

A calibrated unit may bow inward, bow outward, be asymmetric, or differ independently zone-by-zone from the built-in ST nominal fallback.

## Geometry profile

`host/python/rangeweave_geometry.py` defines `TofGeometryProfile`.

Each producer-native ZoneID stores one independent slope pair:

```text
(x_per_z, y_per_z)
```

The existing VL53L5CX distance contract is unchanged:

```text
X = x_per_z[zone] * distance_mm
Y = y_per_z[zone] * distance_mm
Z = distance_mm
```

The default built-in profile remains:

```text
name: vl53l5cx-st-plane-algo-2022-corrected-yaw
role: nominal-fallback
```

It is useful without any builder calibration, but it is not a regularisation target and calibrated profiles are not required to resemble it.

Projection functions accept an optional explicit profile. Omitting it uses the ST nominal fallback.

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

The JSON contains all 64 producer-native zones independently. A shortened structural example is:

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

A real v1 artifact must contain exactly 64 unique ZoneIDs `0..63`; the abbreviated example above is illustrative only.

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

The summary and plot title identify the selected profile name and role.

## Known-plane calibration model

`host/python/rangeweave_tof_calibration.py` estimates every ZoneID independently from observations of known planes.

For one plane in `tof_optical`:

```text
nx*X + ny*Y + nz*Z = d
```

where `(nx, ny, nz)` is the plane normal and `d` is its offset from the optical origin. Substituting the Rangeweave projection model gives, for each zone measurement `Z`:

```text
nx*x_per_z + ny*y_per_z = d/Z - nz
```

That is linear in the two unknown zone slopes. Multiple plane poses therefore give an ordinary two-parameter least-squares problem for each ZoneID.

The 64 zones are solved separately. There is no neighbour smoothing, symmetry constraint, ST-LUT prior, or common radial-distortion model in this calibration method.

## Required pose diversity

The calibration planes must constrain both X and Y.

For example, several one-axis-only tilts cannot determine the other slope component; the solver detects the rank-deficient system and rejects it rather than inventing a value.

A practical calibration set should therefore contain independent tilts about both image axes, preferably with redundancy and one or more mixed tilts.

A fronto-parallel plane is useful for range/bias diagnostics but contributes no X/Y ray-direction constraint by itself.

## Invalid measurements

A calibration plane carries 64 axial-Z measurements in producer-native ZoneID order.

For an individual zone, `None`, non-finite, zero, or negative distances are skipped. Calibration can still succeed for that zone if the remaining valid planes independently constrain both slopes.

The current producer does not yet expose VL53L5CX `target_status`, so the physical calibration workflow currently has the same quality-information limitation as the existing point-cloud pipeline. Target-status acquisition remains a separate refinement.

## Fit diagnostics

For every zone the solver records:

- observation count;
- fitted `x_per_z` and `y_per_z`;
- RMS residual to the known planes in millimetres;
- maximum absolute plane residual in millimetres.

The calibrated profile metadata stores all per-zone residual summaries plus aggregate maxima/means.

Because plane normals are normalised internally, those residuals are geometric plane-distance errors in millimetres.

## Synthetic regression requirement

CI deliberately calibrates a synthetic 64-zone sensor whose lattice is:

- outward-bowing rather than matching the ST fallback;
- asymmetric;
- offset/cross-coupled so opposite zones are not forced mirror images.

Five known tilted planes are generated from that synthetic truth. The solver must recover all 128 independent slope values to floating-point precision and predict a separate held-out plane.

This test exists specifically to prevent a future implementation from accidentally introducing an ST-profile prior or symmetry assumption.

CI also checks that a rank-deficient all-one-axis plane set is rejected.

## Calibration versus range correction

This geometry solver estimates only `(X/Z, Y/Z)` ray slopes. It does not fit range scale, range offset, per-zone Z bias, cover-window effects, or the optical-centre translation.

Those quantities should remain separate calibration layers. In particular, Rangeweave should not bend a zone ray merely to compensate for an axial range bias.

## Physical capture workflow

The optional physical workflow is documented in [`tof-calibration-plane-workflow.md`](tof-calibration-plane-workflow.md).

It defines a general measured known-plane pose rather than requiring a particular jig. `host/python/rangeweave_tof_calibration_capture.py` now reduces one stationary canonical capture to robust per-zone median distances, reports valid coverage, MAD and temporal half-drift, and preserves existing stream/metadata/health evidence before that observation is admitted to the plane solver.

The remaining workflow work is to define a multi-capture manifest, run the solver from that manifest, and perform held-out physical validation against the nominal fallback.
