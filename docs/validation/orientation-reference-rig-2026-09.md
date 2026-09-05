# Phase 3 orientation reference-rig evidence — September 2026

This record freezes the first empirical Rangeweave Phase 3 rotation-in-place wall acceptance gate from the physically tested September 2026 reference rig.

The gate is a **project validation contract**, not a universal VL53L5CX/LSM6DSOX specification and not a claim that independently assembled units will have identical timing, boresight or residual distributions. Builders may use quick-start nominal profiles, while calibrated builds should retain per-assembly artifacts and provenance.

## Calibration inputs

Reference ToF/body rotational boresight:

- artifact: `calibration/tof-body-rotation-boresight-guided-20260904_010944.json`
- role: `calibrated`
- `R_body_from_tof`: approximately `Rx +5.880 deg, Ry +3.930 deg, Rz +0.160 deg`
- boresight normal RMS: `0.797 deg`

Reference effective ToF timing:

- artifact: `calibration/tof-time-alignment-reference-rig-20260905.json`
- assembly id: `reference-rig-2026-09`
- role: `calibrated`
- effective offset: `-24 ms` added to protocol `mcu_ready_us` before attitude interpolation
- two independent wall captures both showed a broad best region around `-20` to `-28 ms`, with `-24 ms` the lowest-RMS point on the 2 ms scan grid

The `-24 ms` value is scoped to this assembly/configuration and must not be treated as a universal VL53L5CX constant.

## Continuous wall capture 1

`capture_20260905_000517Z_phase3-rotation-in-place-wall`

- packets SHA-256: `69a2a48fedb65b58b3e8adb21c0ab820ff802cedd68dfbb996fa33d7f6220b67`
- bad frames: `0`
- sequence gaps: `0`
- acquisition health deltas: all zero
- orientation excursion: `38.503 deg`
- usable wall observations: `443 / 452` (`98.0%`)

With calibrated `-24 ms` timing:

- residual median: `0.464 deg`
- residual RMS: `0.776 deg`
- residual p95: `1.664 deg`
- residual max: `3.530 deg`
- start/end wall-normal delta: `0.703 deg`

The same numbers were reproduced through the versioned calibrated timing-artifact resolver, without supplying the offset numerically.

## Continuous wall capture 2

`capture_20260905_005203Z_phase3-rotation-in-place-wall-2`

- packets SHA-256: `eeec92bad3b015760d70a3ec8c163a96789eec91eaa61da78b8c284a0e284971`
- bad frames: `0`
- sequence gaps: `0`
- acquisition health deltas: all zero
- orientation excursion: `28.412 deg`
- usable wall observations: `452 / 453` (`99.8%`)

With calibrated `-24 ms` timing:

- residual median: `0.367 deg`
- residual RMS: `0.637 deg`
- residual p95: `1.222 deg`
- residual max: `1.859 deg`
- start/end wall-normal delta: `0.151 deg`

This capture also reproduced the earlier explicit-offset result exactly through calibrated artifact resolution.

## Frozen Phase 3 reference wall gate

The acceptance limits are deliberately rounded beyond the worse of the two compensated runs rather than copied from the best case:

```text
stream/health integrity:       clean
configured gyro range:         PASS
usable wall observations:      >= 95%
orientation excursion:         >= 20 deg
wall-normal residual RMS:      <= 1.0 deg
wall-normal residual p95:      <= 2.0 deg
wall-normal residual maximum:  <= 5.0 deg
start/end wall-normal delta:   <= 1.0 deg
```

Why these values:

- `RMS <= 1.0 deg` leaves meaningful headroom over the worse measured `0.776 deg` run while preserving a genuinely sub-degree RMS target.
- `p95 <= 2.0 deg` is above the worse measured `1.664 deg` tail and is the main robust transient bound.
- `max <= 5.0 deg` is intentionally looser than p95 so one noisy ToF frame does not make the validation brittle; the worse measured maximum was `3.530 deg`.
- `start/end <= 1.0 deg` provides margin over the worse measured `0.703 deg` return consistency.
- `>=95%` usable observations ensures the result cannot look good by discarding a large share of the capture; the reference runs retained `98.0%` and `99.8%`.
- `>=20 deg` excursion prevents a nearly stationary recording from satisfying the gate trivially; the reference runs exercised `38.503 deg` and `28.412 deg`.

These bounds validate the current reference estimation path. They may be revised when independent builds provide enough cross-unit evidence, but revisions must remain evidence-based and must not silently redefine old validation results.

The executable gate is `host/python/validate_phase3_wall.py`; the pure numeric contract is in `host/python/rangeweave_phase3_gate.py`.
