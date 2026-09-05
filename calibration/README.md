# Rangeweave calibration artifacts

This directory is for small, versioned calibration artifacts that are deliberately retained with the repository or copied into a deployment for a specific physical sensing head.

Raw calibration captures remain outside Git by default under `captures/`; promoted artifacts should therefore embed enough provenance to identify and replay the source evidence independently.

Rangeweave supports two first-class operating styles:

- **quick-start** uses matched nominal/reference profiles where available and otherwise falls back conservatively with visible `nominal-fallback` / `uncalibrated` status;
- **calibrated** uses optional guided per-build measurements to generate more accurate artifacts for the physical assembly.

Calibration is modular. A builder may calibrate boresight and timing while leaving ToF intrinsics nominal, for example.

## ToF/body rotational boresight

Generate a validated per-device artifact from a successful guided P0-P5 session with:

```powershell
py host/python/generate_boresight_artifact.py <session-prefix>
```

The default output is:

```text
calibration/tof-body-rotation-<session-prefix>.json
```

The JSON schema is `rangeweave.tof-body-rotation` version 1. A calibrated artifact records the exact `tof_optical -> device_body` rotation matrix plus calibration/held-out-validation diagnostics, capture SHA-256 hashes, `STREAM_INFO`, IMU mapping role, gyro-range provenance, quality gates and the ToF geometry profile used by the solve.

The validated September 2026 reference-rig artifact is retained as [`tof-body-rotation-boresight-guided-20260904_010944.json`](tof-body-rotation-boresight-guided-20260904_010944.json). It belongs to that one physical sensing head and is reference evidence, not a universal rotation to copy to another independently assembled unit.

Do not copy one physical unit's calibrated rotation into another independently assembled unit. The identity `nominal-fallback` remains the uncalibrated default.

See [`../docs/boresight-calibration.md`](../docs/boresight-calibration.md) and [`../docs/validation/boresight-reference-rig-2026-09.md`](../docs/validation/boresight-reference-rig-2026-09.md).

## ToF/IMU timing alignment

Protocol v0.1 records `mcu_ready_us` when the producer observes VL53L5CX data-ready. The effective observation time represented by a grid can be earlier than that readiness timestamp, so continuous-motion ToF/IMU alignment is treated as a calibration/profile quantity rather than a universal sensor-model constant.

The JSON schema is `rangeweave.tof-time-alignment` version 1. It records:

- the effective offset added to `mcu_ready_us` before attitude interpolation;
- the sign convention and timestamp field;
- a timing-relevant producer/ToF/protocol applicability fingerprint;
- for calibrated artifacts, a builder/deployment-managed physical `assembly_id`;
- source capture hashes, scan/plateau evidence and quality metrics;
- `calibrated` or `nominal-fallback` role.

The September 2026 reference rig has a retained per-build artifact at [`tof-time-alignment-reference-rig-20260905.json`](tof-time-alignment-reference-rig-20260905.json). Two independent wall-motion captures support `-24 ms` for that assembly/configuration. **That value is not a universal VL53L5CX default.**

The quick-start reference profile currently remains a conservative `0 ms` nominal until cross-device evidence is available. An unmatched build also falls back to `0 ms`, but is reported as `uncalibrated` rather than nominal.

Calibrated replay requires the artifact and its matching assembly identity, for example:

```powershell
py host/python/inspect_orientation_wall.py <capture> `
  --boresight-artifact calibration\tof-body-rotation-boresight-guided-20260904_010944.json `
  --timing-mode calibrated `
  --timing-artifact calibration\tof-time-alignment-reference-rig-20260905.json `
  --timing-assembly-id reference-rig-2026-09
```

See [`../docs/timing-calibration.md`](../docs/timing-calibration.md) for the resolver, quick-start/calibrated behaviour and artifact contract.
