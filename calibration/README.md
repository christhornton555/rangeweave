# Rangeweave calibration artifacts

This directory is for small, versioned calibration artifacts that are deliberately retained with the repository or copied into a deployment for a specific physical sensing head.

Raw calibration captures remain outside Git by default under `captures/`; promoted artifacts should therefore embed enough provenance to identify and replay the source evidence independently.

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
