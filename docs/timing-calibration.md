# ToF timing calibration and profile resolution

Rangeweave protocol v0.1 timestamps each `TOF_GRID` with `mcu_ready_us`: the MCU time when the producer observes VL53L5CX data-ready. That timestamp is deterministic and useful for replay, but it is not guaranteed to be the sensor-internal instant represented by the ranging result.

For continuous motion, a small effective ToF/IMU alignment error can therefore appear as an angular error even when the orientation estimate and ToF/body boresight are otherwise correct.

## Product rule: quick-start and calibrated modes

Rangeweave treats calibration as optional refinement rather than a prerequisite for basic operation.

### Quick-start mode

Quick-start is the default software path.

- No physical calibration is required.
- The resolver uses a project-supplied `nominal-fallback` timing profile only when the capture configuration matches that profile.
- If no profile matches, timing falls back to `0 ms` and the state is reported as `uncalibrated`.
- The software must expose the resulting role instead of silently claiming calibration.
- A nominal profile may later contain a non-zero population value, but only after evidence across multiple physical builds supports it.

The current Pico 2W / LSM6DSOX / LIS3MDL / VL53L5CX 8x8@15 Hz reference profile deliberately retains a conservative nominal `0 ms` timing offset. The measured `-24 ms` value belongs to one physical reference assembly and is not promoted as the cross-device nominal.

### Calibrated mode

Calibrated mode uses a per-build `rangeweave.tof-time-alignment` artifact.

A calibrated artifact contains:

- the effective offset applied to `mcu_ready_us` before attitude interpolation;
- an explicit sign convention;
- a configuration applicability fingerprint;
- a builder/deployment-managed `assembly_id` for the physical sensing head;
- source capture hashes and replay evidence;
- the offset scan range and supported optimum/plateau evidence;
- metadata describing how the artifact was generated.

Protocol v0.1 does not expose a stable physical ToF-module serial, so `assembly_id` is intentionally external to the wire protocol. This prevents a calibrated artifact for one physical build being silently accepted on another build whose electronic configuration happens to look identical.

A calibrated artifact is rejected if any fingerprinted field no longer matches. Material firmware, source-profile, ToF-mode, protocol or assembly changes therefore require either a new calibration or an explicit user override. Replacing the ToF module, IMU, controller, or another timing-critical part of the sensing head should be treated as a new calibration identity even if the replacement reports the same model/configuration strings; update the deployment-managed `assembly_id` (or otherwise invalidate the old artifact) and recalibrate rather than silently inheriting the previous unit's timing value.

## Runtime precedence

Timing resolution follows this order:

```text
explicit --tof-time-offset-ms override
        |
        v
matched per-build calibrated artifact   (calibrated mode)
        |
        v
matched nominal timing profile          (quick-start mode)
        |
        v
0 ms + visible uncalibrated status
```

An explicit numeric override is never labelled as calibrated evidence.

## Artifact schema

The v1 schema name is:

```text
rangeweave.tof-time-alignment
```

Important top-level fields are:

```json
{
  "schema": "rangeweave.tof-time-alignment",
  "schema_version": 1,
  "name": "example-build-timing",
  "role": "calibrated",
  "timestamp_field": "mcu_ready_us",
  "offset_sign_convention": "offset_added_to_mcu_ready_us_before_attitude_interpolation",
  "effective_offset_ms": -24.0,
  "applies_to": {
    "assembly_id": "example-build",
    "protocol": {"major": 0, "minor": 1},
    "stream_info": {
      "firmware_label": "...",
      "source_profile": "...",
      "tof_grid": {"rows": 8, "cols": 8, "hz": 15}
    }
  },
  "evidence": {},
  "metadata": {}
}
```

`role` is `calibrated` for a per-build artifact or `nominal-fallback` for a reusable quick-start profile. `uncalibrated` is a resolver state, not a fake calibration artifact.

## Current reference-rig evidence

Two independent 30 s continuous wall captures on the September 2026 reference rig produced a broad timing optimum around `-20` to `-28 ms`. `-24 ms` was the lowest-RMS 2 ms grid point in both captures.

At `-24 ms`:

| capture | excursion | residual RMS | p95 | max | start/end delta |
|---|---:|---:|---:|---:|---:|
| wall 1 | 38.503 deg | 0.776 deg | 1.664 deg | 3.530 deg | 0.703 deg |
| wall 2 | 28.412 deg | 0.637 deg | 1.222 deg | 1.859 deg | 0.151 deg |

The retained artifact is [`../calibration/tof-time-alignment-reference-rig-20260905.json`](../calibration/tof-time-alignment-reference-rig-20260905.json). It is calibrated for `assembly_id=reference-rig-2026-09` and the fingerprinted acquisition configuration only.

## CLI use

Quick-start is the default:

```powershell
py host/python/inspect_orientation_wall.py <capture> `
  --boresight-artifact <boresight.json>
```

For the retained reference-rig calibrated replay:

```powershell
py host/python/inspect_orientation_wall.py <capture> `
  --boresight-artifact calibration\tof-body-rotation-boresight-guided-20260904_010944.json `
  --timing-mode calibrated `
  --timing-artifact calibration\tof-time-alignment-reference-rig-20260905.json `
  --timing-assembly-id reference-rig-2026-09
```

The report prints `timing mode`, `timing role`, source/profile name and effective offset so quick-start, calibrated and explicit-override results cannot be confused.

## Guided calibration direction

The eventual builder-facing calibrated workflow should remain short and modular. Timing calibration can reuse the same kind of smooth rotation-against-flat-wall capture already used for Phase 3 validation, scan a physically plausible offset range, require repeatable evidence across at least two independent movements, and generate the artifact automatically.

This timing step should sit alongside, not replace, the other optional guided calibration modules:

1. stationary IMU initialization/quality check;
2. multi-pose ToF/body rotational boresight;
3. smooth wall rotation for ToF/IMU timing alignment;
4. optional known-plane per-zone ToF correction;
5. later magnetometer mapping/calibration;
6. automatic verification and artifact summary.

A builder may stop after any useful subset and continue operating with nominal profiles for the remaining quantities.
