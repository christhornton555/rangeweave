# Documentation

Start here:

- [Build guide](build-guide.md) - reproduce and self-test the validated breadboard sensor stack.
- [Project plan](project-plan.md) - living architecture/roadmap and current phase status.
- [Orientation estimation plan](orientation-estimation.md) - Phase 3 attitude-estimation work package and rotation-in-place validation plan.
- [Attitude conventions](attitude-conventions.md) - frozen Phase 3 `local_reference`, quaternion, composition, gyro and gravity semantics.
- [Boresight calibration](boresight-calibration.md) - validated P0-P5 physical ToF-to-device-body alignment workflow, including the recommended wall/3-axis-mount setup, held-out P5 check and artifact generation.
- [ToF timing calibration](timing-calibration.md) - quick-start versus calibrated ToF/IMU timing resolution, per-build artifacts, applicability fingerprints and the reference-rig `-24 ms` evidence.
- [Calibration model](calibration.md) - how timing, sensor intrinsics, assembly extrinsics and runtime state are kept separate.
- [Optional ToF intrinsic known-plane workflow](tof-calibration-plane-workflow.md) - per-zone VL53L5CX ray/geometry refinement; distinct from boresight calibration.
- [ToF/body extrinsics](tof-body-extrinsics.md) - rotational contract, physical reference-rig IMU mapping, fixed-plane solver and held-out validation contract.
- [Coordinate frames](coordinate-frames.md) - frozen optical/body conventions and current local/world-frame status.
- [Validation](validation/README.md) - reference self-test evidence and reproduction reporting.
- [Boresight physical evidence](validation/boresight-reference-rig-2026-09.md) - reference-rig axis, gyro-range, settling, P0-P5 and held-out-validation summary.
- [Phase 3 orientation physical evidence](validation/orientation-reference-rig-2026-09.md) - two continuous wall captures, calibrated timing-artifact replay and the empirical reference wall gate.
- [Calibration artifacts](../calibration/README.md) - generated per-device boresight/timing artifacts and provenance policy.
- [Architecture](architecture.md) - layer boundaries and portability contract.
- [Protocol](protocol.md) - packet/record requirements and protocol status.
- [Capture format](capture-format.md) - canonical recording layout and metadata.
- [Raw depth viewer](raw-depth-viewer.md) and [temporal depth viewer](temporal-depth-viewer.md) - current visualization tooling.
- [Repository layout](repository-layout.md) - what belongs where.
- [Development history](development-history.md) - why the current split-bus/FIFO/timestamp design exists.
- [Android porting](android-porting.md) - Kotlin/Android parity notes.
- [Hardware porting](hardware-porting.md) - MCU/transport portability constraints.
- [ADRs](adr/README.md) - accepted architecture decisions.

## Current development frontier

The acquisition/protocol/capture path, nominal sparse geometry and reference-rig ToF/body rotational boresight are complete on the reference stack. The exact boresight artifact is promoted under `calibration/`.

Phase 3 orientation is active. Project-owned attitude/quaternion conventions are frozen; the deterministic gyro+gravity estimator has passed five retained hold-move-hold regressions and two independent continuous rotation-in-place wall captures. Both captures reproduce their calibrated `-24 ms` timing result through the versioned per-build timing resolver rather than a hard-coded offset.

An empirical Phase 3 reference wall gate is now encoded and documented: clean stream/health, gyro-range PASS, at least 95% usable wall frames, at least 20 deg orientation excursion, RMS <= 1.0 deg, p95 <= 2.0 deg, max <= 5.0 deg and start/end delta <= 1.0 deg. These are project validation bounds derived from the reference evidence, not universal cross-unit sensor specifications. The next step is replaying both retained wall captures through the executable gate before proceeding to orientation-aware ToF geometry/viewing.

## Calibration terminology

Three physical workflows are easy to confuse:

1. **ToF intrinsic/ray calibration** estimates per-zone geometry inside `tof_optical` and can require measured known-plane geometry.
2. **ToF/body boresight calibration** estimates one rigid `R_body_from_tof` rotation for the assembled sensing head. Its preferred target is simply a fixed clear wall; the wall's absolute room orientation is not measured.
3. **ToF/IMU timing calibration** estimates the effective observation-time offset relative to protocol `mcu_ready_us` during motion and is scoped to the physical build/acquisition configuration.

Use the dedicated documents above rather than treating the older phrase "calibration board" as a requirement for every calibration task.
