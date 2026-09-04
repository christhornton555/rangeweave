# Documentation

Start here:

- [Build guide](build-guide.md) - reproduce and self-test the validated breadboard sensor stack.
- [Project plan](project-plan.md) - living architecture/roadmap and current phase status.
- [Orientation estimation plan](orientation-estimation.md) - Phase 3 attitude-estimation work package and rotation-in-place validation plan.
- [Attitude conventions](attitude-conventions.md) - frozen Phase 3 `local_reference`, quaternion, composition, gyro and gravity semantics.
- [Boresight calibration](boresight-calibration.md) - validated P0-P5 physical ToF-to-device-body alignment workflow, including the recommended wall/3-axis-mount setup, held-out P5 check and artifact generation.
- [Calibration model](calibration.md) - how timing, sensor intrinsics, assembly extrinsics and runtime state are kept separate.
- [Optional ToF intrinsic known-plane workflow](tof-calibration-plane-workflow.md) - per-zone VL53L5CX ray/geometry refinement; distinct from boresight calibration.
- [ToF/body extrinsics](tof-body-extrinsics.md) - rotational contract, physical reference-rig IMU mapping, fixed-plane solver and held-out validation contract.
- [Coordinate frames](coordinate-frames.md) - frozen optical/body conventions and current local/world-frame status.
- [Validation](validation/README.md) - reference self-test evidence and reproduction reporting.
- [Boresight physical evidence](validation/boresight-reference-rig-2026-09.md) - reference-rig axis, gyro-range, settling, P0-P5 and held-out-validation summary.
- [Calibration artifacts](../calibration/README.md) - generated per-device `rangeweave.tof-body-rotation` artifacts and provenance policy.
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

Phase 3 orientation is now active. Project-owned attitude/quaternion conventions are frozen and the first deterministic gyro+gravity Python core/replay inspector exists as an **unvalidated candidate**. The next evidence step is replaying the existing clean boresight motion captures, followed by one continuous rotation-in-place wall capture. Magnetic heading follows only after LIS3MDL body mapping/calibration and disturbance gating.

## Calibration terminology

Two physical workflows are easy to confuse:

1. **ToF intrinsic/ray calibration** estimates per-zone geometry inside `tof_optical` and can require measured known-plane geometry.
2. **ToF/body boresight calibration** estimates one rigid `R_body_from_tof` rotation for the assembled sensing head. Its preferred target is simply a fixed clear wall; the wall's absolute room orientation is not measured.

Use the dedicated documents above rather than treating the older phrase "calibration board" as a requirement for every calibration task.
