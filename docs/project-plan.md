# Rangeweave — Project Plan and Architecture Roadmap

**Living project plan. Git history and validation records are the authoritative change history.**

## Project intent

Develop a small, inexpensive, RGB-free sensing module that combines sparse active depth and inertial sensing to estimate motion and build 3D representations of objects/environments. The architecture must remain portable beyond the current Pico 2 W prototype, support replay and future PC/Android implementations, and keep raw evidence available for later algorithm improvements.

## Current validated baseline

- Raspberry Pi Pico 2 W running MicroPython.
- LSM6DSOX + LIS3MDL on I2C0 (GP4/GP5, 400 kHz).
- VL53L5CX on I2C1 (GP2/GP3, 1 MHz), 8x8 at ~15 Hz.
- LSM6DSOX accel/gyro FIFO with hardware timestamps.
- Per-unit `INTERNAL_FREQ_FINE` discovery and recorded Pico<->LSM clock correlation.
- LIS3MDL deterministic address `0x1E` via `ADM -> 3V3`.
- Reproducibility self-test can reach `SYSTEM READY: PASS` on the reference stack.
- Hardware-validated Rangeweave v0.1 binary acquisition producer over USB CDC with explicit loss/health reporting.
- Canonical Python capture/replay and stream-inspection tooling.
- Physically validated VL53L5CX producer-to-image orientation and nominal 64-zone projection convention.
- Raw/temporal depth viewing and sparse point-cloud inspection.
- Physical reference-rig LSM6DSOX axis mapping into `device_body`.
- Short-baseline relative-body rotation with independent accelerometer gravity closure.
- Acquisition gyro moved to +/-500 deg/s after physical calibration testing exposed the old +/-250 deg/s range limit.
- Fixed-plane ToF/body boresight sequence/solver has produced a plausible provisional four-pose physical result.
- Boresight motion and stationary-ToF quality gates are implemented.

The current focus is finishing the physical ToF/body calibration workflow cleanly, then promoting a versioned per-device extrinsic before moving deeper into orientation/odometry.

## Non-negotiable design principles

1. **Privacy by capability.** The core sensing path does not require RGB.
2. **Cheap first, small later.** Development boards are acceptable; architecture must remain portable.
3. **No display-SDK dependency.** Tracking is a sensing problem, not a headset-API feature.
4. **No installed beacons as a core requirement.**
5. **Keep raw data.** Avoid irreversible MCU preprocessing.
6. **Use explicit timestamps, not assumed sample rates.**
7. **Represent uncertainty/failure.** Poor observability is a valid result.
8. **Physics first; ML augments.** Learned corrections must be bounded and measurable against conventional baselines.
9. **Replay everything.** Live and recorded data should enter equivalent host paths.
10. **Transport independence.** USB now; other transports later without changing record meaning.
11. **Cross-platform by construction.** Python is the reference implementation; protocol/calibration conventions must be portable.
12. **Keep intrinsics, rigid extrinsics and runtime state separate.**
13. **Keep public documentation aligned with implementation and physical evidence.**

## Layered architecture

```text
SENSORS
  LSM6DSOX     LIS3MDL      sparse ToF
      \            |            /
       SENSOR-ACQUISITION LAYER
        raw samples + timestamps
                  |
          PACKET / RECORD MODEL
       versioned, language-neutral
                  |
          TRANSPORT ADAPTER
     USB now | BLE/Wi-Fi later
                  |
       HOST CAPTURE / REPLAY API
          PC Python | Android
                  |
        GEOMETRY / ESTIMATION CORE
 orientation -> rays -> pose -> map
                  |
        VISUALISATION / APPLICATIONS
```

The packet/record model is the portability boundary. Sensor semantics must not depend on Pico GPIOs, MicroPython object types or USB-specific timing.

## Calibration architecture

Keep four categories distinct:

- **timing calibration** — per-unit clock/timestamp properties discovered automatically;
- **sensor intrinsics** — ToF rays/range bias, IMU scale/bias, magnetic hard/soft iron;
- **assembly extrinsics** — rigid rotations/translations between `tof_optical`, `imu_sensor`, `mag_sensor` and `device_body`;
- **runtime estimator state** — pose, covariance, online gyro bias, magnetic confidence and map state.

Two current ToF workflows must not be conflated:

### Optional VL53L5CX intrinsic/ray calibration

Uses measured known planes to refine per-zone geometry inside `tof_optical`. A portable board or another measurable plane may be appropriate. See [`tof-calibration-plane-workflow.md`](tof-calibration-plane-workflow.md).

### ToF/body boresight calibration

Uses a **single fixed plane** plus relative IMU body rotations to estimate one rigid `R_body_from_tof` rotation. The wall's absolute room orientation is a nuisance parameter and is not measured.

Recommended builder setup is now:

- clear flat wall patch at least ~1 m x 1 m;
- P0 roughly 500 mm from the wall and approximately square-on;
- rigid sensing head on a 3-way photographic pan/tilt/roll head or equivalent;
- moderate multi-axis pose changes rather than exact measured commanded angles.

Current empirical boresight gates:

```text
relative motion >= 5 deg
gyro warning / reject = 80% / 90% configured full scale
gyro/gravity closure <= 2 deg
ToF plane RMS <= 10 mm
ToF max plane residual <= 30 mm
ToF max half-capture drift <= 10 mm
```

These are workflow safeguards derived from reference-rig data, not universal sensor specifications.

## Development phases

| Phase | Current state | Exit gate |
|---|---|---|
| 0. Sensor-stack validation | **DONE / BASELINE** | Fresh reference assembly can reach `SYSTEM READY: PASS`. |
| 1. Protocol + Pico acquisition + PC capture/replay | **DONE on reference path** | Live stream is loss-detectable and canonical captures replay through the same parser. |
| 1A. Android protocol parity | **PLANNED** | Kotlin decodes shared fixtures/captures consistently with Python. |
| 1B. MCU portability spike | **PLANNED** | Alternate MCU producer preserves protocol semantics. |
| 2. Raw viewer + nominal sparse geometry | **IMPLEMENTED / PHYSICALLY EXERCISED** | Real wall/object captures project with the documented orientation/depth convention. |
| 2A. Optional ToF intrinsic calibration | **IMPLEMENTED CANDIDATE** | Repeatable per-device geometry profile with held-out validation/provenance. |
| 2B. ToF/body rotational boresight | **PHYSICAL VALIDATION IN PROGRESS** | Guided fixed-wall workflow + held-out validation + promoted versioned artifact. |
| 3. Orientation | **NEXT MAJOR ESTIMATION LAYER** | Rotation-in-place keeps static geometry stable within quantified bounds. |
| 4. Full calibration suite | **PARTIAL** | Versioned timing/intrinsic/extrinsic artifacts are reproducible and portable. |
| 5. Known-pose scanner | **PLANNED** | Controlled motion produces consistent geometry. |
| 6. Freehand local odometry | **RESEARCH / PLANNED** | Quantified drift on repeatable trajectories with uncertainty. |
| 7. Persistent mapping + loop closure | **PLANNED** | Verified loop closure improves return-to-start without false closures. |
| 8. Android viewer/mapping parity | **PLANNED** | Android reproduces reference outputs within tolerance. |
| 9. Compact sensor node | **PLANNED** | Smaller MCU/transport options retain data integrity. |
| 10. Multi-ToF / wearable variants | **FUTURE** | Additional sensors measurably improve coverage/robustness. |

## Immediate work package

Current priority order:

1. finish the builder-facing guided boresight command so it owns capture warm-up and HOLD / MOVE NOW / STOP MOVING cues;
2. run a fresh fixed-wall sequence under the current +/-500 deg/s acquisition configuration using the improved multi-axis fixture;
3. reserve a geometrically different pose as held-out validation;
4. promote a per-device `rangeweave.tof-body-rotation` artifact with capture/firmware/geometry provenance;
5. update PR/documentation and merge Phase 2 only after the physical workflow is internally consistent;
6. then proceed to the broader orientation layer (gyro + gravity, later confidence-gated magnetometer) and static-geometry validation.

Parallel lower-priority portability work remains Android protocol parity and an alternate-MCU producer spike.

## Testing strategy

- protocol unit/corruption/resynchronisation tests;
- cross-language fixture tests when Kotlin implementation begins;
- replay regression tests using deliberately published small recordings where appropriate;
- hardware self-test for every physical build;
- fixed-wall and known-plane calibration tests;
- held-out calibration observations rather than fitting every captured pose;
- repeatable rotation/translation/return-to-start trajectories;
- explicit health, range, temporal-stability and observability gates.

## Public claims policy

### Supported on the current reference rig

- sparse ToF and inertial acquisition can coexist under the tested workload without the previously observed FIFO/transport loss conditions;
- per-unit IMU timing is measured rather than assumed;
- protocol v0.1 acquisition/capture/replay works over the reference USB path;
- physical ToF zone orientation and the nominal axial-Z projection convention are established;
- the reference-rig LSM6DSOX axis mapping and short-baseline relative-rotation estimator have physical evidence;
- fixed-plane ToF/body boresight is physically feasible and has produced a plausible provisional result.

### Experimental / not yet promoted

- optional per-device ToF intrinsic calibration;
- final per-device ToF/body boresight artifact;
- any workflow currently requiring developer CLI orchestration rather than the planned guided command.

### Not yet supported

- reliable freehand SLAM or dense/accurate room reconstruction;
- universal calibration parameters across arbitrary third-party assemblies;
- completed Android/BLE/Wi-Fi/ESP32 production implementations;
- blanket privacy/anonymity claims based only on absence of RGB.

## Frozen/current decisions

- sparse active ToF remains the primary range modality;
- reference hardware uses one VL53L5CX + one LSM6DSOX/LIS3MDL board on split I2C buses;
- ADM is tied high for LIS3MDL address `0x1E`;
- IMU timing comes from hardware FIFO timestamps plus measured clock correlation;
- acquisition gyro currently uses +/-500 deg/s for calibration headroom;
- Pico 2 W is a reference controller, not a protocol dependency;
- USB-to-PC is the first transport;
- calibration keeps ToF intrinsic rays separate from rigid ToF/body boresight;
- no installed beacons or display-SDK dependency in the core design;
- ML remains an augmentation to an uncertainty-aware physics/geometry baseline.

## Future-session checklist

Before accepting a design change, ask:

- Does it change sensor-data meaning? Version the protocol/calibration convention.
- Does non-platform code know it is on Pico/USB? Refactor behind an adapter.
- Are timestamps explicit rather than inferred?
- Can the same experiment be replayed from raw data?
- Are calibration intrinsics/extrinsics/runtime state kept separate?
- Is failure/uncertainty visible?
- Is the claim validated, experimental or planned?
- Does the change make later Android/alternate-MCU parity harder?
