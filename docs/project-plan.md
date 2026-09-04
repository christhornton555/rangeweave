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
- Guided fixed-plane ToF/body boresight capture runs the recommended P0-P5 sequence.
- Boresight motion and stationary-ToF quality gates are implemented.
- The reference-rig P0-P5 workflow passed held-out validation: P5 prediction error 0.644 deg and 0.639 deg final-fit rotation change after revealing P5.
- A provenance-rich `rangeweave.tof-body-rotation` artifact generator is implemented.
- The exact validated reference-rig boresight artifact is promoted under `calibration/` and Phase 2 PR #10 is merged.

**Current focus: Phase 3 orientation estimation.** Project-owned `local_reference`/quaternion/body-rate conventions are now frozen and golden-tested. A first deterministic gyro+gravity Python core and replay inspector are implemented as an **unvalidated candidate**. The immediate evidence step is replaying the existing clean boresight motion captures before gathering a new continuous rotation-in-place wall sequence.

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

Two ToF workflows must not be conflated:

### Optional VL53L5CX intrinsic/ray calibration

Uses measured known planes to refine per-zone geometry inside `tof_optical`. A portable board or another measurable plane may be appropriate. See [`tof-calibration-plane-workflow.md`](tof-calibration-plane-workflow.md).

### ToF/body boresight calibration

Uses a **single fixed plane** plus relative IMU body rotations to estimate one rigid `R_body_from_tof` rotation. The wall's absolute room orientation is a nuisance parameter and is not measured.

Recommended builder setup is:

- clear flat wall patch at least ~1 m x 1 m;
- P0 roughly 500 mm from the wall and approximately square-on;
- rigid sensing head on a 3-way photographic pan/tilt/roll head or equivalent;
- full P0-P5 guided sequence with moderate multi-axis pose changes rather than exact measured commanded angles;
- P5 ToF held out from the P0-P4 fit before the final six-pose refit.

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

The September 2026 reference run used the built-in ToF geometry profile with role `nominal-fallback`; that provenance is recorded explicitly rather than implying intrinsic per-zone calibration.

## Development phases

| Phase | Current state | Exit gate |
|---|---|---|
| 0. Sensor-stack validation | **DONE / BASELINE** | Fresh reference assembly can reach `SYSTEM READY: PASS`. |
| 1. Protocol + Pico acquisition + PC capture/replay | **DONE on reference path** | Live stream is loss-detectable and canonical captures replay through the same parser. |
| 1A. Android protocol parity | **PLANNED** | Kotlin decodes shared fixtures/captures consistently with Python. |
| 1B. MCU portability spike | **PLANNED** | Alternate MCU producer preserves protocol semantics. |
| 2. Raw viewer + nominal sparse geometry | **DONE on reference path** | Real wall/object captures project with the documented orientation/depth convention. |
| 2A. Optional ToF intrinsic calibration | **IMPLEMENTED / NOT YET PHYSICALLY PROMOTED** | Repeatable per-device geometry profile with held-out validation/provenance. |
| 2B. ToF/body rotational boresight | **DONE / REFERENCE ARTIFACT PROMOTED** | Guided P0-P5 workflow, held-out P5 validation, artifact generation/provenance and merged documentation. |
| 3. Orientation | **IN IMPLEMENTATION / REPLAY VALIDATION NEXT** | Rotation-in-place keeps static geometry stable within quantified bounds using a replayable attitude estimator with frozen conventions and visible confidence. |
| 4. Full calibration suite | **PARTIAL** | Versioned timing/intrinsic/extrinsic artifacts are reproducible and portable, including magnetic and any required rigid translation calibration. |
| 5. Known-pose scanner | **PLANNED** | Controlled motion produces consistent geometry. |
| 6. Freehand local odometry | **RESEARCH / PLANNED** | Quantified drift on repeatable trajectories with uncertainty. |
| 7. Persistent mapping + loop closure | **PLANNED** | Verified loop closure improves return-to-start without false closures. |
| 8. Android viewer/mapping parity | **PLANNED** | Android reproduces reference outputs within tolerance. |
| 9. Compact sensor node | **PLANNED** | Smaller MCU/transport options retain data integrity. |
| 10. Multi-ToF / wearable variants | **FUTURE** | Additional sensors measurably improve coverage/robustness. |

## Immediate work package: Phase 3 orientation

Detailed plan: [`orientation-estimation.md`](orientation-estimation.md). Normative attitude semantics: [`attitude-conventions.md`](attitude-conventions.md).

Priority order:

1. **DONE — freeze attitude conventions.** Scalar-first Hamilton `(w,x,y,z)`, active `R_reference_from_body`, body-frame angular-rate propagation, gravity/specific-force semantics and local yaw-zero are documented and golden-tested.
2. **IMPLEMENTED CANDIDATE — six-axis Python baseline.** Timestamp-driven gyro integration, fixed explicit-startup bias estimate, confidence-gated gravity correction, diagnostics and deterministic replay are implemented but not yet physically promoted.
3. **NEXT — replay existing clean boresight motions.** Compare persistent start-to-end orientation against the already validated hold-move-hold relative estimator for sign/composition/gravity regression evidence.
4. **Then capture one new continuous multi-axis rotation-in-place sequence** against a fixed wall/static scene.
5. **Validate orientation using static geometry** — apply the promoted `R_body_from_tof` plus estimated attitude and measure how constant the same wall normal remains. Use observed evidence to set acceptance bounds rather than inventing them in advance.
6. **Add orientation-aware ToF viewing/projection** once the attitude path passes the rotation-only test.
7. **Then tackle magnetic heading** — physically map `mag_sensor -> device_body`, calibrate hard/soft iron, characterize disturbances and add confidence-gated heading correction.
8. Only after orientation is stable, proceed to translation/6DoF pose, known-pose scanning and freehand odometry.

Parallel lower-priority portability work remains Android protocol parity and an alternate-MCU producer spike. Optional per-device ToF intrinsic calibration also remains a separate experimental branch of work rather than a blocker for Phase 3.

## Phase 3 validation concept

The first orientation validation intentionally avoids needing an externally measured camera angle.

Keep the sensor approximately at one location and rotate it in front of a fixed wall. Small positional shifts from the photographic head are acceptable because a plane normal is translation-invariant. For every usable ToF observation:

```text
n_tof
  -> R_body_from_tof
  -> estimated local/reference orientation
  -> n_local
```

The same physical wall should then have approximately the same `n_local` throughout the rotation. This gives a direct end-to-end test of timestamps, body-axis mapping, gyro integration, gravity correction, quaternion composition and boresight use.

## Testing strategy

- protocol unit/corruption/resynchronisation tests;
- cross-language fixture tests when Kotlin implementation begins;
- replay regression tests using deliberately published small recordings where appropriate;
- hardware self-test for every physical build;
- fixed-wall and known-plane calibration tests;
- held-out calibration observations rather than fitting every captured pose without validation;
- golden attitude/quaternion composition/body-rate tests;
- repeatable rotation-in-place tests with static-geometry residuals;
- later repeatable translation/return-to-start trajectories;
- explicit health, range, temporal-stability, confidence and observability gates.

## Public claims policy

### Supported on the current reference rig

- sparse ToF and inertial acquisition can coexist under the tested workload without the previously observed FIFO/transport loss conditions;
- per-unit IMU timing is measured rather than assumed;
- protocol v0.1 acquisition/capture/replay works over the reference USB path;
- physical ToF zone orientation and the nominal axial-Z projection convention are established;
- the reference-rig LSM6DSOX axis mapping and short-baseline relative-rotation estimator have physical evidence;
- fixed-plane ToF/body boresight is physically validated through a P0-P5 sequence with held-out P5 prediction and stable six-pose refit;
- the exact reference-rig `rangeweave.tof-body-rotation` artifact is promoted with provenance.

### Experimental / not yet generalized

- optional per-device ToF intrinsic calibration;
- boresight reproducibility across independently assembled units;
- universal numeric acceptance bounds for held-out/final-fit stability across arbitrary fixtures and sensor assemblies;
- the new persistent six-axis `local_reference` attitude implementation until replay and rotation-in-place validation are complete.

### Not yet supported

- reliable freehand SLAM or dense/accurate room reconstruction;
- globally referenced heading without validated magnetometer/body calibration and magnetic confidence handling;
- universal calibration parameters across arbitrary third-party assemblies;
- completed Android/BLE/Wi-Fi/ESP32 production implementations;
- blanket privacy/anonymity claims based only on absence of RGB.

## Frozen/current decisions

- sparse active ToF remains the primary range modality;
- reference hardware uses one VL53L5CX + one LSM6DSOX/LIS3MDL board on split I2C buses;
- ADM is tied high for LIS3MDL address `0x1E`;
- IMU timing comes from hardware FIFO timestamps plus measured clock correlation;
- acquisition gyro currently uses +/-500 deg/s for calibration/head-motion headroom;
- Pico 2 W is a reference controller, not a protocol dependency;
- USB-to-PC is the first transport;
- calibration keeps ToF intrinsic rays separate from rigid ToF/body boresight;
- the builder boresight sequence is P0-P5 with P5 held out before the final refit;
- the reference-rig exact boresight matrix is stored as a per-device artifact rather than a universal constant;
- Phase 3 attitude is scalar-first Hamilton `q_reference_from_body=(w,x,y,z)` with an active body-to-local transform and body-frame right-multiplied gyro increments;
- `local_reference` has gravity-down `+Y` and a deterministic initial-body-derived yaw zero, but no global heading claim;
- the first persistent orientation layer is gyro+gravity and must remain useful without magnetometer heading;
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
