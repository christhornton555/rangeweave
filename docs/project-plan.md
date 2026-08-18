# Privacy-First Sparse-ToF Spatial Mapping

**Living project plan and architecture roadmap. Initial public baseline derived from v0.2 (18 August 2026).**

> From this repository onward, Git history and tagged releases supersede version numbers in documentation filenames.

## Project intent

Develop a small, inexpensive, privacy-preserving sensing module that combines sparse active depth and inertial sensing to estimate motion and build 3D representations of objects and environments. The architecture must remain portable beyond the Raspberry Pi Pico prototype, support PC and Android hosts, and be suitable for eventual open-source publication.

## Current validated baseline

- Raspberry Pi Pico 2 W running MicroPython.
- LSM6DSOX + LIS3MDL on I2C0 (GP4/GP5, 400 kHz).
- VL53L5CX on I2C1 (GP2/GP3, 1 MHz), 8×8 at ~15 fps.
- LSM6DSOX accel/gyro hardware FIFO with one hardware timestamp per inertial slot.
- Per-unit `INTERNAL_FREQ_FINE` discovery and rolling Pico↔LSM clock model; no hard-coded real ODR.
- LIS3MDL address made deterministic at `0x1E` by tying ADM high.
- Reference v0.5 reproducibility self-test: **SYSTEM READY: PASS**.

The diagnostic firmware is now a baseline. The next firmware is an acquisition/transport implementation, not another sensor bring-up experiment.

## Non-negotiable design principles

1. **Privacy by capability.** The core mapping path does not require RGB imagery.
2. **Cheap first, small later.** Development boards are acceptable, but architecture must permit an ESP32-class or similarly small final controller.
3. **Display-independent tracking.** Do not depend on a particular AR headset SDK.
4. **No installed beacons as a core requirement.**
5. **Keep raw data.** Avoid irreversible preprocessing on the MCU.
6. **Use explicit timestamps, not assumed sample rates.**
7. **Represent uncertainty.** Poorly observable motion is a valid state.
8. **Physics first; ML augments.** Learned models provide bounded corrections/confidence rather than unchecked absolute truth.
9. **Replay everything.** Live and recorded data should enter the same host-processing interfaces.
10. **Transport independence.** USB now; Android USB, BLE/Wi-Fi or local recording later without changing packet meaning.
11. **Cross-platform by construction.** Python is the reference implementation; Kotlin/Android receives specs and golden test vectors from the beginning.
12. **Open-source readiness.** Clearly label validated, experimental and planned behaviour.

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

The packet/record model is the portability boundary. Sensor hardware and transport may change without changing the semantic data model; host platform and visualisation may change without changing firmware sensor acquisition.

## MCU portability contract

Firmware should be conceptually split into:

1. **Sensor drivers** — configure devices and return raw measurements plus native timestamps.
2. **Clock service** — MCU monotonic time and sensor↔MCU clock-correlation observations.
3. **Packetizer/queue** — versioned records, unaware of transport.
4. **Transport adapter** — USB initially; later BLE, Wi-Fi, UART or local storage.

No host parser or mapping algorithm may depend on MicroPython objects, Pico GPIO numbering, or the measured ~101 Hz rate of the reference IMU.

## Protocol and recording requirements

The protocol should be binary, versioned, language-neutral, resynchronisable, sequence-numbered, explicitly endian-defined and transport-agnostic. Firmware version, protocol version and calibration-schema version must be independent.

Initial logical records:

- **IMU:** sequence, LSM timestamp, raw accel XYZ, raw gyro XYZ.
- **MAG:** sequence, MCU timestamp, raw magnetic XYZ, quality/status.
- **TOF:** sequence, frame-observed MCU timestamp, mapped LSM timestamp, 64 ranges, 64 reflectance values, validity/status.
- **CLOCK_SYNC:** MCU time and raw LSM timestamp/correlation quality.
- **STATUS:** versions, sensor configuration/identity, `FREQ_FINE`, FIFO/error counters.

Canonical capture layout initially:

```text
capture_YYYYMMDD_HHMMSS/
  metadata.json
  packets.bin
  calibration.json
  notes.txt
```

`packets.bin` should contain the same framed packets used live so replay is not a separate data model.

## PC-first, Android-first-class

### Python reference stack

```text
host/python/
  protocol/
  capture/
  replay/
  core/
  visualization/
  mapping/
  tests/
```

Desktop libraries such as NumPy/SciPy/Open3D may be used behind project-owned interfaces; they must not define the protocol or calibration format.

### Android portability rules

- Kotlin decoder must pass the same golden packet fixtures as Python.
- Start with Android USB host/OTG for the wired prototype; BLE/Wi-Fi are later transport adapters.
- Keep parsing and mapping out of Activity/UI lifecycle code; use a streaming boundary such as coroutines/Flow.
- Define byte order, integer widths, units and coordinate frames explicitly.
- Do not add JNI/NDK merely for theoretical performance; introduce native code only after profiling.
- Every mapping milestone should leave an **Android Port Note** describing platform assumptions, Kotlin equivalents and required parity fixtures.

## Mapping pipeline

1. **Raw capture and health:** packet validation, plots, replay parity.
2. **Single-frame geometry:** calibrated 8×8 rays -> 64 sparse 3D points.
3. **Orientation:** timestamped gyro integration + accelerometer gravity + confidence-gated magnetometer.
4. **Local translation/odometry:** inertial prediction corrected by sparse-depth geometry; uncertainty retained.
5. **Mapping:** point/voxel baseline, then submaps.
6. **Loop closure:** place candidates + geometric verification + global optimisation.
7. **Learned enhancements:** bias/confidence/dynamics/relative corrections/neural map representations, evaluated against conventional baselines.

## Calibration model

Keep separate:

- **Factory/unit timing:** discovered automatically (`INTERNAL_FREQ_FINE`, measured clock model).
- **Per-sensor intrinsic calibration:** ToF zone rays/range bias, IMU scale/bias, magnetic hard/soft iron.
- **Assembly extrinsics:** rigid transforms between ToF, IMU and device-body frames.
- **Runtime estimator state:** pose, covariance, gyro bias estimate, magnetic confidence.

Use named frames (`tof_optical`, `imu_sensor`, `mag_sensor`, `device_body`, `world/local`) and version any future convention change.

## Development phases

| Phase | Status | Exit gate |
|---|---|---|
| 0. Sensor-stack validation | **DONE / BASELINE** | Fresh assembly can reach `SYSTEM READY: PASS` without per-unit hard-coded ODR. |
| 1. Protocol + PC capture/replay | **NEXT** | Live and replayed streams decode identically; packet loss is detectable. |
| 1A. Android protocol smoke test | Next after protocol stabilises | Python and Kotlin decode identical golden fixtures. |
| 1B. MCU portability spike | Early planned | ESP32-class synthetic packet source is indistinguishable at protocol level except metadata. |
| 2. Raw viewer + 64-point projection | Planned | Flat wall produces expected sparse plane / known ranges. |
| 3. Orientation | Planned | Rotation-in-place leaves static geometry directionally stable within measured bounds. |
| 4. Calibration suite | Planned | Versioned calibration is repeatable and portable across PC/Android. |
| 5. Known-pose scanner | Planned | Controlled motion produces consistent object/scene geometry. |
| 6. Freehand local odometry | Research → planned | Quantified drift on repeatable trajectories; uncertainty rises when geometry is weak. |
| 7. Persistent mapping + loop closure | Planned | Verified loop closure reduces return-to-start error without false closures. |
| 8. Android viewer/mapping parity | Planned | Android reproduces reference outputs on shared recordings within tolerance. |
| 9. Compact sensor node | Planned | ESP32-class/small MCU, BLE/Wi-Fi/local-storage options retain protocol/data integrity. |
| 10. Dual-sensor wearable | Planned | Two-ToF arrangement measurably improves coverage/robustness. |

## Immediate work package

1. Freeze [`firmware/pico2w/diagnostics/reproducible_sensor_stack.py`](../firmware/pico2w/diagnostics/reproducible_sensor_stack.py) as the diagnostic baseline. Its behaviour is derived from the validated v0.5 candidate.
2. Design protocol framing/fields before writing the acquisition loop.
3. Create byte-level golden fixtures for every initial record type.
4. Implement Pico acquisition firmware using complete raw records.
5. Implement dependency-light Python protocol decoder.
6. Implement USB receiver/recorder and replay adapter.
7. Implement Android/Kotlin parser + USB smoke test while the protocol is still small.
8. Record stationary, rotation-only, translation and simple scene datasets.
9. Build raw viewer + first 64-point ToF projection.
10. Only then begin orientation and SLAM/odometry work.

## Testing strategy

- Protocol unit tests and corruption/resync tests.
- Python↔Kotlin cross-language conformance tests.
- Replay regression tests using small checked-in golden recordings.
- Hardware self-test for every physical build.
- Bench calibration (flat wall, static IMU, controlled rotation, known-pose scanner).
- Repeatable trajectory tests (return-to-start, straight line, feature-rich/feature-poor scenes).

## Suggested open-source repository

```text
repo/
  README.md
  LICENSE
  CONTRIBUTING.md
  CHANGELOG.md
  docs/
    project-plan.md
    build-guide.md
    protocol.md
    calibration.md
    coordinate-frames.md
    android-porting.md
    hardware-porting.md
    validation/
    adr/
  firmware/
    pico2w/
      diagnostics/
      acquisition/
    esp32/
  protocol/
    test-vectors/
  host/python/
  android/
  tools/
  tests/
  sim/
  datasets/README.md
```

Use short Architecture Decision Records (ADRs) for decisions such as transport independence, raw-data preservation, hardware timestamps, diagnostic-vs-acquisition firmware separation, cross-platform test vectors, no-RGB/no-beacon core requirements, and physics-first ML augmentation.

## Public claims policy

### Supported now

- The reference prototype can acquire sparse-ToF and inertial data together under the tested workload without the observed FIFO loss/error conditions.
- The diagnostic firmware measures per-unit IMU timing instead of assuming the reference unit's real sample rate.
- The core sensor path does not require RGB imagery.

### Not yet supported

- Reliable freehand SLAM or accurate 3D reconstruction.
- Guaranteed reproducibility across arbitrary third-party assemblies until additional physical builds are tested.
- Completed Android, BLE/Wi-Fi or ESP32 implementations.
- Any claim that the system is anonymous/privacy-safe in every deployment context merely because it lacks RGB.

## Frozen decisions

- Sparse active ToF remains the primary range modality; PSD/laser triangulation is parked.
- Current baseline: one VL53L5CX + one LSM6DSOX/LIS3MDL board, split across two I2C buses.
- ADM is explicitly tied high for deterministic LIS3MDL address `0x1E`.
- LSM accel/gyro use hardware FIFO + timestamps; real timing is discovered rather than assumed.
- Pico 2 W is a reference prototype controller, not a protocol dependency.
- USB-to-PC is first transport; Android USB host is first portability target; BLE/Wi-Fi later.
- No installed beacons and no display-SDK dependency in the core design.
- Future dual IMUs are calibrated as a rigid multi-IMU system, not naively averaged.
- ML augments an uncertainty-aware physics-based estimator.

## Future-session checklist

Before accepting a design change, ask:

- Does it change sensor-data meaning? Version the protocol/calibration spec.
- Does non-platform code know it is on a Pico or USB? Refactor behind an adapter.
- Could Kotlin reproduce it from the docs and fixtures?
- Are timestamps explicit rather than inferred from record index/nominal ODR?
- Can the same experiment be replayed?
- Are raw measurements retained?
- Is uncertainty represented?
- Is the claim validated, experimental or planned?
- Does this make the eventual ESP32 or Android port harder? If so, insert an interface now.
