# Rangeweave - RGB-Free Active Depth + IMU Spatial Mapping

> **Project status:** reference sensor acquisition, protocol/capture/replay, raw/temporal depth viewing, nominal 64-zone projection and the reference-rig ToF/body rotational boresight workflow are physically validated on the current Pico 2 W stack. The exact reference boresight artifact is promoted. **Phase 3 reference orientation now has frozen attitude conventions, a deterministic gyro+gravity replay core, two independent calibrated rotation-in-place wall validations and executable PASS evidence against a frozen empirical reference wall gate.** Translation/odometry and 3D mapping remain work in progress.

Rangeweave explores a small, inexpensive **RGB-free** sensing module combining sparse time-of-flight depth with inertial sensing. The long-term goal is to turn synchronized depth + motion observations into trajectories, sparse point clouds and eventually 3D maps while keeping the sensing head compact and portable beyond the current Raspberry Pi Pico prototype.

The project is deliberately evidence-first: preserve raw measurements and source timing, validate coordinate frames and calibration physically, and only then build higher-level estimation.

## Current reference hardware

- **MCU:** Raspberry Pi Pico 2 W (RP2350), MicroPython.
- **IMU bus:** I2C0, GP4 SDA / GP5 SCL, 400 kHz.
- **Depth bus:** I2C1, GP2 SDA / GP3 SCL, 1 MHz.
- **Motion sensors:** LSM6DSOX at `0x6A`; LIS3MDL at deterministic `0x1E` (`ADM -> 3V3`).
- **Depth sensor:** VL53L5CX at `0x29`, 8x8 / 64 zones at ~15 Hz.
- **Timing:** LSM6DSOX FIFO timestamps plus recorded Pico<->LSM `CLOCK_SYNC` observations; hosts do not assume nominal ODR is exact.
- **Acquisition gyro:** current stream firmware uses `CTRL2_G = 0x44` (104 Hz, +/-500 deg/s) to give calibration/head-motion adequate rate headroom.

The frozen hardware diagnostic remains the first reproduction gate and should reach `SYSTEM READY: PASS` before using the acquisition stream. The acquisition producer has also demonstrated lossless steady-state USB capture on the reference unit.

## What works now

**Physically validated on the current reference rig**

- split-bus ToF + inertial acquisition;
- FIFO protection/timestamp parsing and per-unit clock correlation;
- deterministic LIS3MDL addressing;
- loss-detectable Rangeweave v0.1 binary stream over USB CDC;
- canonical Python capture/replay path;
- physical VL53L5CX zone orientation and nominal 64-zone projection convention;
- live/temporal raw-depth viewing and sparse point-cloud inspection;
- current reference-rig `imu_sensor -> device_body` axis mapping;
- short-baseline relative-body gyro integration with independent gravity closure;
- gyro full-scale quality checking;
- guided fixed-plane ToF/body boresight capture through P0-P5;
- stationary ToF plane/temporal quality gates;
- held-out P5 validation of the P0-P4 boresight fit;
- stable six-pose final refit on the reference assembly;
- promoted per-device reference-rig `rangeweave.tof-body-rotation` artifact with capture/configuration provenance;
- deterministic persistent gyro+gravity orientation replay across five retained hold-move-hold motions;
- two independent continuous rotation-in-place wall validations with calibrated ToF/IMU timing alignment;
- versioned per-build `rangeweave.tof-time-alignment` resolution for the reference rig;
- empirical Phase 3 reference wall gate with quantified residual, drift, usable-frame and motion bounds;
- executable PASS evidence for both independent wall captures under that frozen gate.

The September 2026 reference boresight run predicted a held-out P5 ToF wall normal to **0.644 deg** and changed the fitted extrinsic by only **0.639 deg** after P5 was revealed. The six-pose result was approximately `Rx +5.880, Ry +3.930, Rz +0.160 deg` with `0.797 deg` normal RMS. These numbers describe that assembly only; other units must be calibrated independently for highest accuracy.

The two continuous Phase 3 wall captures retained `98.0%` and `99.8%` usable ToF observations. With the reference rig's calibrated `-24 ms` effective ToF timing offset they produced wall-normal RMS values of `0.776 deg` and `0.637 deg`, and both passed the executable reference-path gate. The gate is deliberately looser than either run and is not a universal sensor specification.

**Implemented / experimental**

- optional per-device VL53L5CX known-plane intrinsic calibration tooling;
- versioned `rangeweave.tof-body-rotation` extrinsic representation and fixed-plane solver;
- provenance-rich per-device boresight artifact generation;
- frozen Phase 3 `local_reference`/quaternion/body-rate attitude conventions with golden tests;
- deterministic timestamp-driven gyro+gravity orientation core;
- recorded-capture orientation replay inspector with gravity-confidence diagnostics and optional comparison against the validated hold-move-hold relative estimator;
- quick-start versus calibrated ToF timing resolver with configuration fingerprints and assembly identity;
- executable calibrated Phase 3 wall gate.

**Still open**

- orientation-aware ToF geometry/viewing;
- magnetometer/body calibration and confidence-gated magnetic heading;
- independent cross-unit reproduction of the boresight/timing/orientation workflow;
- rigid translation between sensor origins where required;
- robust freehand 6DoF tracking, odometry, loop closure and metrically validated 3D reconstruction;
- Android/BLE/Wi-Fi/ESP32 production parity.

## Current milestone: orientation-aware geometry

The first gravity-referenced Phase 3 reference-orientation gate is now satisfied on the tested reference rig. The validated orientation baseline:

1. uses explicit project-owned scalar-first Hamilton quaternion and `local_reference_from_body` conventions;
2. integrates mapped body-frame gyro using recorded sensor timestamps;
3. initializes from an explicit stationary interval and keeps the initial gyro-bias estimate fixed rather than silently treating later slow motion as bias;
4. uses accelerometer specific force as a confidence-gated gravity/pitch/roll reference while leaving yaw unobservable in six-axis mode;
5. replays canonical captures and compares hold-move-hold start/end rotation with the validated relative estimator;
6. applies versioned calibrated or quick-start ToF timing resolution instead of hard-coding a universal offset;
7. validates continuous rotation against a fixed wall using the calibrated ToF/body boresight.

The frozen empirical reference wall gate is:

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

These are evidence-based project validation bounds, not guaranteed cross-unit specifications. Quick-start remains a supported lower-friction path; calibrated builds use per-assembly artifacts.

The next steps are:

1. add orientation-aware ToF viewing/projection so calibrated rays/points can be expressed in `local_reference` during replay and live use;
2. verify static geometry remains stable in that orientation-aware view using the retained wall evidence;
3. add magnetic heading only after `mag_sensor -> device_body`, hard/soft-iron calibration and disturbance gating are physically validated;
4. only after orientation-aware geometry is stable, proceed to translation/6DoF pose, known-pose scanning and freehand odometry.

See [the Phase 3 orientation plan](docs/orientation-estimation.md), [the frozen attitude conventions](docs/attitude-conventions.md), [the physical orientation evidence](docs/validation/orientation-reference-rig-2026-09.md), and [the overall project roadmap](docs/project-plan.md).

## Quick start: reproduce the sensor stack

1. Read the [build guide](docs/build-guide.md).
2. Assemble the [Pico 2 W reference wiring](hardware/wiring/pico2w-reference.md).
3. Install the validated class of Pimoroni RP2350/Pico 2 W MicroPython build described in the guide.
4. Obtain `/vl53l5cx_firmware.bin` as described in the guide or with [`tools/fetch_vl53l5cx_firmware.py`](tools/fetch_vl53l5cx_firmware.py).
5. Bring up the system in order using the diagnostics in [`firmware/pico2w/diagnostics/`](firmware/pico2w/diagnostics/).
6. Require `SYSTEM READY: PASS` before moving to the binary acquisition producer in [`firmware/pico2w/acquisition/`](firmware/pico2w/acquisition/).
7. Use [`host/python/capture.py`](host/python/capture.py) for canonical captures and the documented host inspection tools for replay/geometry/calibration.

A healthy second IMU does not have to reproduce the exact measured rate of the first reference unit; timing is discovered and recorded per device.

## Calibration: quick-start and per-build refinement

Rangeweave deliberately supports two product-level modes:

- **Quick-start:** no mandatory physical calibration. Matching nominal profiles are used where justified by evidence; otherwise conservative fallbacks remain visibly `uncalibrated`.
- **Calibrated:** guided per-build poses/motions/scans generate versioned artifacts for quantities such as ToF/body boresight and effective ToF/IMU timing alignment.

The calibration jobs remain separate:

1. **ToF intrinsic/ray calibration** — optional refinement of the 64 rays inside `tof_optical`; see [the intrinsic known-plane workflow](docs/tof-calibration-plane-workflow.md).
2. **ToF/body boresight calibration** — one rigid rotation `R_body_from_tof` for the assembled head; see [the boresight guide](docs/boresight-calibration.md).
3. **ToF/IMU timing alignment** — one configuration- and assembly-scoped effective observation-time offset; see [the timing guide](docs/timing-calibration.md).

For boresight, the default physical target is a clear flat wall: P0 roughly 500 mm from the centre of at least a 1 m x 1 m unobstructed patch. A stable 3-way photographic head or equivalent fixture is a convenient way to hold and reposition a breadboard/sensor package. Exact wall distance and exact pose angles are not solver measurements.

The standard builder command is the full P0-P5 sequence:

```powershell
py host/python/guided_boresight.py COM5
```

After a clean run, generate the per-device artifact with:

```powershell
py host/python/generate_boresight_artifact.py <session-prefix>
```

That command fits P0-P4, checks P5 as held-out ToF, refits P0-P5, and writes a versioned `rangeweave.tof-body-rotation` JSON artifact with capture/configuration provenance.

## Architecture

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

The packet/record model is the portability boundary. Pico GPIOs, MicroPython objects and USB-specific behaviour must not define sensor-data meaning.

## Repository map

- [`docs/`](docs/) - build, architecture, calibration, coordinate-frame, orientation and validation documentation.
- [`firmware/pico2w/diagnostics/`](firmware/pico2w/diagnostics/) - builder-facing hardware self-test.
- [`firmware/pico2w/acquisition/`](firmware/pico2w/acquisition/) - binary acquisition producer.
- [`protocol/`](protocol/) - language-neutral wire specification and fixtures.
- [`host/python/`](host/python/) - capture/replay, geometry, viewers and calibration/estimation reference implementation.
- [`android/`](android/) - Android portability notes/future implementation.
- [`hardware/`](hardware/) - BOM, wiring and mechanical/PCB notes.
- [`datasets/`](datasets/) - dataset policy and future published reference recordings.

Start with the [documentation index](docs/README.md) and [project plan](docs/project-plan.md).

## Engineering rules

1. Preserve raw measurements and explicit timestamps.
2. Never infer time from sample index or nominal ODR.
3. Keep sensor semantics independent of transport and MCU implementation.
4. Make live and replayed data enter the same host APIs.
5. Define coordinate frames, units and transform direction explicitly.
6. Keep uncertainty/failure visible instead of forcing a confident pose.
7. Use conventional physics/geometry baselines before learned corrections.
8. Label claims as validated, experimental or planned.
9. Keep calibration intrinsics, rigid extrinsics and runtime estimator state separate.
10. Preserve quick-start nominal operation alongside optional per-build calibrated refinement.

## Not yet claimed

Rangeweave does not yet claim globally referenced heading, reliable freehand SLAM, dense reconstruction, loop closure, cross-unit calibration equivalence, universal timing/boresight values, or privacy guarantees merely because RGB is absent.

## Contributing and license

See [CONTRIBUTING.md](CONTRIBUTING.md), [LICENSE](LICENSE) and [THIRD_PARTY_NOTICES.md](THIRD_PARTY_NOTICES.md). Hardware reproduction reports are most useful when they include bus scans, sensor IDs, self-test output, runtime version, capture metadata and clear physical mounting/wiring information.
