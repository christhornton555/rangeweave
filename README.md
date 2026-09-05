# Rangeweave - RGB-Free Active Depth + IMU Spatial Mapping

> **Project status:** reference sensor acquisition, protocol/capture/replay, raw/temporal depth viewing, nominal 64-zone projection and the reference-rig ToF/body rotational boresight workflow are physically validated on the current Pico 2 W stack. Phase 3 persistent orientation now has five clean hold-move-hold regressions plus two independent continuous wall-motion validations. A configuration-scoped ToF timing resolver supports first-class quick-start and calibrated modes; final Phase 3 physical acceptance bounds are the next step. Translation/odometry and 3D mapping remain work in progress.

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
- five persistent-orientation hold-move-hold replays with sub-degree endpoint disagreement versus the trusted relative estimator;
- two independent continuous multi-axis wall captures with sub-degree wall-normal RMS at zero timing compensation.

The September 2026 reference boresight run predicted a held-out P5 ToF wall normal to **0.644 deg** and changed the fitted extrinsic by only **0.639 deg** after P5 was revealed. The six-pose result was approximately `Rx +5.880, Ry +3.930, Rz +0.160 deg` with `0.797 deg` normal RMS. These numbers describe that assembly only; other units must be calibrated independently.

The two continuous wall captures independently preferred a broad effective ToF timing region around `-20` to `-28 ms`; `-24 ms` was the lowest-RMS 2 ms grid point in both runs. That value is retained only as a **per-build reference-rig calibration**, not as a universal VL53L5CX constant.

**Implemented / experimental**

- optional per-device VL53L5CX known-plane intrinsic calibration tooling;
- versioned `rangeweave.tof-body-rotation` extrinsic representation and fixed-plane solver;
- provenance-rich per-device boresight artifact generation;
- frozen Phase 3 `local_reference`/quaternion/body-rate attitude conventions with golden tests;
- deterministic timestamp-driven gyro+gravity orientation core;
- recorded-capture orientation replay inspector with gravity-confidence diagnostics and optional comparison against the validated hold-move-hold relative estimator;
- continuous wall-normal orientation validation tooling;
- versioned `rangeweave.tof-time-alignment` timing artifact/profile resolver.

**Still open**

- freezing conservative Phase 3 wall-normal acceptance bounds from the retained physical evidence;
- orientation-aware ToF viewing/projection;
- magnetometer/body calibration and confidence-gated magnetic heading;
- independent cross-unit reproduction of the boresight/timing workflows;
- rigid translation between sensor origins where required;
- robust freehand 6DoF tracking, odometry, loop closure and metrically validated 3D reconstruction;
- Android/BLE/Wi-Fi/ESP32 production parity.

## Quick-start and calibrated operation

Rangeweave deliberately supports two first-class operating modes.

### Quick-start

No mandatory physical calibration. Software uses a matched nominal/reference profile where there is enough evidence to trust one; otherwise it falls back conservatively and reports `nominal-fallback` or `uncalibrated` status rather than pretending the build is calibrated.

For ToF timing, the current known Pico 2 W / VL53L5CX 8x8@15 Hz quick-start profile still uses a conservative `0 ms` nominal because only one physical timing-calibrated assembly has been measured so far.

### Calibrated

A guided builder workflow uses short poses/motions/scans to generate per-build versioned artifacts for the quantities that matter on that assembly. Boresight and ToF timing are independent calibration modules; ToF intrinsics may remain nominal if desired.

Calibrated timing artifacts include a physical `assembly_id`, producer/protocol/ToF configuration fingerprint, source hashes and scan/plateau evidence. A material mismatch invalidates the artifact rather than silently reusing it.

See [the ToF timing calibration/profile contract](docs/timing-calibration.md) and [the calibration artifact policy](calibration/README.md).

## Current milestone: orientation

Phase 3 adds persistent attitude estimation before any attempt at freehand translation/SLAM.

The current baseline:

1. uses explicit project-owned scalar-first Hamilton quaternion and `local_reference_from_body` conventions;
2. integrates mapped body-frame gyro using recorded sensor timestamps;
3. initializes from an explicit stationary interval and keeps the initial gyro-bias estimate fixed rather than silently treating later slow motion as bias;
4. uses accelerometer specific force as a confidence-gated gravity/pitch/roll reference while leaving yaw unobservable in six-axis mode;
5. replays canonical captures and compares hold-move-hold start/end rotation with the validated relative estimator;
6. applies the promoted `R_body_from_tof` during continuous fixed-wall validation;
7. resolves effective ToF timing through explicit override, matched calibrated artifact, matched quick-start nominal profile, or visible zero-offset uncalibrated fallback.

The next evidence step is to replay the two retained wall captures through the calibrated timing artifact path, then freeze conservative physical acceptance gates from the worse compensated run. Orientation-aware ToF viewing follows once those bounds are frozen. Magnetic heading remains later work after `mag_sensor -> device_body`, hard/soft-iron calibration and disturbance gating are physically validated.

See [the Phase 3 orientation plan](docs/orientation-estimation.md), [the ToF timing calibration plan](docs/timing-calibration.md), [the frozen attitude conventions](docs/attitude-conventions.md), and [the overall project roadmap](docs/project-plan.md).

## Quick start: reproduce the sensor stack

1. Read the [build guide](docs/build-guide.md).
2. Assemble the [Pico 2 W reference wiring](hardware/wiring/pico2w-reference.md).
3. Install the validated class of Pimoroni RP2350/Pico 2 W MicroPython build described in the guide.
4. Obtain `/vl53l5cx_firmware.bin` as described in the guide or with [`tools/fetch_vl53l5cx_firmware.py`](tools/fetch_vl53l5cx_firmware.py).
5. Bring up the system in order using the diagnostics in [`firmware/pico2w/diagnostics/`](firmware/pico2w/diagnostics/).
6. Require `SYSTEM READY: PASS` before moving to the binary acquisition producer in [`firmware/pico2w/acquisition/`](firmware/pico2w/acquisition/).
7. Use [`host/python/capture.py`](host/python/capture.py) for canonical captures and the documented host inspection tools for replay/geometry/calibration.

A healthy second IMU does not have to reproduce the exact measured rate of the first reference unit; timing is discovered and recorded per device.

## Calibration jobs

Rangeweave keeps these distinct:

1. **ToF intrinsic/ray calibration** — optional refinement of the 64 rays inside `tof_optical`; see [the intrinsic known-plane workflow](docs/tof-calibration-plane-workflow.md).
2. **ToF/body boresight calibration** — one rigid rotation `R_body_from_tof` for the assembled head; see [the boresight guide](docs/boresight-calibration.md).
3. **ToF/IMU timing alignment** — effective observation-time alignment relative to protocol `mcu_ready_us`; see [the timing calibration guide](docs/timing-calibration.md).

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
10. Keep per-build calibrated values distinct from cross-device nominal profiles.

## Not yet claimed

Rangeweave does not yet claim globally referenced heading, reliable freehand SLAM, dense reconstruction, loop closure, cross-unit calibration equivalence or privacy guarantees merely because RGB is absent.

## Contributing and license

See [CONTRIBUTING.md](CONTRIBUTING.md), [LICENSE](LICENSE) and [THIRD_PARTY_NOTICES.md](THIRD_PARTY_NOTICES.md). Hardware reproduction reports are most useful when they include bus scans, sensor IDs, self-test output, runtime version, capture metadata and clear physical mounting/wiring information.
