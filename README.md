# Rangeweave - RGB-Free Active Depth + IMU Spatial Mapping

> **Project status:** reference sensor acquisition, protocol/capture/replay, raw/temporal depth viewing, nominal 64-zone projection and the main physical pieces of ToF/body boresight calibration are implemented on the current Pico 2 W reference rig. Final held-out boresight validation, world-orientation/odometry and 3D mapping remain work in progress.

Rangeweave explores a small, inexpensive **RGB-free** sensing module combining sparse time-of-flight depth with inertial sensing. The long-term goal is to turn synchronized depth + motion observations into trajectories, sparse point clouds and eventually 3D maps while keeping the sensing head compact and portable beyond the current Raspberry Pi Pico prototype.

The project is deliberately evidence-first: preserve raw measurements and source timing, validate coordinate frames and calibration physically, and only then build higher-level estimation.

## Current reference hardware

- **MCU:** Raspberry Pi Pico 2 W (RP2350), MicroPython.
- **IMU bus:** I2C0, GP4 SDA / GP5 SCL, 400 kHz.
- **Depth bus:** I2C1, GP2 SDA / GP3 SCL, 1 MHz.
- **Motion sensors:** LSM6DSOX at `0x6A`; LIS3MDL at deterministic `0x1E` (`ADM -> 3V3`).
- **Depth sensor:** VL53L5CX at `0x29`, 8x8 / 64 zones at ~15 Hz.
- **Timing:** LSM6DSOX FIFO timestamps plus recorded Pico<->LSM `CLOCK_SYNC` observations; hosts do not assume nominal ODR is exact.
- **Acquisition gyro:** current stream firmware uses `CTRL2_G = 0x44` (104 Hz, +/-500 deg/s) to give calibration motions adequate rate headroom.

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
- fixed-plane ToF/body boresight sequencing and a provisional physical four-pose solve;
- stationary ToF boresight quality gates for plane fit and temporal drift.

**Implemented / experimental**

- optional per-device VL53L5CX known-plane intrinsic calibration tooling;
- versioned `rangeweave.tof-body-rotation` extrinsic representation and fixed-plane solver;
- developer command-line boresight replay/inspection.

**Still open**

- one polished guided boresight capture command/manifest;
- final held-out physical boresight validation under the current +/-500 deg/s acquisition configuration;
- promoted per-device boresight artifact with provenance;
- magnetometer/body calibration and world-frame attitude conventions;
- robust freehand 6DoF tracking, odometry, loop closure and metrically validated 3D reconstruction;
- Android/BLE/Wi-Fi/ESP32 production parity.

## Quick start: reproduce the sensor stack

1. Read the [build guide](docs/build-guide.md).
2. Assemble the [Pico 2 W reference wiring](hardware/wiring/pico2w-reference.md).
3. Install the validated class of Pimoroni RP2350/Pico 2 W MicroPython build described in the guide.
4. Obtain `/vl53l5cx_firmware.bin` as described in the guide or with [`tools/fetch_vl53l5cx_firmware.py`](tools/fetch_vl53l5cx_firmware.py).
5. Bring up the system in order using the diagnostics in [`firmware/pico2w/diagnostics/`](firmware/pico2w/diagnostics/).
6. Require `SYSTEM READY: PASS` before moving to the binary acquisition producer in [`firmware/pico2w/acquisition/`](firmware/pico2w/acquisition/).
7. Use [`host/python/capture.py`](host/python/capture.py) for canonical captures and the documented host inspection tools for replay/geometry/calibration.

A healthy second IMU does not have to reproduce the exact measured rate of the first reference unit; timing is discovered and recorded per device.

## Calibration: two different jobs

Rangeweave deliberately separates:

1. **ToF intrinsic/ray calibration** — optional refinement of the 64 rays inside `tof_optical`; see [the intrinsic known-plane workflow](docs/tof-calibration-plane-workflow.md).
2. **ToF/body boresight calibration** — one rigid rotation `R_body_from_tof` for the assembled head; see [the boresight guide](docs/boresight-calibration.md).

For boresight, the current default physical target is a clear flat wall: P0 roughly 500 mm from the centre of at least a 1 m x 1 m unobstructed patch. A 3-way photographic head is a convenient way to hold and reposition a breadboard/sensor package. Exact wall distance and exact pose angles are not solver measurements.

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

- [`docs/`](docs/) - build, architecture, calibration, coordinate-frame and validation documentation.
- [`firmware/pico2w/diagnostics/`](firmware/pico2w/diagnostics/) - builder-facing hardware self-test.
- [`firmware/pico2w/acquisition/`](firmware/pico2w/acquisition/) - binary acquisition producer.
- [`protocol/`](protocol/) - language-neutral wire specification and fixtures.
- [`host/python/`](host/python/) - capture/replay, geometry, viewers and calibration reference implementation.
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

## Not yet claimed

Rangeweave does not yet claim reliable freehand SLAM, dense reconstruction, loop closure, cross-unit calibration equivalence or privacy guarantees merely because RGB is absent.

## Contributing and license

See [CONTRIBUTING.md](CONTRIBUTING.md), [LICENSE](LICENSE) and [THIRD_PARTY_NOTICES.md](THIRD_PARTY_NOTICES.md). Hardware reproduction reports are most useful when they include bus scans, sensor IDs, self-test output, runtime version, capture metadata and clear physical mounting/wiring information.
