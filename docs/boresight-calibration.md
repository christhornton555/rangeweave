# ToF / device-body boresight calibration

**Status: physically validated developer workflow; builder-facing guided command still in development.**

This procedure estimates the rotational extrinsic `R_body_from_tof` for one rigidly assembled Rangeweave sensing head. It is separate from optional per-zone VL53L5CX intrinsic/ray calibration.

The key idea is simple: keep one flat plane fixed, move the sensing head through several stationary poses, measure the plane normal with ToF at each pose, and measure the relative `device_body` rotation between poses with the LSM6DSOX. The solver finds the one constant ToF-to-body rotation that makes every observation describe the same fixed plane.

## Recommended physical setup

For boresight calibration, use an ordinary flat wall rather than a small calibration board whenever possible.

Recommended P0 setup:

- choose a flat, unobstructed, preferably matt-painted wall;
- keep windows, mirrors, glossy tiles, corners, door frames, shelves and other depth discontinuities out of the working patch;
- aim the ToF approximately at the centre of at least a **1 m x 1 m clear wall area**;
- place the ToF approximately **500 mm from the wall**;
- aim approximately square-on for P0.

The 500 mm distance and square-on alignment are **not calibration measurements**. They are convenient starting geometry that leaves margin for later pitch/yaw/roll poses while keeping all 64 ToF zones on the same plane.

A large flat board is still valid when no suitable wall exists, but the wall is the default recommendation for boresight because it is easier to reproduce and much harder to move accidentally.

## Holding and moving the sensing head

The sensing head must remain rigid internally: the ToF and IMU may not move relative to one another during the sequence.

A photographic **3-way pan/tilt/roll head** or similar three-axis mount is recommended. A simple carrier plate with a standard camera thread, straps or clamps can hold a breadboard or prototype PCB. A geared head is convenient but not required.

The rotation axes do not need to intersect the ToF optical centre. The fixed-plane boresight solve uses plane normals rather than absolute sensor position, so the small translation caused by rotating around a camera-head pivot is acceptable.

## Frames and signs on the validated reference rig

`device_body` is right-handed:

```text
+X = device right
+Y = device down
+Z = mechanical forward / ToF-facing
```

Positive rotations use the right-hand rule. On the validated breadboard reference rig:

```text
body +X -> imu_sensor -X
body +Y -> imu_sensor -Z
body +Z -> imu_sensor -Y
```

Equivalently, host tools currently apply:

```text
body X = -imu X
body Y = -imu Z
body Z = -imu Y
```

This mapping is **physical-build specific**. A builder who mounts the IMU differently must establish the corresponding `imu_sensor -> device_body` rotation rather than copying these signs blindly.

## Acquisition gyro range

The current acquisition firmware configures the LSM6DSOX gyro for **104 Hz, +/-500 deg/s** (`CTRL2_G = 0x44`). The earlier +/-250 deg/s configuration was exceeded during a real mixed-axis calibration motion and produced a bad gyro/gravity closure result.

Host motion-quality checks read the actual `CTRL2_G` value from `STREAM_INFO`; they do not assume a fixed scale.

Current boresight motion gates:

```text
minimum useful relative rotation: 5 deg
gyro range warning:               80% of configured full scale
gyro range rejection:             90% of configured full scale
maximum gyro/gravity closure:      2 deg
```

The 80/90% values are workflow safety margins, not LSM6DSOX datasheet accuracy guarantees.

## Stationary ToF pose gates

The current boresight workflow also rejects stationary poses that do not look like one stable planar target:

```text
maximum plane RMS residual:        10 mm
maximum absolute plane residual:   30 mm
maximum half-capture drift:        10 mm
```

These are empirical **boresight workflow gates**, chosen from physical reference-rig captures. They are deliberately much looser than normal clean-wall values (typically around 1-2 mm plane RMS and only a few millimetres of half-capture drift), while rejecting the demonstrated moving/slipped and off-target failures.

They are not universal VL53L5CX specifications.

## Sequence structure

The current implementation is stateless and uses:

```text
P0 stationary ToF baseline
M1 relative IMU motion -> P1 stationary ToF pose
M2 relative IMU motion -> P2 stationary ToF pose
M3 relative IMU motion -> P3 stationary ToF pose
...
```

Each motion produces `R_previous_from_current`. The sequence composes these into `R_reference_from_body`, then combines each stationary ToF plane normal with its body orientation for the fixed-plane solver.

The solver needs at least four stationary plane observations. Useful motion should span multiple axes; repeating one orientation or only tiny rotations is underconstrained.

For final physical validation, prefer at least five poses and reserve one geometrically different pose as a held-out check before refitting all observations.

The exact commanded angles do not need to be measured. A practical sequence can use moderate movements around 10-20 degrees, for example a pitch-rich pose, yaw-rich poses in both directions, and one roll-rich mixed pose. The important requirements are clean stationary endpoints, useful multi-axis geometry, and passing the quality gates.

## Current command-line tooling

Existing captures can be inspected with:

```powershell
py host/python/inspect_relative_rotation.py <motion-capture>
py host/python/inspect_tof_calibration_capture.py <stationary-pose-capture>
```

A complete sequence can be replayed with:

```powershell
py host/python/inspect_boresight_sequence.py `
  --baseline <P0> `
  --step <M1> <P1> `
  --step <M2> <P2> `
  --step <M3> <P3>
```

The current generic `capture.py` has a default **3 second warm-up** that discards startup/backlog bytes before the recorded capture begins. Manual motion tests must account for that. The planned guided boresight command will own the warm-up and explicitly display HOLD / MOVE NOW / STOP MOVING cues so builders do not have to manage this timing themselves.

## What has been physically validated

On the current reference rig:

- the LSM6DSOX package-axis mapping above has been physically checked;
- a short-baseline relative-rotation estimator has recovered simple and compound physical motions with sub-degree gyro/gravity closure;
- a four-pose fixed-plane physical sequence produced a small, plausible provisional ToF/body boresight with low residual;
- an old +/-250 deg/s mixed-axis capture exceeded configured gyro full scale and failed closure;
- after switching acquisition to +/-500 deg/s, a ~21 deg pitch used only ~14% of full scale and closed to ~0.78 deg;
- a later compound motion used <=14% of full scale and closed to ~0.30 deg.

See [`validation/boresight-reference-rig-2026-09.md`](validation/boresight-reference-rig-2026-09.md) for the physical evidence summary.

## Not yet claimed

The following remain open before this becomes a polished end-user calibration path:

- a single guided capture command and manifest;
- held-out final physical validation of the complete boresight solve after the +/-500 deg/s change;
- automatic generation/selection of the per-device `rangeweave.tof-body-rotation` artifact;
- validation across independently assembled third-party units;
- `mag_sensor -> device_body`, world-frame conventions, and rigid translation between sensor origins.

Do not treat the provisional reference-rig solve as a universal extrinsic for other builds.