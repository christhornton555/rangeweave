# ToF / device-body boresight calibration

**Status: physically validated builder workflow on the reference rig.**

This procedure estimates the rotational extrinsic `R_body_from_tof` for one rigidly assembled Rangeweave sensing head. It is separate from optional per-zone VL53L5CX intrinsic/ray calibration.

The key idea is simple: keep one flat plane fixed, move the sensing head through several stationary poses, measure the plane normal with ToF at each pose, and measure the relative `device_body` rotation between poses with the LSM6DSOX. The solver finds the one constant ToF-to-body rotation that makes every observation describe the same fixed plane.

The recommended builder sequence is now **P0 through P5**, with P5 used as a held-out ToF validation pose before the final six-pose refit is written to a per-device artifact.

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

## Suggested low-cost mounting rig

The sensing head must remain rigid internally: the ToF and IMU may not move relative to one another during the sequence. The support also needs to settle fully after each adjustment.

A photographic **3-way pan/tilt/roll head** or similar three-axis camera mount works well. The validated reference setup used a deliberately simple improvised fixture:

- a small lower MDF plate attached to one of the camera mount's built-in threaded connectors and held by a selfie-stick/support clamp at the base;
- a second MDF plate screwed to the camera mount's upper camera screw;
- the breadboard sensing head clamped firmly to that upper MDF plate;
- additional spring clamps used where useful to stop the carrier or breadboard shifting;
- USB and sensor wiring left with enough slack that the cables do not pull the head while it settles.

This exact construction is only an example. A tripod, geared head, machined bracket, printed fixture or other arrangement is equally suitable if it provides three useful rotational degrees of freedom and keeps the sensing head rigid. The rotation axes do not need to intersect the ToF optical centre: the fixed-plane solve uses plane normals rather than absolute sensor position, so the translation caused by rotating around a camera-head pivot is acceptable.

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

## Acquisition gyro range and motion gates

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

The boresight workflow rejects stationary poses that do not look like one stable planar target:

```text
maximum plane RMS residual:        10 mm
maximum absolute plane residual:   30 mm
maximum half-capture drift:        10 mm
```

These are empirical **boresight workflow gates**, chosen from physical reference-rig captures. They are deliberately much looser than normal clean-wall values, while rejecting the demonstrated moving/slipped and off-target failures. They are not universal VL53L5CX specifications.

## Standard P0-P5 sequence

The builder-facing command now captures six stationary poses and five relative motions:

```text
P0  stationary baseline
M1  pitch-rich motion                  -> P1
M2  move partly back + yaw right       -> P2
M3  yaw left across neutral            -> P3
M4  roll-rich mixed motion             -> P4
M5  another distinct yaw/pitch/roll    -> P5 held-out validation pose
```

The exact commanded angles do **not** need to be measured. The guided instructions use moderate motions as practical targets, not solver inputs. What matters is that the poses are meaningfully different, span multiple axes, keep the complete ToF field on the same wall, and have clean stationary endpoints.

Each motion produces `R_previous_from_current`. The sequence composes these into `R_reference_from_body`, then combines each stationary ToF plane normal with its body orientation for the fixed-plane solver.

### Timing and settling

Each motion capture uses:

```text
3 s   discarded acquisition warm-up
5 s   recorded initial stationary hold
10 s  movement / adjustment allowance
12 s  hands-off final settling hold
```

When the tool prints `STOP MOVING — HANDS OFF — HOLD THIS NEW POSE`, release the controls completely and do not touch the rig until the capture finishes. The long final hold was added after physical testing showed that camera-head oscillation could otherwise contaminate endpoint gyro-bias estimation.

Each stationary ToF pose then uses a separate 3 s discarded warm-up plus 8 s recorded hold.

## Run the guided calibration

From the repository root, replacing `COM5` as required:

```powershell
py host/python/guided_boresight.py COM5
```

The default is the full P0-P5 sequence. The command owns capture warm-up, HOLD / MOVE NOW / STOP MOVING cues, motion/ToF quality gates and intermediate sequence checks. It writes timestamped canonical capture directories under `captures/` and does not overwrite earlier attempts.

A shortened `--steps` run remains available for diagnostics, but the recommended artifact-promotion workflow uses all five motions and all six stationary poses.

## Held-out P5 validation and artifact generation

For promotion, P0-P4 are fitted first. M5 supplies the independently measured body orientation at P5, while P5's ToF plane normal is excluded from that fit. The P0-P4 calibration therefore predicts what wall normal the ToF should observe at P5. Only then is the actual P5 ToF plane revealed.

The artifact generator performs this held-out check, refits all six accepted poses, and writes a versioned `rangeweave.tof-body-rotation` JSON artifact with capture and configuration provenance:

```powershell
py host/python/generate_boresight_artifact.py <session-prefix>
```

For example:

```powershell
py host/python/generate_boresight_artifact.py boresight-guided-20260904_010944
```

The default output is:

```text
calibration/tof-body-rotation-<session-prefix>.json
```

The generator supports both the current one-session P0-P5 layout and the earlier reference run in which M5/P5 were captured immediately afterwards by `continue_boresight_validation.py`; in the latter case it auto-discovers the unique continuation capture pair.

The artifact records:

- schema/version and exact `rotation_body_from_tof` matrix;
- fixed-XYZ diagnostic angles and fit residuals;
- P5 held-out prediction error;
- change in the fitted rotation after revealing P5;
- IMU mapping role and gyro full-scale provenance;
- ToF geometry profile name/role;
- the active boresight quality gates;
- P0-P5 and M1-M5 capture-directory names and `packets.bin` SHA-256 hashes;
- captured `STREAM_INFO` metadata where available.

The current boresight plane fitter uses the built-in ST-derived `nominal-fallback` ToF geometry profile unless a future workflow explicitly supplies a calibrated intrinsic profile. The artifact records that fact rather than implying intrinsic calibration was performed.

## Reference-rig validation result

The completed September 2026 reference run passed every physical quality gate. P0-P4 produced:

```text
R_body_from_tof: Rx +5.430  Ry +3.780  Rz -0.300 deg
normal RMS:      0.847 deg
```

A geometrically different P5 was then held out from that fit. The P0-P4 calibration predicted P5's ToF wall normal with:

```text
held-out P5 angular error: 0.644 deg
```

After revealing P5, the six-pose refit became:

```text
R_body_from_tof: Rx +5.880  Ry +3.930  Rz +0.160 deg
normal RMS:      0.797 deg
normal max:      1.648 deg
fit rotation change after P5: 0.639 deg
```

This was a substantial improvement over the earlier P0-P3 -> P4 experiment, in which adding P4 moved the fit by about 5.6 deg because the first four poses did not yet constrain all axes strongly enough. The additional pose diversity is why P0-P5 is now the recommended builder sequence.

These values are calibration evidence for the **reference assembly**, not constants to copy to another build. Cross-unit reproduction remains a separate requirement.

See [`validation/boresight-reference-rig-2026-09.md`](validation/boresight-reference-rig-2026-09.md) for the evidence summary.

## Existing diagnostic tooling

Individual captures can still be inspected with:

```powershell
py host/python/inspect_relative_rotation.py <motion-capture>
py host/python/inspect_tof_calibration_capture.py <stationary-pose-capture>
```

A sequence can be replayed explicitly with `inspect_boresight_sequence.py`, and the earlier P0-P4 held-out diagnostic remains available as `inspect_boresight_holdout.py`.

## Limitations

The validated workflow establishes the rotational ToF/body extrinsic on the current reference rig. It does **not** yet establish:

- universal calibration parameters across independently assembled units;
- `mag_sensor -> device_body` mapping;
- translation between sensor origins;
- world-frame attitude conventions;
- odometry, SLAM or loop closure.

Do not treat the reference-rig result as a universal extrinsic for other builds.