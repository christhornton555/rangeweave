# Reference-rig ToF/body boresight validation — August/September 2026

**Scope:** physical evidence for the current breadboard reference rig and the Phase 2 fixed-plane boresight workflow. This is not a claim that every third-party assembly shares the same extrinsic or IMU-axis mapping.

## Reference frames validated on this rig

The physical LSM6DSOX mapping measured on the rigid prototype is:

```text
device_body +X -> imu_sensor -X
device_body +Y -> imu_sensor -Z
device_body +Z -> imu_sensor -Y
```

The host therefore uses:

```text
body X = -imu X
body Y = -imu Z
body Z = -imu Y
```

A simple positive body-X pitch capture recovered approximately +21 deg around X with sub-degree independent gravity closure.

## First successful fixed-plane sequence

The restarted fixed-plane sequence used four clean stationary ToF observations and three clean relative IMU motions. The provisional four-pose solve reported approximately:

```text
R_body_from_tof:
  Rx -1.440 deg
  Ry +1.910 deg
  Rz +1.210 deg
normal RMS: 0.535 deg
normal max: 0.666 deg
```

This was the first convincing physical demonstration that the fixed-plane solver, relative-body composition and ToF plane-normal path can produce a small plausible assembly boresight on the reference rig.

The result remains **provisional** because the intended additional held-out/roll-rich validation pose was not successfully completed under the then-current gyro configuration.

## Old +/-250 deg/s failure

The attempted roll-rich mixed motion under `CTRL2_G = 0x40` (+/-250 deg/s) produced:

```text
peak recorded body-X rate: about 267 deg/s
configured full scale:      +/-250 deg/s
gyro/gravity closure:       12.642 deg
```

The capture had clean stream/health counters and clean stationary endpoints, but exceeded the configured gyro range. It is therefore retained as diagnostic evidence and must not be admitted to a boresight solve.

This observation motivated two changes:

1. acquisition gyro range changed to `CTRL2_G = 0x44` (+/-500 deg/s, same nominal 104 Hz ODR);
2. host motion-quality checks now report configured full scale and reject captures at >=90% utilisation, with a warning at >=80%.

The rate excursion is a strong explanation for the failed closure but is not claimed as mathematically proven to be the sole possible cause.

## +/-500 deg/s validation

A stationary configuration check after flashing `CTRL2_G = 0x44` reported:

```text
configured range: +/-500 deg/s
peak utilisation: about 0.1%
estimated rotation: 0.045 deg
gyro/gravity closure: 0.043 deg
```

A later simple pitch validation reported:

```text
rotation angle: 21.000 deg
fixed XYZ:      Rx +20.802, Ry -2.838, Rz -1.276 deg
peak body rate: 70.7 deg/s on X
peak use:       14.1% of configured full scale
gravity change: 20.021 deg
gyro/gravity closure: 0.778 deg
```

A subsequent compound pitch/roll validation reported:

```text
rotation angle: 14.632 deg
fixed XYZ:      Rx +12.769, Ry +1.179, Rz +7.194 deg
peak body rate: X 70.2, Y 34.0, Z 35.2 deg/s
peak use:       <=14.0% of configured full scale
gravity change: 14.387 deg
gyro/gravity closure: 0.303 deg
```

These results support using +/-500 deg/s for calibration capture: it provides substantial headroom for normal deliberate hand/fixture movements while retaining good relative-rotation closure on the reference rig.

## ToF stationary-pose evidence

Clean stationary fixed-plane captures on the reference rig typically produced roughly 1-2 mm plane RMS residual and only a few millimetres of maximum half-capture drift.

Two distinct failure modes were physically observed:

- a pose could retain a good instantaneous plane fit while the target/sensor moved during the capture, producing large temporal half-capture drift;
- an off-target pose could produce very large plane RMS/max residuals when some ToF zones no longer observed the intended plane.

The boresight workflow therefore currently uses deliberately generous empirical gates:

```text
plane RMS <= 10 mm
plane max residual <= 30 mm
max half-capture drift <= 10 mm
```

These are workflow gates derived from the reference-rig evidence, not universal VL53L5CX sensor specifications.

## Capture-data provenance

Raw developer captures are intentionally not committed to the public repository by default. When preserving or publishing a calibration result, retain the original capture directories (`metadata.json`, `packets.bin`, `notes.txt`) and their recorded SHA-256 hashes so future code can replay the observations.

A final promoted per-device boresight artifact should record enough provenance to identify the capture set, firmware/configuration, geometry profile and fit diagnostics used to produce it.

## Remaining validation

Before the reference-rig boresight is promoted from provisional to a final per-device artifact:

- repeat the full fixed-wall sequence with the +/-500 deg/s acquisition configuration;
- reserve a geometrically different pose as a held-out validation observation;
- compare the four-pose fit against the held-out pose, then refit all poses and verify parameter stability;
- record the resulting artifact and provenance explicitly.

Cross-unit reproduction remains a separate requirement.