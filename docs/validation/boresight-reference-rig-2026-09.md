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

## Early four-pose evidence

The first convincing fixed-plane sequence used four clean stationary ToF observations and three clean relative IMU motions. Its provisional solve reported approximately:

```text
R_body_from_tof:
  Rx -1.440 deg
  Ry +1.910 deg
  Rz +1.210 deg
normal RMS: 0.535 deg
normal max: 0.666 deg
```

This established that the fixed-plane solver, relative-body composition and ToF plane-normal path could produce a plausible assembly boresight. It was deliberately not promoted because the intended additional held-out/roll-rich pose had not been validated.

## Old +/-250 deg/s failure and range change

The attempted roll-rich mixed motion under `CTRL2_G = 0x40` (+/-250 deg/s) produced:

```text
peak recorded body-X rate: about 267 deg/s
configured full scale:      +/-250 deg/s
gyro/gravity closure:       12.642 deg
```

The capture had clean stream/health counters and clean stationary endpoints, but exceeded the configured gyro range. It was retained as diagnostic evidence and excluded from calibration solves.

This motivated two changes:

1. acquisition gyro range changed to `CTRL2_G = 0x44` (+/-500 deg/s, same nominal 104 Hz ODR);
2. host motion-quality checks report configured full scale and reject captures at >=90% utilisation, with a warning at >=80%.

The rate excursion is a strong explanation for the failed closure but is not claimed as mathematically proven to be the sole possible cause.

## +/-500 deg/s validation before the final sequence

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

These results support using +/-500 deg/s for calibration capture: it provides substantial headroom for normal deliberate fixture movements while retaining good relative-rotation closure on the reference rig.

## Mechanical settling finding

An otherwise clean guided M4 attempt initially failed the 2 deg gravity-closure gate at 2.744 deg. Its final nominally stationary window showed substantially more gyro and accelerometer variation than its initial window, consistent with camera-head oscillation/settling contaminating the endpoint bias estimate.

The guided motion schedule was therefore changed from a 6 s to a **12 s hands-off final hold**, while retaining the 10 s movement allowance. The final successful sequence used:

```text
3 s discarded warm-up
5 s recorded initial hold
10 s movement / adjustment
12 s hands-off final settling hold
```

The replacement M4 then closed to 0.745 deg with stationary gyro robust spreads of 0.026 / 0.021 deg/s.

## Final P0-P5 fixed-wall sequence

The successful guided session prefix was:

```text
boresight-guided-20260904_010944
```

It used a rigid breadboard sensing head on a three-axis camera mount, observing one unchanged flat wall. P0-P4 were captured in the main guided session; the rig was left untouched at P4 and M5/P5 were captured immediately afterwards as validation continuation:

```text
boresight-guided-20260904_010944-validation-20260904_014426
```

The recommended builder workflow has since been simplified to capture P0-P5 in one guided session.

### Motion quality

```text
M1: angle 17.822 deg; gravity closure 0.647 deg; gyro  8.6% used
M2: angle 28.080 deg; gravity closure 0.390 deg; gyro 10.4% used
M3: angle 45.426 deg; gravity closure 0.322 deg; gyro 11.5% used
M4: angle 23.322 deg; gravity closure 0.745 deg; gyro  9.0% used
M5: angle 24.295 deg; gravity closure 0.510 deg; gyro 20.8% used
```

All motions passed the 5 deg minimum-motion, 2 deg maximum gravity-closure and gyro-range gates with substantial +/-500 deg/s headroom.

### Stationary ToF quality

```text
P0: RMS 1.315 mm; max 4.308 mm; half-drift 1.00 mm
P1: RMS 1.836 mm; max 4.801 mm; half-drift 2.00 mm
P2: RMS 2.818 mm; max 9.624 mm; half-drift 2.00 mm
P3: RMS 2.062 mm; max 5.630 mm; half-drift 1.00 mm
P4: RMS 1.836 mm; max 4.942 mm; half-drift 2.00 mm
P5: RMS 2.366 mm; max 5.920 mm; half-drift 1.50 mm
```

All six poses used all 64 zones and passed the boresight plane/temporal-stability gates comfortably.

## Why six poses are now recommended

A P0-P3 training fit, with P4 held out, produced:

```text
R_body_from_tof: Rx +7.790  Ry +8.860  Rz -0.410 deg
normal RMS:      0.911 deg
P4 held-out error: 1.769 deg
```

After revealing P4, the five-pose fit changed by 5.607 deg in 3-D rotation, with `Ry` moving by about 5.08 deg. The physical P4 data itself was clean; the observability diagnostics showed that P4 contributed important new geometric constraint, especially to the previously weak direction.

The P0-P4 fit was therefore treated as the candidate calibration and a sixth geometrically different pose was captured for independent validation.

## Held-out P5 validation

With P5 ToF excluded, the P0-P4 fit was:

```text
R_body_from_tof:  Rx +5.430  Ry +3.780  Rz -0.300 deg
normal RMS:       0.847 deg
normal max:       1.604 deg
observability:    X 6.040e-06  Y 4.721e-06  Z 2.330e-05
```

M5's IMU-derived body orientation was then used to predict the ToF-frame wall normal at P5. The held-out result was:

```text
predicted n_tof: X +0.36748  Y +0.22585  Z +0.90219
observed n_tof:  X +0.36990  Y +0.21500  Z +0.90385
angular error:   0.644 deg
```

The prediction error was lower than the P0-P4 training RMS, providing strong evidence that the calibration generalized to a new physical pose.

After revealing P5, the six-pose refit became:

```text
R_body_from_tof:  Rx +5.880  Ry +3.930  Rz +0.160 deg
normal RMS:       0.797 deg
normal max:       1.648 deg
observability:    X 5.169e-06  Y 4.278e-06  Z 2.742e-05
```

The parameter shift after adding P5 was small:

```text
dRx +0.450 deg
dRy +0.150 deg
dRz +0.460 deg
3-D rotation change: 0.639 deg
```

This is the validation result that supports promotion of the six-pose reference-rig boresight artifact.

## Capture-data provenance

Raw developer captures are intentionally not committed to the public repository by default. The successful captures and their recorded `packets.bin` SHA-256 hashes are:

```text
P0  8ec010679e1b30461238785dabd39c89b254d72b3094ff573ab395c6d496a753
M1  ff8fec5813640967a8844b0be973644eb5510e486f81e60cfbee7700739f5e25
P1  fed9d8e9852a8d3b51cd0505f2d5064483ef5e31c1419fdfe0951cef4b233d29
M2  87c135ffaf44f5059cb7ddbe17f7464bfad5cbf5f3e3c80b029f2c5537288bad
P2  ef15070e6bc3534cae8449d0484e264f953ee80ae476619c00438aa2fe36250c
M3  9ee2a3061064d6b1ad537648726afa15516e1f37ed91d4af8004b308f2675c28
P3  308f9ebe7324fa643d6e3c2edd30066f6e3b9dd11c4d6047fc723d29b6cce94b
M4  659c9f43bb83963835041a814bf8bd3b446fa4c4e186b61ebbe2e061094ba37c
P4  98d29cc819b1e39bd5d9e4c6f4396384363b2b384f76714f4f924bae9a4da795
M5  d684c1d5c605cf0e2b596d3f5424a2395ddc37d65759fa71b5f4d4d7f33fdae4
P5  c521ef7405936b8948f439cd6896f8607b37387ae77ae1a565d63e55017f39e9
```

The promoted artifact generator embeds capture-directory names, these packet hashes, `STREAM_INFO`, IMU mapping role, gyro-range provenance, quality gates and the ToF geometry profile role.

The final calibration used the built-in ST-derived `nominal-fallback` ToF geometry profile. That provenance is recorded explicitly; intrinsic per-zone calibration was not silently assumed.

## Current claim boundary

The **reference rig's rotational ToF/body boresight workflow is now physically validated**, including a genuinely held-out ToF pose and a stable six-pose refit.

The calibrated rotation remains per-assembly. Cross-unit reproduction is still required before making any claim about typical manufacturing spread or universal values. Magnetometer/body mapping, sensor-origin translation, world attitude, odometry and SLAM remain separate later work.