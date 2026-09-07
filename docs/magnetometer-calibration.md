# Magnetometer mapping, calibration and heading workflow

**Status:** Phase 4 groundwork. No LIS3MDL reading is yet accepted as a body-frame or heading observation.

Rangeweave already records raw LIS3MDL samples and the sensor's CTRL_REG1..5 configuration snapshot. The next task is to turn that evidence into a physically validated, disturbance-aware heading source without weakening the existing gyro+gravity orientation path.

## Design contract

Keep these quantities separate:

1. **raw sensor measurement** in `mag_sensor`;
2. **rigid assembly rotation** `R_body_from_mag` (`mag_sensor -> device_body`);
3. **hard-iron bias** fixed in the sensor/assembly frame;
4. **soft-iron correction** describing anisotropic scale/cross-axis distortion;
5. **runtime magnetic confidence/disturbance state**;
6. **optional heading/yaw correction** applied only when confidence is adequate.

A magnetic calibration artifact must never silently redefine `device_body`, the six-axis attitude convention, or the local yaw-zero convention.

## Product behaviour

Magnetic heading is optional. The existing gyro+gravity estimator remains a supported fallback when:

- no magnetometer calibration exists;
- the assembly fingerprint does not match;
- the local field is disturbed;
- motion/field geometry is insufficient for a credible heading observation;
- the user explicitly disables magnetic correction.

A calibrated magnetometer is not automatically trusted indoors.

## M0 — raw acquisition/configuration health

Before any axis mapping or calibration fit:

- decode LIS3MDL CTRL_REG1..5 from `STREAM_INFO`;
- derive full scale / sensitivity from the recorded configuration;
- convert raw signed counts to engineering units in **native `mag_sensor` only**;
- report sample cadence/read bracketing, retry/overrun flags and stream health;
- report per-axis range/span and field-magnitude distribution;
- do **not** apply an Earth-field acceptance threshold yet.

Reference host tools:

- `host/python/rangeweave_magnetometer.py`
- `host/python/inspect_magnetometer.py`

### Reference M0 evidence

`capture_20260905_005203Z_phase3-rotation-in-place-wall-2` produced 301 MAG samples over 30.000 s with clean protocol/producer health, no retries, and a recorded LIS3MDL configuration of `74 00 00 0C 40` (+/-4 gauss, 20 Hz sensor ODR, ultra-high-performance XY/Z, continuous conversion, BDU enabled). The producer recorded at 10.000 Hz, so 297/301 records carried the LIS3MDL overrun flag: unread intermediate 20 Hz conversions were being overwritten before the 10 Hz producer read. The retained XYZ triplets remain coherent because BDU is enabled, but this cadence mismatch must be corrected before new calibration evidence or heading fusion is accepted.

The uncalibrated native field magnitude in that capture ranged from 39.98 to 60.94 uT (median 49.66 uT). This is diagnostic only and is not yet treated as an Earth-field gate.

## M1 — physically establish `mag_sensor -> device_body`

The mapping is assembly-specific. Do not promote a transform from package drawings or breakout-board assumptions alone.

Recommended evidence is a rigid multi-axis rotation capture in a magnetically quiet location while the already validated gyro+gravity estimator tracks `device_body`. Candidate proper signed-permutation mappings can then be compared by how nearly they make the measured magnetic-field direction consistent in `local_reference` over the motion.

The mapping experiment should:

- begin with an explicit stationary interval so the six-axis orientation estimator initializes normally;
- include substantial rotation about at least two non-parallel body axes, preferably all three;
- avoid large ferromagnetic fixtures, loudspeakers, motors, power bricks and other obvious magnetic sources where practical;
- retain the complete raw capture and configuration fingerprint;
- report candidate separation rather than only the winning mapping;
- remain diagnostic until the winner is independently reproduced.

Because hard/soft-iron errors can bias candidate scoring, mapping evidence and magnetic calibration may need to be iterated rather than treated as perfectly independent one-shot fits.

### Retained wall-capture mapping experiments

Two pre-existing Phase 3 wall captures were scanned over all 24 proper signed-permutation mappings. They are useful as diagnostics but **do not reproduce the same winning mapping**, so they are explicitly not accepted as M1 calibration evidence.

Wall capture 2 (`capture_20260905_005203Z_phase3-rotation-in-place-wall-2`, 28.412 deg orientation excursion) selected:

```text
body X = +mag Z
body Y = -mag X
body Z = -mag Y
```

with a broad timing optimum near -84 ms, direction RMS 5.636 deg, p95 11.105 deg, and second/best RMS ratio 1.195x.

Wall capture 1 (`capture_20260905_000517Z_phase3-rotation-in-place-wall`, 38.503 deg excursion) instead selected:

```text
body X = -mag Y
body Y = +mag Z
body Z = -mag X
```

with its best result at the +0 ms edge of the searched -160..0 ms range, direction RMS 6.095 deg, p95 11.830 deg, and second/best ratio 1.299x. The capture-2 candidate ranked second at 7.920 deg RMS.

This disagreement is decisive evidence that the retained wall motions, uncalibrated field distortion and 20 Hz/10 Hz overrun timing are insufficient to identify `R_body_from_mag` reliably. Do not extend or tune these old captures further in search of a preferred mapping. The next M1 evidence must come from a purpose-made multi-axis capture after the magnetic acquisition cadence mismatch has been corrected.

## M2 — hard-iron and soft-iron calibration

After a credible body mapping exists, collect a broad 3D orientation sweep of the **actual completed sensing assembly** in a magnetically quiet area.

A suitable calibration fit should produce a versioned assembly-scoped artifact containing at least:

- schema/version and artifact role;
- `assembly_id` / configuration fingerprint;
- promoted `R_body_from_mag` or a reference to the promoted mapping artifact;
- hard-iron bias vector;
- soft-iron correction matrix;
- input capture hash(es);
- sample count and orientation/field coverage diagnostics;
- pre/post calibration norm residuals;
- fit/held-out or repeat-capture evidence where practical;
- creation tool/version and provenance notes.

The fit must reject degenerate sweeps that cover only a plane/arc of orientations.

## M3 — disturbance characterization and confidence

Characterize the calibrated assembly in representative conditions, including deliberately disturbed cases.

Potential confidence evidence includes:

- calibrated field magnitude versus the locally learned/reference magnitude;
- rapid magnitude/direction changes inconsistent with gyro-predicted body motion;
- calibration residual / model consistency;
- saturation/full-scale utilization;
- sensor status/retry/overrun health;
- disagreement between magnetic yaw innovation and gyro-predicted yaw change;
- persistence/hysteresis so heading correction does not chatter on/off.

Thresholds must be derived from physical evidence rather than copied from generic compass examples.

## M4 — confidence-gated heading correction

Only after M1-M3 are physically validated should magnetic heading enter the persistent attitude estimator.

Expected behaviour:

- gyro remains the high-rate propagation source;
- accelerometer continues to constrain gravity/pitch/roll when credible;
- calibrated magnetometer supplies a low-bandwidth yaw/heading observation only when magnetic confidence is adequate;
- poor confidence smoothly reduces or disables magnetic correction;
- six-axis output remains available and explicitly labelled when heading is not trusted;
- no global geographic heading/declination convention is claimed until separately frozen.

## Current immediate work

Correct the reference producer's magnetic acquisition cadence so the 20 Hz LIS3MDL output is not routinely overwritten by a 10 Hz read schedule. After that producer change has its own smoke/health evidence, collect a new purpose-made M1 capture with a stationary initialization period followed by broad, slow rotation about multiple body axes in a magnetically quiet setup. The old wall captures remain retained diagnostics only.
