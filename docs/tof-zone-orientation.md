# VL53L5CX zone orientation

**Status: physically validated on 2026-08-22.**

This note records the experiment used to map VL53L5CX producer-native 8x8 zone indices to the physically upright sensor image used by Rangeweave.

## Why this was measured

Protocol v0.1 intentionally defines `layout_id = 0` only as producer-native flattened zone order. Earlier capture, analysis and terminal output therefore preserved the sensor producer's row/column ordering without claiming that producer row 0 was physically top or producer column 0 was physically left.

During validation of the live/temporal viewer, the graphical image was observed to be upside down. PR #6 introduced a 180-degree **presentation-only** correction, but deliberately did not promote that observation to a sensor-geometry convention until a dedicated physical test had been completed.

## Experimental setup

The sensor assembly was held stationary. A narrow close target was placed in each physical corner of the VL53L5CX field of view in turn, with a more distant background behind it. The live and recorded depth viewer was used to identify the producer-native cell containing the closest target.

"Physical left/right/up/down" here means the image seen when standing behind the sensor and looking forward into the scene along the sensor viewing direction.

Four short captures were made using the labels:

```text
tof-corner-upper-left
tof-corner-upper-right
tof-corner-lower-left
tof-corner-lower-right
```

The exact capture directories remain local test data under the Git-ignored `captures/` tree.

## Measured result

```text
physical upper-left   -> producer r07c07
physical upper-right  -> producer r07c00
physical lower-left   -> producer r00c07
physical lower-right  -> producer r00c00
```

All four corners therefore agree with one transform: a **180-degree rotation** between producer-native row/column ordering and the physically upright image.

For producer-native `(r, c)` on the 8x8 grid:

```text
physical_row = 7 - r
physical_col = 7 - c
```

No transpose is required. No independent horizontal or vertical mirror is required.

## Consequences

1. Raw protocol and capture semantics remain unchanged. `packets.bin`, decoded `TOF_GRID` tuples, terminal matrices, statistics and analysis retain producer-native ordering.
2. Graphical views may rotate the producer-native matrix by 180 degrees for physically upright presentation while retaining producer index labels.
3. Geometry code may convert producer indices into physical/image indices using the explicit mapping above.
4. `tof_optical` is defined as a right-handed frame with `+X` image-right, `+Y` image-down and `+Z` forward into the scene. See [`coordinate-frames.md`](coordinate-frames.md).
5. This experiment determines zone **orientation/order only**. It does not determine per-zone ray angles, lens distortion, optical-centre offset, range bias, or sensor/IMU extrinsics.

## What this does not prove

The four-corner test is sufficient to distinguish the relevant discrete 8x8 orientation transforms for the current producer layout, but it is not an optical calibration. The next geometry increment must still obtain or define nominal/calibrated direction vectors for all 64 zones before converting a measured range into an `(x, y, z)` point.