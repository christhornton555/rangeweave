# Single ToF + IMU Prototype Module

**Build, validation and reproducibility guide - public Markdown edition**

Derived from revision 0.3 (17 August 2026). From this point onward, Git history supersedes versioned documentation filenames.

> **Status: validated breadboard reference build**  
> This public guide supersedes the older v0.1/v0.2 development documents. The reference path is MicroPython on Raspberry Pi Pico 2 W, with the IMU and ToF on separate hardware I2C buses, LSM6DSOX hardware FIFO timestamps, deterministic magnetometer addressing and runtime per-unit clock calibration. The v0.5 self-test firmware has passed on the reference hardware.

> **Project intent**  
> Collect honest, well-timestamped depth + motion data first; build tracking, mapping and learned models on top of that evidence rather than hiding acquisition problems inside the estimator.

# Revision 0.3: what changed

Revision 0.3 is a substantial correction and consolidation of the prototype guide after hardware bring-up and stress testing. It is written so that a new builder should be able to reproduce the reference sensor stack without needing the development conversation that produced it.

| Area | v0.2 | v0.3 validated position |
| --- | --- | --- |
| Development environment | Arduino/C++ examples | MicroPython only; Pimoroni RP2350 build with VL53L5CX support |
| Bus topology | One shared I2C bus | Two physical hardware I2C buses: IMU on I2C0, ToF on I2C1 |
| IMU timing | Host polling | LSM6DSOX hardware FIFO + one hardware timestamp per FIFO slot |
| IMU rate | Assumed nominal rate | Nominal configuration only; actual rate is discovered per device from INTERNAL_FREQ_FINE and measured clock correlation |
| Magnetometer | Default 0x1C accepted | Reference build forces ADM high so LIS3MDL is deterministically 0x1E; BDU + status check + retry |
| ToF | 8x8 with generic bring-up | 8x8 at 15 Hz on 1 MHz I2C1; full frame reads are allowed to block because IMU samples continue into FIFO |
| Transport | Large ASCII logger proposed | Diagnostic text only for bring-up; production data path will use buffered structured/binary packets |

# 1. Project scope and design principles

The reference module combines a 64-zone time-of-flight depth sensor with a 6-axis inertial sensor and a 3-axis magnetometer. The immediate goal is not full SLAM on the microcontroller. The goal is a small and inexpensive sensing head that can emit synchronized, reproducible observations for a host computer to turn into trajectories, point clouds and eventually 3D maps.

- Privacy-first sensing: the reference design does not require an RGB camera. It measures sparse depth, acceleration, angular velocity and magnetic field.

- Raw-first engineering: preserve raw observations and timing metadata so that later filtering, calibration, ML and SLAM can be improved without recollecting every experiment.

- Physics before ML: use hardware timestamps, geometry and calibration as the foundation; learned components can later estimate confidence, drift corrections, dynamics or spatial priors.

- Unit-agnostic timing: never assume that a nominal 104 Hz IMU is exactly 104 Hz. Every physical sensor must calibrate onto a shared time base at runtime.

- Fail loudly during bring-up: deterministic addresses and a self-test are preferred to silently accepting a subtly different wiring configuration.

- Modularity: the reference build uses breakout boards and a Pico 2 W so individual subsystems can be replaced and characterized independently before a custom PCB is attempted.

> **What this revision proves**
> The current firmware proves reliable coexistence of the three sensing subsystems and establishes a common timing model. It does not yet claim six-degree-of-freedom tracking, loop closure, object reconstruction or SLAM accuracy.

# 2. Validated reference hardware

| Item | Reference part | Role / notes |
| --- | --- | --- |
| Microcontroller | Raspberry Pi Pico 2 W (RP2350) | Runs MicroPython, owns both I2C buses and USB connection. |
| Depth sensor | Pimoroni VL53L5CX 8x8 ToF breakout | 64 depth zones. Reference mode: 8x8 at 15 Hz. |
| 9-DoF board | Adafruit LSM6DSOX + LIS3MDL Precision 9 DoF breakout | LSM6DSOX accel/gyro + LIS3MDL magnetometer. |
| Prototype wiring | Breadboard + short jumper wires | Suitable for bring-up. Move to rigid wiring for motion datasets. |
| Host | PC with Thonny / Python tooling | Flashing, serial console, later logging and visualization. |

## 2.1 Known-good sensor identities

| Device | Reference I2C address | WHO_AM_I / identity |
| --- | --- | --- |
| LSM6DSOX | 0x6A | 0x6C |
| LIS3MDL | 0x1E | 0x3D |
| VL53L5CX | 0x29 | Detected and initialized by Pimoroni driver |

The reference firmware deliberately requires the LIS3MDL to appear at 0x1E. This is not because 0x1C is electrically invalid; it is because the project now treats the address-select wiring as part of the reproducible hardware specification.

# 3. Mechanical layout

## 3.1 Make the sensing head a rigid body

- Rigid is more important than close. The ToF-to-IMU transform must remain constant while the module moves.

- Keep the VL53L5CX aperture unobstructed. Do not add a cover window during the first reference build; cover-window crosstalk is a separate calibration problem.

- Mark +X, +Y and +Z axes physically on the mounting plate. Do not rely on memory when writing coordinate transforms later.

- Measure and record the approximate XYZ displacement between the ToF optical centre and the IMU package centre. Millimetre accuracy is adequate for this prototype stage.

- Keep the LIS3MDL away from magnets, speakers, motors, steel fasteners and large current-carrying wires where practical.

A small piece of 3 mm plywood, acrylic, perfboard or a 3D-printed plate is sufficient for a first rigid assembly. The Pico can sit behind or below the sensor plate because its position is not a measurement coordinate, but its magnetic/electrical influence should still be included when the final assembly is calibrated.

# 4. Deterministic electrical wiring

> **Important: two physical I2C buses**
> Do not place the VL53L5CX and the IMU stack on one shared bus for this reference build. Combined testing showed that the long full-frame ToF transaction made shared-bus operation unreliable. The split-bus architecture is the validated baseline.

## 4.1 Power

| From | To | Notes |
| --- | --- | --- |
| Pico 3V3(OUT), physical pin 36 | Breadboard 3.3 V rail | Reference sensor supply. |
| Pico GND | Breadboard GND rail | Common ground for both sensor boards. |
| 3.3 V rail | Adafruit VIN | Adafruit breakout accepts this supply. |
| 3.3 V rail | Pimoroni VL53L5CX VCC | Keep logic at 3.3 V. |
| GND rail | Both breakout GND pins | Common return. |

> **Servo warning**
> Do not power hobby servos from the Pico 3V3 rail. A later pan/tilt rig must use an appropriate external supply (typically around 5 V for common micro-servos) with a common ground to the Pico.

## 4.2 IMU bus: hardware I2C0

| Pico 2 W | Adafruit 9-DoF | Reference setting |
| --- | --- | --- |
| GP4 | SDA | I2C0 SDA |
| GP5 | SCL | I2C0 SCL |
| 3V3 | ADM | Mandatory in reference build: forces LIS3MDL to 0x1E |
| leave open | ADAG | Reference breakout default is LSM6DSOX 0x6A; firmware verifies 0x6A and stops if it sees 0x6B |
| unconnected | INT1 / INT2 / DRDY / INTM | Not used by the current polling/FIFO firmware |

The Adafruit breakout documents ADM as the magnetometer address-select pin: pulling it high changes the LIS3MDL from 0x1C to 0x1E. The same board documents ADAG as the accel/gyro address-select pin: pulling it high changes 0x6A to 0x6B. Revision 0.3 standardizes on 0x1E + 0x6A.

## 4.3 ToF bus: hardware I2C1

| Pico 2 W | Pimoroni VL53L5CX | Reference setting |
| --- | --- | --- |
| GP2 | SDA | I2C1 SDA |
| GP3 | SCL | I2C1 SCL |
| 3V3 | VCC | Sensor supply |
| GND | GND | Common ground |

## 4.4 Validated bus speeds

| Bus | Devices | Speed |
| --- | --- | --- |
| I2C0 | LSM6DSOX + LIS3MDL | 400 kHz |
| I2C1 | VL53L5CX | 1 MHz |

## 4.5 Power-on sequence

1. Disconnect USB before changing wiring.

2. Wire GND first, then 3.3 V, then SDA/SCL and ADM.

3. Check that 3.3 V is not shorted to GND and that breadboard power rails are continuous end-to-end; many breadboards split the rails at the midpoint.

4. Connect USB with no servos or other high-current loads attached.

5. If anything becomes hot or the Pico repeatedly disconnects, remove power and re-check wiring before retrying.

# 5. MicroPython environment

Revision 0.3 uses MicroPython, not Arduino. The VL53L5CX path relies on the Pimoroni Pico libraries, so use a Pimoroni RP2350/Pico 2 W MicroPython build rather than assuming stock MicroPython contains the required breakout module.

Pimoroni maintains a separate RP2350 repository for Pico 2 / Pico 2 W builds. The reference prototype was validated with the Pico 2 W build identified by the runtime string below; later compatible builds may also work, but record the exact build when publishing reproduction results.

```text
MicroPython pico2_w_2025_09_19, on 2025-09-22
MicroPython 1.27.0 preview
board/build: rpi_pico2_w
```

## 5.1 Flashing

1. Download the Raspberry Pi Pico 2 W build from the Pimoroni RP2350 MicroPython releases: https://github.com/pimoroni/pimoroni-pico-rp2350/releases. The validated runtime belongs to the `pico2_w_2025_09_19` build lineage; record the exact runtime string for every reproduction.

2. Unplug the Pico. Hold BOOTSEL while reconnecting USB so the boot drive appears.

3. Copy the appropriate .uf2 file to the boot drive. The Pico reboots into MicroPython.

4. Open Thonny, select the Raspberry Pi Pico / MicroPython interpreter and confirm you can open the REPL.

5. Record the firmware fingerprint before running project code.

```python
import os
import sys
print(os.uname())
print(sys.implementation)
```

# 6. VL53L5CX firmware blob

The VL53L5CX contains internal firmware that must be uploaded during initialization. The current Pimoroni MicroPython driver expects the binary firmware blob to be present in the Pico filesystem. In the validated setup the file is named exactly:

```text
/vl53l5cx_firmware.bin
```

Copy the blob to the root of the Pico filesystem using Thonny or mpremote before constructing the VL53L5CX object. The source used during prototype bring-up came from the ST VL53L5CX ULD firmware distribution. This repository does **not** bundle the blob; see [`tools/fetch_vl53l5cx_firmware.py`](../tools/fetch_vl53l5cx_firmware.py) and [`THIRD_PARTY_NOTICES.md`](../THIRD_PARTY_NOTICES.md).

```python
import os
print(os.listdir())
# Confirm that 'vl53l5cx_firmware.bin' appears in the root listing.
```

> **Why initialization pauses**
> The first VL53L5CX constructor call uploads the sensor firmware over I2C. A delay of several seconds at startup is expected; do not enable the LSM FIFO before this upload in the reference firmware, otherwise the IMU FIFO can accumulate unnecessary backlog while Python is occupied.

# 7. Bring-up sequence

When assembling another unit, bring the hardware up in layers. Do not start with the entire mapping stack. Each step below has a clear pass condition and isolates a different class of fault.

## 7.1 Scan both buses

```python
from machine import Pin, I2C

imu = I2C(0, sda=Pin(4), scl=Pin(5), freq=400_000)
tof = I2C(1, sda=Pin(2), scl=Pin(3), freq=1_000_000)

print('IMU:', [hex(x) for x in imu.scan()])
print('ToF:', [hex(x) for x in tof.scan()])
```

Reference pass condition: IMU bus contains 0x1E and 0x6A; ToF bus contains 0x29. If the LIS3MDL appears at 0x1C, check ADM before proceeding.

## 7.2 Verify sensor identities

Read WHO_AM_I from both ST motion sensors. The known-good values are LSM6DSOX 0x6C and LIS3MDL 0x3D. A bus scan proves that something ACKs an address; WHO_AM_I proves it is the device the firmware expects.

## 7.3 Magnetometer multi-byte-read requirement

For the LIS3MDL, multi-byte output reads must set the sub-address auto-increment bit. The working MicroPython read starts at OUT_X_L with bit 7 set:

```python
raw = i2c.readfrom_mem(0x1E, 0x28 | 0x80, 6)
mx, my, mz = struct.unpack('<hhh', raw)
```

The reference magnetometer configuration uses ultra-high-performance X/Y/Z, +/-4 gauss, continuous conversion, an internal 20 Hz ODR, block-data-update enabled, and host consumption at approximately 10 Hz. Firmware checks STATUS_REG ZYXDA before consuming a sample and retries a failed transaction once after a short delay.

## 7.4 Bring up VL53L5CX alone

Use 8x8 resolution at 15 Hz. Do not configure 8x8 at 60 Hz: 60 Hz is associated with lower-resolution 4x4 operation, not the 64-zone mode used here. Confirm that distance_avg, reflectance_avg and the 64-element distance/reflectance arrays update when objects move in front of the sensor.

## 7.5 Run the combined self-test

Once the individual devices work, use the validated combined firmware rather than combining the standalone examples by hand. The current reference file is:

```text
firmware/pico2w/diagnostics/reproducible_sensor_stack.py
```

For publication, this file can be renamed to a stable release filename, but keep the source hash and version history so validation results remain traceable.

# 8. Reproducible combined firmware

The v0.5 firmware encodes the acquisition architecture rather than the quirks of one physical IMU. It uses a hardware FIFO for accel/gyro, embeds one LSM6DSOX timestamp record per FIFO time slot, discovers the unit-specific oscillator trim, and continuously fits the LSM clock to the Pico clock.

## 8.1 Fixed design choices

| Setting | Reference value |
| --- | --- |
| IMU physical bus | I2C0, GP4 SDA / GP5 SCL, 400 kHz |
| ToF physical bus | I2C1, GP2 SDA / GP3 SCL, 1 MHz |
| LSM6DSOX nominal accel mode | 104 Hz, +/-4 g |
| LSM6DSOX nominal gyro mode | 104 Hz, +/-250 dps |
| LIS3MDL internal ODR | 20 Hz; host consumes about 10 Hz |
| VL53L5CX mode | 8x8, 15 Hz |
| FIFO timestamping | One timestamp record per fastest FIFO time slot |
| Clock-model window | Rolling least-squares fit over latest 8 correlation points |

## 8.2 Per-unit values discovered automatically

- INTERNAL_FREQ_FINE, read as a signed 8-bit correction value.

- Factory-estimated LSM timestamp tick and actual ODR derived from the sensor trim.

- Observed timestamp ticks per FIFO slot.

- Measured Pico microseconds per LSM timestamp tick from repeated direct clock-correlation reads.

- Measured real IMU ODR from the runtime clock model.

- Clock-fit RMS residual and factory-versus-measured difference in ppm.

> **Never infer time from sample number**
> Downstream tracking code must use sensor timestamps mapped onto the common time base. It must not calculate time as sample_index / 104 or sample_index / 101.04. A second LSM6DSOX may run at a different real rate while still being completely healthy.

## 8.3 Why the FIFO is essential

A complete VL53L5CX 8x8 frame read takes roughly 19 ms on the reference build and can occasionally take close to 29 ms. Direct host polling of the IMU would leave large timing holes during those transfers. With hardware FIFO batching, the LSM6DSOX keeps producing accel/gyro/timestamp records while MicroPython is occupied and the Pico drains the backlog afterward.

```text
LSM6DSOX hardware clock
    |
    +-- accel ----\
    +-- gyro ------+--> hardware FIFO --> Pico when available
    +-- timestamp-/

VL53L5CX on separate I2C1 --> ~19 ms frame read does not stop IMU sampling
```

## 8.4 Pico <-> LSM clock model

Every few seconds the firmware brackets a direct 32-bit LSM timestamp read with Pico monotonic timestamps and associates the sensor timestamp with the midpoint of the bracket. A rolling linear fit estimates:

```text
pico_time_us = intercept + slope * lsm_timestamp_ticks
```

The fitted slope is the empirically observed microseconds per LSM timestamp tick. This is the time conversion downstream code should prefer; INTERNAL_FREQ_FINE remains a useful independent factory-derived estimate and startup sanity check.

## 8.5 Companion firmware fingerprint

| File | SHA-256 |
| --- | --- |
| firmware/pico2w/diagnostics/reproducible_sensor_stack.py | ee176394004aa66795aeda0645b6032a7f7f3e485b538c0fea7349c8b864d49c |

# 9. Understanding the self-test

The firmware runs a startup validation window and prints a REPRODUCIBILITY SELF TEST block. Its purpose is to answer a practical question: “Is this newly assembled sensor stack behaving like a valid member of the reference design?” It does not require the new unit to match the original unit’s oscillator trim.

| Check | What PASS means |
| --- | --- |
| Deterministic I2C addresses | 0x6A LSM6DSOX, 0x1E LIS3MDL, 0x29 VL53L5CX are present on the intended buses. |
| Sensor identities | WHO_AM_I values match LSM6DSOX and LIS3MDL. |
| FIFO structural integrity | No unknown tags, FIFO I/O errors, full/overrun conditions, slot jumps or bad timestamp metadata during the validation window. |
| Stream alignment | Accel, gyro and timestamp record counts remain aligned. |
| Timestamp consistency | Raw timestamp spacing is consistent with the selected FIFO BDR relationship; this is independent of one particular device’s real ODR. |
| Magnetometer acquisition | At least 90% of attempted reads succeeded. This checks acquisition reliability, not magnetic heading quality. |
| ToF acquisition | At least 12 fps observed during the test and zero ToF I/O errors. Normal operation targets about 15 fps. |
| Clock model | Enough Pico<->LSM points exist to form a positive, usable linear clock model. |

> **Magnetometer PASS is not “compass calibrated”**
> The current self-test only confirms reliable LIS3MDL communication and sample acquisition. A magnetically distorted room can still produce perfectly valid I2C data. Heading corrections must later be confidence-gated using calibrated magnitude, stability and continuity checks.

# 10. Reference validation result

The following is the reference-unit result recorded after the v0.5 refactor. These values are evidence that this unit is healthy; they are not constants that another build must reproduce exactly.

| Metric | Reference run |
| --- | --- |
| I2C addresses | 0x1E + 0x6A on IMU bus; 0x29 on ToF bus |
| INTERNAL_FREQ_FINE | -20 (raw 0xEC) |
| Factory-estimated real IMU ODR | 101.046720 Hz |
| Measured clock-model ODR at self-test | 101.057830 Hz |
| Measured clock-model tick | 25.769074 us |
| Clock fit RMS at self-test | 2.09 us |
| 12 s FIFO records | 1213 accel / 1213 gyro / 1213 timestamp |
| Timestamp delta | min 384 / avg 384.000 / max 384 ticks |
| Maximum FIFO backlog | 12 words during startup self-test |
| Magnetometer | 120/120 successful; 0 retries; 0 drops |
| VL53L5CX | 179 frames; 14.907 fps; 0 errors |
| Longest ToF get_data() during self-test | 28,770 us |
| Overall | SYSTEM READY: PASS |

After the startup window, the rolling eight-point clock model remained stable around 25.769 us/tick and approximately 101.058 Hz while accel, gyro and timestamp records continued to match, the magnetometer remained 10/10 successful per report, and the ToF stream remained close to 15 fps with no reported I/O errors.

# 11. Calibration and data-quality work

## 11.1 Stationary accel/gyro characterization

1. Rigidly clamp the assembled sensor head and record at least 60 seconds.

2. Calculate mean and standard deviation of gx, gy and gz. The means form an initial zero-rate bias estimate.

3. Calculate acceleration-vector magnitude. Stationary magnitude should be close to local gravitational acceleration, but do not force individual axes to +/-g unless the sensor is deliberately aligned to that axis.

4. Repeat on several orientations later to estimate accelerometer offset/scale and cross-axis effects.

5. Repeat at different temperatures if long-term drift becomes important.

## 11.2 Magnetometer calibration and confidence gating

- Calibrate after the sensor is installed in its rigid final prototype assembly; nearby electronics and fasteners are part of the magnetic environment.

- Collect raw mx/my/mz while slowly rotating the entire assembly through many orientations. Preserve the raw file before applying corrections.

- Fit hard-iron offset and soft-iron scale/skew offline.

- Do not use field direction blindly. Reject/down-weight magnetic corrections when calibrated field magnitude changes implausibly, jumps abruptly or is inconsistent with recent history.

- For room mapping, geographic north is not required. The magnetometer is best treated as an occasional long-term yaw reference when conditions are trustworthy.

The breadboard reference currently measures a magnetic magnitude well above the typical terrestrial field, which is a useful reminder that indoor electronics and ferrous objects can dominate a magnetometer even when the sensor itself is communicating perfectly.

## 11.3 Flat-wall ToF characterization

| Distance | Procedure | Record |
| --- | --- | --- |
| 0.5 m | Large matte wall, sensor square-on, static 20-30 s | Per-zone mean, standard deviation, invalid/status rate |
| 1.0 m | Repeat without changing sensor settings | Per-zone bias vs tape-measured distance |
| 2.0 m | Repeat | Noise growth and edge-zone behaviour |
| 3.0 m if practical | Repeat | Useful room-scale range and failure modes |

Do not “calibrate away” every observed difference immediately. First establish whether bias depends on range, zone, incidence angle, reflectance and scene edges. Those dependencies may later become an explicit calibration model or an input to a learned confidence model.

## 11.4 Rigid sensor-to-sensor transform

Before mapping, define a module coordinate frame and measure the relative rotation and translation between the LSM6DSOX axes and the VL53L5CX optical frame. This extrinsic transform should be stored as calibration metadata rather than baked into visualization code.

# 12. Troubleshooting lessons already learned

| Symptom | Likely cause / lesson | Action |
| --- | --- | --- |
| Combined sensor stack becomes unreliable on one bus | VL53L5CX full-frame transactions monopolize a shared bus and interact badly with high-rate IMU/magnetometer traffic | Use the validated split-bus topology. |
| LIS3MDL multi-byte read gives EIO / nonsense | Auto-increment bit missing from the sub-address | Read starting at OUT_X_L \| 0x80. |
| LIS3MDL appears at 0x1C or behaves inconsistently | ADM address-select not at the reference high state | Power off; wire ADM directly to 3.3 V; confirm 0x1E repeatedly. |
| Mag reads intermittently fail while LSM traffic is active | Bus/scheduling interaction made worse by coincident transactions | Use 20 Hz internal ODR, BDU, ZYXDA check, 10 Hz host consumption, 5 ms phase offset and one retry. Keep failures from blocking accel/gyro. |
| IMU appears to run at ~101 Hz instead of 104 Hz | Real LSM oscillator differs from nominal; not necessarily lost samples | Read INTERNAL_FREQ_FINE and trust hardware FIFO timestamps + runtime clock correlation. |
| 20-30 ms gaps appear in direct IMU polling | Python is blocked during full ToF frame retrieval | Use LSM6DSOX hardware FIFO; do not poll the IMU directly as the production acquisition mechanism. |
| Huge serial lines disturb timing | Formatting/transmitting 64-zone ASCII frames is expensive | Keep diagnostics terse; production transport should be buffered and binary/structured. |
| ToF configured for 8x8 at 60 Hz | Rate/resolution assumption is wrong | Use 8x8 at 15 Hz; 4x4 is the mode associated with higher maximum rate. |
| Magnetic magnitude is large or varies by location | Local hard/soft iron or current-generated field | Calibrate final assembly and gate magnetometer yaw corrections by confidence. |

# 13. Reproduction checklist

A second builder - or future you - should be able to treat the following as a release acceptance checklist. Save the completed values alongside the firmware version and hardware photos.

| Done | Criterion |
| --- | --- |
| [ ] | Pico 2 W runs a recorded Pimoroni RP2350 MicroPython build. |
| [ ] | VL53L5CX firmware blob is present in the Pico filesystem root. |
| [ ] | IMU bus is GP4/GP5 at 400 kHz; ToF bus is GP2/GP3 at 1 MHz. |
| [ ] | ADM is wired to 3.3 V and repeated scans show LIS3MDL at 0x1E. |
| [ ] | LSM6DSOX consistently appears at 0x6A and VL53L5CX at 0x29. |
| [ ] | WHO_AM_I: LSM6DSOX=0x6C; LIS3MDL=0x3D. |
| [ ] | v0.5 self-test reports FIFO structural integrity PASS. |
| [ ] | Accel/gyro/timestamp streams are aligned and timestamp consistency PASS. |
| [ ] | Magnetometer acquisition PASS; any retry/drop counts are recorded. |
| [ ] | ToF acquisition PASS at close to 15 fps with zero I/O errors. |
| [ ] | Pico<->LSM clock model PASS; measured ODR recorded but not hard-coded elsewhere. |
| [ ] | Overall SYSTEM READY: PASS. |
| [ ] | Rigid mounting and axis labels completed before collecting motion datasets. |
| [ ] | Raw calibration datasets and exact firmware hash saved with the unit record. |

## 13.1 Reproduction record template

| Field | Record for this unit |
| --- | --- |
| Build / unit ID |  |
| Date assembled |  |
| Pico 2 W board / notes |  |
| MicroPython os.uname() |  |
| Project firmware commit/hash |  |
| LSM6DSOX WHO_AM_I |  |
| LIS3MDL WHO_AM_I |  |
| INTERNAL_FREQ_FINE |  |
| Factory-estimated ODR |  |
| Measured clock-model ODR |  |
| Clock-model RMS |  |
| Max FIFO backlog |  |
| Mag success / attempts |  |
| Observed ToF fps |  |
| Max ToF get_data() time |  |
| SYSTEM READY result |  |
| Mechanical/extrinsic calibration file |  |

# 14. Open-source project structure

The project is structured so a contributor can clone the repository, assemble the reference hardware, run the self-test and create a comparable reproduction record before changing the estimator. See [Repository layout](repository-layout.md) for the canonical tree.

## 14.1 Documentation conventions

- Mark every major feature as **VALIDATED**, **EXPERIMENTAL** or **PLANNED**. Do not let a roadmap item read like a measured capability.
- Keep a changelog entry whenever register settings, bus topology, packet formats or calibration conventions change.
- Publish raw validation logs or compact summaries alongside release tags so other builders can compare their hardware.
- Record exact firmware versions, source hashes and build strings. Avoid “latest” as the only reproducibility instruction.
- Keep sensor coordinate-frame conventions and units explicit in every packet/schema document.
- Prefer links/fetch tooling for upstream vendor firmware or libraries unless redistribution rights and notices are deliberately reviewed.
- When contributors report failures, ask for bus scan, WHO_AM_I, self-test block, firmware build string and wiring photo before debugging higher-level mapping code.

## 14.2 What not to promise yet

The current prototype has demonstrated synchronized sparse-depth and inertial acquisition, not a complete tracking product. Public documentation should avoid claiming centimetre-level pose accuracy, robust loop closure, dense reconstruction, outdoor sunlight performance, dynamic-object handling or headset-ready power/thermal performance until those have been measured.

# 15. Next development stage

With the acquisition stack now reproducible, the next engineering milestone is a real Pico-to-PC data path rather than another diagnostic loop. The firmware should buffer compact records and stream them over USB; the PC should decode, timestamp, save and visualize them without requiring the Pico to format giant text lines.

## 15.1 Planned record types

| Record | Core contents |
| --- | --- |
| IMU | LSM timestamp + accel XYZ + gyro XYZ |
| MAG | Pico timestamp + raw/calibrated magnetic XYZ + quality flags |
| TOF | Pico frame-observed timestamp + estimated LSM timestamp + distance[64] + reflectance[64] + later quality/status fields |
| CLOCK_SYNC | Pico timestamp + raw/unwrapped LSM timestamp + correlation quality metadata |
| META | Firmware version, sensor modes, units, axis mapping, calibration IDs, clock model parameters |

Once that logger exists, the first useful PC-side milestones are a live 8x8 depth viewer, synchronized orientation visualization, raw dataset recording, stationary bias estimation, rigid sensor-frame calibration, and then a first motion-compensated point-cloud accumulation experiment.

## 15.2 Longer-term path

- Single-sensor motion-compensated point-cloud accumulation.

- Known-pose pan/tilt scan rig to validate geometry independently of freehand tracking.

- Pose estimation using gyro/accel with confidence-gated magnetometer correction.

- Sparse scan matching / relative pose correction from ToF observations.

- Simulation and learned confidence/drift models that operate on explicit uncertainty rather than hallucinating absolute pose.

- Dual-ToF / dual-IMU headset geometry later, with explicit inter-sensor calibration and ToF time multiplexing if optical interference is observed.

# 16. References

1. Adafruit ST 9-DoF Combo Breakouts and Wings - pinouts, I2C address pins and breakout wiring
<https://learn.adafruit.com/st-9-dof-combo?view=all>

2. STMicroelectronics LSM6DSOX datasheet
<https://www.st.com/resource/en/datasheet/lsm6dsox.pdf>

3. STMicroelectronics AN5272 - LSM6DSOX application note / FIFO and timestamp behaviour
<https://www.st.com/resource/en/application_note/an5272-lsm6dsox-alwayson-3axis-accelerometer-and-3axis-gyroscope-stmicroelectronics.pdf>

4. STMicroelectronics LIS3MDL datasheet
<https://www.st.com/resource/en/datasheet/lis3mdl.pdf>

5. STMicroelectronics VL53L5CX datasheet
<https://www.st.com/resource/en/datasheet/vl53l5cx.pdf>

6. STMicroelectronics UM2884 - VL53L5CX ULD user manual
<https://www.st.com/resource/en/user_manual/um2884-a-guide-to-using-the-vl53l5cx-multizone-timeofflight-ranging-sensor-with-wide-field-of-view-ultra-lite-driver-uld-stmicroelectronics.pdf>

7. Pimoroni Pico MicroPython for RP2350 / Pico 2 boards
<https://github.com/pimoroni/pimoroni-pico-rp2350>

8. Pimoroni MicroPython setup notes
<https://github.com/pimoroni/pimoroni-pico/blob/main/setting-up-micropython.md>

9. Pimoroni Pico libraries / VL53L5CX support
<https://github.com/pimoroni/pimoroni-pico>

10. Validated VL53L5CX firmware blob source used during prototype bring-up
<https://github.com/ST-mirror/VL53L5CX_ULD_driver/blob/no-fw/lite/en/vl53l5cx_firmware.bin>

> **Publication note**
> Before an open-source release, re-check upstream links, library/API versions and third-party licensing. Vendor documentation can change after this guide is published; the project should preserve the exact tested firmware files and hashes for every tagged release.

# Appendix A. Public provenance

The public diagnostic baseline is [`firmware/pico2w/diagnostics/reproducible_sensor_stack.py`](../firmware/pico2w/diagnostics/reproducible_sensor_stack.py). It is derived from the validated local v0.5 candidate whose SHA-256 was `ee176394004aa66795aeda0645b6032a7f7f3e485b538c0fea7349c8b864d49c`.

The earlier shared-bus, direct-polling, FIFO stress and timestamp-correlation scripts are intentionally not part of the builder-facing source tree. Their engineering conclusions are preserved in [Development history](development-history.md). Future development history should live in Git commits, issues, ADRs and tagged releases rather than dated scratch filenames.
