"""Standalone LSM6DSOX + LIS3MDL bring-up for the reference Pico 2 W build.

This is a readable hardware diagnostic, not the final acquisition path.
The full reproducibility diagnostic later uses the LSM6DSOX hardware FIFO
and timestamp records.
"""

from machine import Pin, I2C
import time
import struct

I2C_SDA = 4
I2C_SCL = 5
I2C_FREQ = 400_000

LSM_ADDR = 0x6A
MAG_ADDR = 0x1E

LSM_WHO_AM_I = 0x0F
LSM_CTRL1_XL = 0x10
LSM_CTRL2_G = 0x11
LSM_CTRL3_C = 0x12
LSM_OUTX_L_G = 0x22
LSM_OUTX_L_A = 0x28

MAG_WHO_AM_I = 0x0F
MAG_CTRL_REG1 = 0x20
MAG_CTRL_REG2 = 0x21
MAG_CTRL_REG3 = 0x22
MAG_CTRL_REG4 = 0x23
MAG_CTRL_REG5 = 0x24
MAG_STATUS_REG = 0x27
MAG_OUT_X_L = 0x28

ACCEL_SCALE = 0.122 * 0.00980665      # +/-4 g -> m/s^2 per LSB
GYRO_SCALE_DPS = 8.75 / 1000.0        # +/-250 dps -> deg/s per LSB
MAG_SCALE_UT = 100.0 / 6842.0         # +/-4 gauss -> microtesla per LSB

bus = I2C(0, sda=Pin(I2C_SDA), scl=Pin(I2C_SCL), freq=I2C_FREQ)

devices = bus.scan()
print("IMU bus:", ["0x{:02X}".format(x) for x in devices])

if MAG_ADDR not in devices:
    if 0x1C in devices:
        raise RuntimeError("LIS3MDL is at 0x1C. Connect ADM directly to 3V3.")
    raise RuntimeError("LIS3MDL not found at 0x1E")
if LSM_ADDR not in devices:
    raise RuntimeError("LSM6DSOX not found at 0x6A")

lsm_id = bus.readfrom_mem(LSM_ADDR, LSM_WHO_AM_I, 1)[0]
mag_id = bus.readfrom_mem(MAG_ADDR, MAG_WHO_AM_I, 1)[0]

print("LSM6DSOX WHO_AM_I: 0x{:02X}".format(lsm_id))
print("LIS3MDL WHO_AM_I:  0x{:02X}".format(mag_id))

if lsm_id != 0x6C:
    raise RuntimeError("Unexpected LSM6DSOX WHO_AM_I")
if mag_id != 0x3D:
    raise RuntimeError("Unexpected LIS3MDL WHO_AM_I")

# LSM6DSOX: 104-Hz nominal accel +/-4 g, gyro +/-250 dps, BDU + IF_INC.
bus.writeto_mem(LSM_ADDR, LSM_CTRL1_XL, bytes([0x48]))
bus.writeto_mem(LSM_ADDR, LSM_CTRL2_G, bytes([0x40]))
bus.writeto_mem(LSM_ADDR, LSM_CTRL3_C, bytes([0x44]))

# LIS3MDL: UHP XY/Z, 20-Hz internal ODR, +/-4 gauss, continuous mode, BDU.
bus.writeto_mem(MAG_ADDR, MAG_CTRL_REG1, bytes([0x74]))
bus.writeto_mem(MAG_ADDR, MAG_CTRL_REG2, bytes([0x00]))
bus.writeto_mem(MAG_ADDR, MAG_CTRL_REG3, bytes([0x00]))
bus.writeto_mem(MAG_ADDR, MAG_CTRL_REG4, bytes([0x0C]))
bus.writeto_mem(MAG_ADDR, MAG_CTRL_REG5, bytes([0x40]))

time.sleep_ms(100)


def read_lsm6dsox():
    gyro_raw = bus.readfrom_mem(LSM_ADDR, LSM_OUTX_L_G, 6)
    accel_raw = bus.readfrom_mem(LSM_ADDR, LSM_OUTX_L_A, 6)
    gx, gy, gz = struct.unpack("<hhh", gyro_raw)
    ax, ay, az = struct.unpack("<hhh", accel_raw)
    return (
        ax * ACCEL_SCALE, ay * ACCEL_SCALE, az * ACCEL_SCALE,
        gx * GYRO_SCALE_DPS, gy * GYRO_SCALE_DPS, gz * GYRO_SCALE_DPS,
    )


def read_lis3mdl():
    status = bus.readfrom_mem(MAG_ADDR, MAG_STATUS_REG, 1)[0]
    if not (status & 0x08):  # ZYXDA: a complete new XYZ sample is available
        return None

    # Bit 7 of the sub-address enables LIS3MDL multi-byte auto-increment.
    raw = bus.readfrom_mem(MAG_ADDR, MAG_OUT_X_L | 0x80, 6)
    mx, my, mz = struct.unpack("<hhh", raw)
    return mx * MAG_SCALE_UT, my * MAG_SCALE_UT, mz * MAG_SCALE_UT


print("PASS: sensor identities and configuration")
print("Move the board. Ctrl-C to stop.\n")

while True:
    ax, ay, az, gx, gy, gz = read_lsm6dsox()
    mag = read_lis3mdl()

    print("ACC  x={:8.3f} y={:8.3f} z={:8.3f} m/s^2".format(ax, ay, az))
    print("GYRO x={:8.2f} y={:8.2f} z={:8.2f} deg/s".format(gx, gy, gz))

    if mag is None:
        print("MAG  waiting for fresh XYZ sample")
    else:
        mx, my, mz = mag
        print("MAG  x={:8.2f} y={:8.2f} z={:8.2f} uT".format(mx, my, mz))

    print()
    time.sleep_ms(100)
