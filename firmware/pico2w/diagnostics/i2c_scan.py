"""Reference split-bus I2C scan for the Pico 2 W sensor stack.

Expected wiring:
  I2C0: GP4 SDA / GP5 SCL -> LSM6DSOX + LIS3MDL
  I2C1: GP2 SDA / GP3 SCL -> VL53L5CX
  ADM -> 3V3 so LIS3MDL is deterministic at 0x1E

Expected devices:
  I2C0: 0x1E, 0x6A
  I2C1: 0x29
"""

from machine import Pin, I2C

imu_bus = I2C(0, sda=Pin(4), scl=Pin(5), freq=400_000)
tof_bus = I2C(1, sda=Pin(2), scl=Pin(3), freq=1_000_000)

imu_devices = imu_bus.scan()
tof_devices = tof_bus.scan()

print("IMU bus:", ["0x{:02X}".format(x) for x in imu_devices])
print("ToF bus:", ["0x{:02X}".format(x) for x in tof_devices])

expected_imu = {0x1E, 0x6A}
expected_tof = {0x29}

imu_ok = expected_imu.issubset(set(imu_devices))
tof_ok = expected_tof.issubset(set(tof_devices))

if 0x1C in imu_devices and 0x1E not in imu_devices:
    print("FAIL: LIS3MDL is at 0x1C. Connect ADM directly to 3V3 for the reference build.")
elif not imu_ok:
    print("FAIL: expected 0x1E and 0x6A on the IMU bus.")
else:
    print("PASS: IMU bus")

if not tof_ok:
    print("FAIL: expected VL53L5CX at 0x29 on the ToF bus.")
else:
    print("PASS: ToF bus")
