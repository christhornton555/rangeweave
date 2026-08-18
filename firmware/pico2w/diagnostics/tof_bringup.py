"""Standalone VL53L5CX bring-up on the reference Pico 2 W ToF bus.

Requires Pimoroni's RP2350 MicroPython build with breakout_vl53l5cx and a
`vl53l5cx_firmware.bin` file in the Pico filesystem root.
"""

from machine import Pin, I2C
import os
import time
import breakout_vl53l5cx

TOF_ADDR = 0x29
FIRMWARE_FILENAME = "vl53l5cx_firmware.bin"

if FIRMWARE_FILENAME not in os.listdir():
    raise RuntimeError(
        "{} is missing from the Pico filesystem root".format(FIRMWARE_FILENAME)
    )

bus = I2C(1, sda=Pin(2), scl=Pin(3), freq=1_000_000)
devices = bus.scan()
print("ToF bus:", ["0x{:02X}".format(x) for x in devices])

if TOF_ADDR not in devices:
    raise RuntimeError("VL53L5CX not found at 0x29")

print("Initialising VL53L5CX...")
print("Firmware upload may take a few seconds.")
start = time.ticks_ms()

sensor = breakout_vl53l5cx.VL53L5CX(bus)

elapsed = time.ticks_diff(time.ticks_ms(), start)
print("Initialised in {} ms".format(elapsed))

sensor.set_resolution(breakout_vl53l5cx.RESOLUTION_8X8)
sensor.set_ranging_frequency_hz(15)
sensor.start_ranging()

print("PASS: 8x8 ranging started at 15-Hz configuration")
print("Move objects in front of the sensor. Ctrl-C to stop.\n")

while True:
    if sensor.data_ready():
        data = sensor.get_data()
        print(
            "Average distance: {} mm | Average reflectance: {} %".format(
                data.distance_avg, data.reflectance_avg
            )
        )
        distances = data.distance
        for y in range(8):
            row = []
            for x in range(8):
                row.append("{:5d}".format(distances[y * 8 + x]))
            print(" ".join(row))
        print()
    time.sleep_ms(5)
