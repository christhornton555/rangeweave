# Reference prototype BOM

| Qty | Part | Role |
|---:|---|---|
| 1 | Raspberry Pi Pico 2 W | RP2350 reference MCU / USB connection |
| 1 | Adafruit LSM6DSOX + LIS3MDL Precision 9-DoF breakout | accel, gyro, magnetometer |
| 1 | Pimoroni VL53L5CX 8x8 ToF breakout | 64-zone active depth |
| 1 | Breadboard | bring-up only |
| several | short jumper wires | 3.3-V power, GND and two I2C buses |
| 1 | USB cable | power/programming/console |
| optional | rigid plywood/acrylic/perfboard/3D-printed plate | required before meaningful motion/calibration datasets |

Do not power hobby servos from the Pico 3V3 rail. If a pan/tilt validation rig is added, use an appropriate external servo supply with common ground.
