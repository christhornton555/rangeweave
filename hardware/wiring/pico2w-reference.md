# Pico 2 W reference wiring

**Status: VALIDATED reference topology.**

## Power

| Pico | Destination |
|---|---|
| 3V3(OUT), physical pin 36 | breadboard 3.3-V rail -> both sensor VCC/VIN |
| GND | common ground rail -> both sensor GND |

## I2C0 - IMU / magnetometer

| Pico 2 W | Adafruit 9-DoF | Notes |
|---|---|---|
| GP4 | SDA | hardware I2C0, 400 kHz |
| GP5 | SCL | hardware I2C0, 400 kHz |
| 3V3 | ADM | **mandatory reference wiring**; forces LIS3MDL to `0x1E` |
| leave default/open | ADAG | validated breakout state gives LSM6DSOX `0x6A`; firmware stops if identity/address is wrong |
| unconnected | INT1 / INT2 / DRDY / INTM | not used by current polling/FIFO diagnostic |

Expected scan: `0x1E`, `0x6A`.

## I2C1 - depth

| Pico 2 W | Pimoroni VL53L5CX | Notes |
|---|---|---|
| GP2 | SDA | hardware I2C1, 1 MHz |
| GP3 | SCL | hardware I2C1, 1 MHz |
| 3V3 | VCC | sensor supply |
| GND | GND | common ground |

Expected scan: `0x29`.

## Important

Do **not** merge these onto one shared physical I2C bus in the reference build. The split-bus topology is the architecture that survived the combined workload testing.

Before collecting motion datasets, move the sensors from a flexible breadboard arrangement onto a rigid head and label the sensor axes.
