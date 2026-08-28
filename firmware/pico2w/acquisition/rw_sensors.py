"""Validated sensor configuration adapted for Rangeweave acquisition.

No protocol or USB semantics live here. The configuration mirrors the validated v0.5
reference stack: LSM6DSOX/LIS3MDL on I2C0, VL53L5CX on I2C1, and timestamped LSM FIFO.
"""

from machine import Pin, I2C
import struct
import time

import breakout_vl53l5cx

IMU_SDA = 4
IMU_SCL = 5
IMU_FREQ = 400_000
TOF_SDA = 2
TOF_SCL = 3
TOF_FREQ = 1_000_000

LSM_ADDR = 0x6A
MAG_ADDR = 0x1E
TOF_ADDR = 0x29

LSM_FIFO_BDR_CODE = 0x04
LSM_TIMESTAMP_DECIMATION_CODE = 0x01

LSM_WHO_AM_I = 0x0F
LSM_FIFO_CTRL1 = 0x07
LSM_FIFO_CTRL2 = 0x08
LSM_FIFO_CTRL3 = 0x09
LSM_FIFO_CTRL4 = 0x0A
LSM_CTRL1_XL = 0x10
LSM_CTRL2_G = 0x11
LSM_CTRL3_C = 0x12
LSM_CTRL10_C = 0x19
LSM_FIFO_STATUS1 = 0x3A
LSM_FIFO_STATUS2 = 0x3B
LSM_TIMESTAMP0 = 0x40
LSM_TIMESTAMP2 = 0x42
LSM_INTERNAL_FREQ_FINE = 0x63
LSM_FIFO_DATA_OUT_TAG = 0x78
LSM_FIFO_DATA_OUT_X_L = 0x79

LSM_TAG_GYRO = 0x01
LSM_TAG_ACCEL = 0x02
LSM_TAG_TIMESTAMP = 0x04

MAG_WHO_AM_I = 0x0F
MAG_CTRL_REG1 = 0x20
MAG_CTRL_REG2 = 0x21
MAG_CTRL_REG3 = 0x22
MAG_CTRL_REG4 = 0x23
MAG_CTRL_REG5 = 0x24
MAG_STATUS_REG = 0x27
MAG_OUT_X_L = 0x28

LSM_CTRL1_XL_VALUE = 0x48
# 104 Hz, +/-500 dps. The earlier +/-250 dps setting was exceeded during a
# physically valid mixed-axis boresight motion, so calibration capture now keeps
# additional rate headroom without changing the ODR.
LSM_CTRL2_G_VALUE = 0x44
LSM_CTRL3_C_VALUE = 0x44
LSM_FIFO_CTRL3_VALUE = 0x44
LSM_FIFO_CTRL4_VALUE = 0x46
MAG_CTRL_VALUES = bytes([0x74, 0x00, 0x00, 0x0C, 0x40])

TOF_ROWS = 8
TOF_COLS = 8
TOF_HZ = 15


def signed_u8(value):
    return value if value < 128 else value - 256


class PicoClock:
    """Extend MicroPython's wrapping ticks_us() into monotonic Python integers."""

    def __init__(self):
        self._last = time.ticks_us()
        self._elapsed = 0

    def now_us(self):
        now = time.ticks_us()
        self._elapsed += time.ticks_diff(now, self._last)
        self._last = now
        return self._elapsed


class SensorStack:
    def __init__(self):
        self.clock = PicoClock()
        self.imu_bus = I2C(0, sda=Pin(IMU_SDA), scl=Pin(IMU_SCL), freq=IMU_FREQ)
        self.tof_bus = I2C(1, sda=Pin(TOF_SDA), scl=Pin(TOF_SCL), freq=TOF_FREQ)
        self.tof = None
        self.freq_fine_raw = 0
        self.freq_fine = 0
        self.lsm_whoami = 0
        self.mag_whoami = 0
        self.last_mag_bus_activity_us = -1_000_000

    def verify_bus_topology(self):
        imu_devices = self.imu_bus.scan()
        tof_devices = self.tof_bus.scan()

        if LSM_ADDR not in imu_devices:
            if 0x6B in imu_devices:
                raise RuntimeError("LSM6DSOX is at 0x6B; check ADAG/address wiring")
            raise RuntimeError("LSM6DSOX not found at 0x6A")

        if MAG_ADDR not in imu_devices:
            if 0x1C in imu_devices:
                raise RuntimeError("LIS3MDL is at 0x1C; connect ADM directly to 3V3")
            raise RuntimeError("LIS3MDL not found at 0x1E")

        if TOF_ADDR not in tof_devices:
            raise RuntimeError("VL53L5CX not found at 0x29")

    def verify_identity(self):
        self.lsm_whoami = self.imu_bus.readfrom_mem(LSM_ADDR, LSM_WHO_AM_I, 1)[0]
        self.mag_whoami = self.imu_bus.readfrom_mem(MAG_ADDR, MAG_WHO_AM_I, 1)[0]
        if self.lsm_whoami != 0x6C:
            raise RuntimeError("Unexpected LSM6DSOX WHO_AM_I")
        if self.mag_whoami != 0x3D:
            raise RuntimeError("Unexpected LIS3MDL WHO_AM_I")

    def read_factory_timing(self):
        self.freq_fine_raw = self.imu_bus.readfrom_mem(
            LSM_ADDR, LSM_INTERNAL_FREQ_FINE, 1
        )[0]
        self.freq_fine = signed_u8(self.freq_fine_raw)

    def configure_lsm(self):
        self.imu_bus.writeto_mem(LSM_ADDR, LSM_FIFO_CTRL4, b"\x00")
        self.imu_bus.writeto_mem(LSM_ADDR, LSM_FIFO_CTRL3, b"\x00")
        self.imu_bus.writeto_mem(LSM_ADDR, LSM_CTRL1_XL, bytes([LSM_CTRL1_XL_VALUE]))
        self.imu_bus.writeto_mem(LSM_ADDR, LSM_CTRL2_G, bytes([LSM_CTRL2_G_VALUE]))
        self.imu_bus.writeto_mem(LSM_ADDR, LSM_CTRL3_C, bytes([LSM_CTRL3_C_VALUE]))
        self.imu_bus.writeto_mem(LSM_ADDR, LSM_CTRL10_C, b"\x20")
        self.imu_bus.writeto_mem(LSM_ADDR, LSM_TIMESTAMP2, b"\xAA")
        time.sleep_ms(5)
        self.read_lsm_timestamp_raw()

    def configure_mag(self):
        for index, value in enumerate(MAG_CTRL_VALUES):
            self.imu_bus.writeto_mem(MAG_ADDR, MAG_CTRL_REG1 + index, bytes([value]))
        time.sleep_ms(100)

    def configure_tof(self):
        self.tof = breakout_vl53l5cx.VL53L5CX(self.tof_bus)
        self.tof.set_resolution(breakout_vl53l5cx.RESOLUTION_8X8)
        self.tof.set_ranging_frequency_hz(TOF_HZ)
        self.tof.start_ranging()

    def enable_fifo(self):
        self.imu_bus.writeto_mem(LSM_ADDR, LSM_FIFO_CTRL4, b"\x00")
        self.imu_bus.writeto_mem(LSM_ADDR, LSM_FIFO_CTRL1, bytes([48]))
        self.imu_bus.writeto_mem(LSM_ADDR, LSM_FIFO_CTRL2, b"\x00")
        self.imu_bus.writeto_mem(LSM_ADDR, LSM_FIFO_CTRL3, bytes([LSM_FIFO_CTRL3_VALUE]))
        self.imu_bus.writeto_mem(LSM_ADDR, LSM_FIFO_CTRL4, bytes([LSM_FIFO_CTRL4_VALUE]))

    def initialise(self):
        self.verify_bus_topology()
        self.verify_identity()
        self.read_factory_timing()
        self.configure_lsm()
        self.configure_mag()
        # Keep the FIFO disabled during the long VL53L5CX firmware upload/initialisation.
        self.configure_tof()
        self.enable_fifo()

    def read_lsm_timestamp_raw(self):
        raw = self.imu_bus.readfrom_mem(LSM_ADDR, LSM_TIMESTAMP0, 4)
        return struct.unpack("<I", raw)[0]

    def fifo_status(self):
        raw = self.imu_bus.readfrom_mem(LSM_ADDR, LSM_FIFO_STATUS1, 2)
        level = raw[0] | ((raw[1] & 0x03) << 8)
        return level, raw[1]

    def read_fifo_record(self):
        tag_raw = self.imu_bus.readfrom_mem(LSM_ADDR, LSM_FIFO_DATA_OUT_TAG, 1)[0]
        raw = self.imu_bus.readfrom_mem(LSM_ADDR, LSM_FIFO_DATA_OUT_X_L, 6)
        tag_sensor = (tag_raw >> 3) & 0x1F
        tag_cnt = (tag_raw >> 1) & 0x03
        return tag_sensor, tag_cnt, raw

    def read_mag(self):
        retried = False
        for attempt in range(2):
            try:
                status = self.imu_bus.readfrom_mem(MAG_ADDR, MAG_STATUS_REG, 1)[0]
                if not (status & 0x08):
                    self.last_mag_bus_activity_us = self.clock.now_us()
                    return "not_ready", None, retried

                before_us = self.clock.now_us()
                raw = self.imu_bus.readfrom_mem(MAG_ADDR, MAG_OUT_X_L | 0x80, 6)
                after_us = self.clock.now_us()
                self.last_mag_bus_activity_us = after_us
                x, y, z = struct.unpack("<hhh", raw)
                flags = 0x01 if retried else 0x00
                return "ok", (before_us, after_us, x, y, z, status, flags), retried
            except OSError:
                self.last_mag_bus_activity_us = self.clock.now_us()
                if attempt == 0:
                    retried = True
                    time.sleep_ms(2)
                else:
                    return "error", None, True
        return "error", None, retried

    def read_tof(self):
        if not self.tof.data_ready():
            return None

        ready_us = self.clock.now_us()
        data = self.tof.get_data()
        complete_us = self.clock.now_us()

        distances = data.distance
        reflectance = data.reflectance
        if len(distances) < 64 or len(reflectance) < 64:
            raise RuntimeError("VL53L5CX driver returned fewer than 64 zones")

        # The validated Pimoroni binding exposes distance + reflectance but not the
        # underlying ST target_status array, so v0.1 emits field mask 0x0003.
        return ready_us, complete_us, distances, reflectance

    def read_clock_sync(self, tick_extender):
        before_us = self.clock.now_us()
        raw_tick = self.read_lsm_timestamp_raw()
        after_us = self.clock.now_us()
        extended = tick_extender.extend(raw_tick)
        return before_us, extended, after_us

    def config_snapshot(self):
        ctrl1 = self.imu_bus.readfrom_mem(LSM_ADDR, LSM_CTRL1_XL, 1)[0]
        ctrl2 = self.imu_bus.readfrom_mem(LSM_ADDR, LSM_CTRL2_G, 1)[0]
        fifo3 = self.imu_bus.readfrom_mem(LSM_ADDR, LSM_FIFO_CTRL3, 1)[0]
        fifo4 = self.imu_bus.readfrom_mem(LSM_ADDR, LSM_FIFO_CTRL4, 1)[0]
        mag_regs = self.imu_bus.readfrom_mem(MAG_ADDR, MAG_CTRL_REG1 | 0x80, 5)
        return ctrl1, ctrl2, fifo3, fifo4, bytes(mag_regs)
