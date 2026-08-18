from machine import Pin, I2C
import time
import struct
import math

import breakout_vl53l5cx


# ============================================================
# Reproducible ToF + 9DoF sensor-stack firmware
# Public diagnostic baseline derived from validated v0.5 candidate
#
# Purpose:
#   - Reproduce the validated dual-I2C architecture.
#   - Never hard-code this particular IMU's measured ~101 Hz.
#   - Discover/calibrate each LSM6DSOX unit at runtime.
#   - Verify FIFO structure, timestamp integrity, magnetometer
#     acquisition, ToF acquisition, and Pico<->LSM clock model.
#   - Print a self-certifying startup report.
#
# Primary register behaviour cross-checked against:
#   ST LSM6DSOX DS12814 Rev 4
#   ST LSM6DSOX AN5272 Rev 5
#   ST LIS3MDL datasheet
#
# IMPORTANT WIRING ASSUMPTIONS FOR A REPRODUCIBLE BUILD:
#
#   I2C0 / IMU bus
#       Pico GP4  -> Adafruit SDA
#       Pico GP5  -> Adafruit SCL
#       Pico 3V3  -> Adafruit VIN
#       Pico GND  -> Adafruit GND
#
#       ADM -> 3V3       (forces LIS3MDL address 0x1E)
#
#       Expected:
#           LSM6DSOX = 0x6A
#           LIS3MDL  = 0x1E
#
#   I2C1 / ToF bus
#       Pico GP2  -> Pimoroni SDA
#       Pico GP3  -> Pimoroni SCL
#       Pico 3V3  -> Pimoroni VCC
#       Pico GND  -> Pimoroni GND
#
#       Expected:
#           VL53L5CX = 0x29
#
#   Leave INT1, INT2, DRDY and INTM disconnected for this
#   polling-based revision.
#
#   The VL53L5CX firmware blob must already exist on the Pico
#   filesystem as required by the Pimoroni driver.
# ============================================================


# ============================================================
# Design configuration
#
# These are DESIGN choices, not per-unit calibration values.
# ============================================================

IMU_SDA = 4
IMU_SCL = 5
IMU_FREQ = 400_000

TOF_SDA = 2
TOF_SCL = 3
TOF_FREQ = 1_000_000

LSM_ADDR = 0x6A
MAG_ADDR = 0x1E
TOF_ADDR = 0x29

# Nominal LSM configuration.
#
# We configure 104 Hz, but NEVER assume the real sensor runs
# at exactly 104 Hz. INTERNAL_FREQ_FINE + measured timestamps
# determine the actual unit-specific timing.
LSM_NOMINAL_ODR_HZ = 104
LSM_ODR_COEFF = 64

# FIFO BDR code 0100 = 104 Hz for both XL and GY.
LSM_FIFO_BDR_CODE = 0x04

# Timestamp FIFO decimation:
#   01 = one timestamp at max(BDR_GY, BDR_XL, BDR_SHUB)
# We keep one timestamp per FIFO time slot in this validation
# / reproducibility revision because it makes integrity checks
# unambiguous.
LSM_TIMESTAMP_DECIMATION_CODE = 0x01

# Magnetometer:
#   internal ODR 20 Hz
#   host consumes at 10 Hz
MAG_PERIOD_US = 100_000
MAG_INITIAL_OFFSET_US = 5_000

# FIFO service opportunity.
#
# This is NOT treated as the IMU sample period. The LSM6DSOX
# samples independently into hardware FIFO. The Pico merely
# empties whatever is waiting.
FIFO_SERVICE_PERIOD_US = 20_000
FIFO_MAX_RECORDS_PER_SERVICE = 192

# ToF polling. The sensor itself is configured to 15 Hz 8x8.
TOF_POLL_US = 5_000

# Keep a small quiet window around LIS3MDL transactions.
FIFO_MAG_GUARD_US = 3_000

# A full 8x8 VL53L5CX get_data() has been measured around
# 19 ms normally, with occasional longer outliers.
TOF_MAG_GUARD_US = 25_000

# Direct LSM timestamp <-> Pico clock correlation.
CLOCK_CORRELATION_PERIOD_US = 5_000_000
CLOCK_CORRELATION_INITIAL_US = 50_000
CLOCK_CORRELATION_MAG_GUARD_US = 3_000

# Keep a rolling least-squares model rather than relying on
# one pair of points.
CLOCK_MODEL_MAX_POINTS = 8

# Human-readable reports.
STATUS_PERIOD_US = 1_000_000
VALUES_PERIOD_US = 10_000_000

# Startup self-test is evaluated after enough time to obtain
# at least three clock-correlation points.
SELF_TEST_DURATION_US = 12_000_000

# Self-test tolerances.
#
# Timestamp slot ticks are derived from the ratio of the
# nominal 40 kHz timestamp clock and the selected ODR
# coefficient. FREQ_FINE affects BOTH, so it largely cancels
# from the raw ticks-per-slot relationship.
EXPECTED_SLOT_TICKS = int(
    round((40_000.0 * LSM_ODR_COEFF) / 6667.0)
)

TIMESTAMP_TICK_TOLERANCE = max(
    2,
    int(round(EXPECTED_SLOT_TICKS * 0.01))
)

# The trim-vs-Pico clock comparison is informative rather
# than the primary source of truth. Pico and LSM oscillators
# both have finite tolerances.
CLOCK_TRIM_WARNING_PPM = 5_000.0

# Communication/performance checks.
SELF_TEST_MIN_MAG_SUCCESS_RATIO = 0.90
SELF_TEST_MIN_TOF_FPS = 12.0


# ============================================================
# LSM6DSOX registers
# ============================================================

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

# FIFO TAG_SENSOR values.
LSM_TAG_GYRO = 0x01
LSM_TAG_ACCEL = 0x02
LSM_TAG_TIMESTAMP = 0x04

# Per-slot completeness mask used only for diagnostics.
SLOT_GYRO_BIT = 0x01
SLOT_ACCEL_BIT = 0x02
SLOT_TIMESTAMP_BIT = 0x04
SLOT_COMPLETE_MASK = (
    SLOT_GYRO_BIT |
    SLOT_ACCEL_BIT |
    SLOT_TIMESTAMP_BIT
)


# ============================================================
# LIS3MDL registers
# ============================================================

MAG_WHO_AM_I = 0x0F

MAG_CTRL_REG1 = 0x20
MAG_CTRL_REG2 = 0x21
MAG_CTRL_REG3 = 0x22
MAG_CTRL_REG4 = 0x23
MAG_CTRL_REG5 = 0x24

MAG_STATUS_REG = 0x27
MAG_OUT_X_L = 0x28


# ============================================================
# Physical scaling
# ============================================================

# LSM6DSOX +/-4 g:
# 0.122 mg/LSB -> m/s^2
ACCEL_SCALE = 0.122 * 0.00980665

# LSM6DSOX +/-250 dps:
# 8.75 mdps/LSB -> deg/s
GYRO_SCALE_DPS = 8.75 / 1000.0

# LIS3MDL +/-4 gauss:
# 6842 LSB/gauss; 1 gauss = 100 uT
MAG_SCALE_UT = 100.0 / 6842.0


# ============================================================
# Small utilities
# ============================================================

def signed_u8(value):
    return value if value < 128 else value - 256


def u32_diff(new_value, old_value):
    return (new_value - old_value) & 0xFFFFFFFF


def abs_int(value):
    return value if value >= 0 else -value


def pass_fail(ok):
    return "PASS" if ok else "FAIL"


# ============================================================
# Wrap-safe Pico monotonic clock
# ============================================================

class PicoClock:

    def __init__(self):
        self._last_tick = time.ticks_us()
        self._elapsed_us = 0

    def now_us(self):
        now = time.ticks_us()

        self._elapsed_us += time.ticks_diff(
            now,
            self._last_tick
        )

        self._last_tick = now

        return self._elapsed_us


# ============================================================
# Rolling Pico <-> LSM clock model
#
# Fits:
#     pico_us = intercept + slope * lsm_unwrapped_ticks
#
# slope is therefore the empirically observed real-world
# microseconds per LSM timestamp tick.
# ============================================================

class ClockModel:

    def __init__(self, max_points):
        self.max_points = max_points
        self.points = []

        self.last_raw = None
        self.last_unwrapped = None

        self.slope_us_per_tick = None
        self.intercept_us = None
        self.rms_us = None

    def point_count(self):
        return len(self.points)

    def _unwrap(self, raw_ticks):

        if self.last_raw is None:
            unwrapped = raw_ticks

        else:
            delta = u32_diff(
                raw_ticks,
                self.last_raw
            )

            # Correlation points are only seconds apart.
            # A delta >= 2^31 would indicate a nonsensical
            # backwards/out-of-order observation.
            if delta >= 0x80000000:
                return None

            unwrapped = (
                self.last_unwrapped + delta
            )

        self.last_raw = raw_ticks
        self.last_unwrapped = unwrapped

        return unwrapped

    def add_point(self, raw_ticks, pico_us):

        unwrapped = self._unwrap(raw_ticks)

        if unwrapped is None:
            return False

        self.points.append(
            (unwrapped, pico_us)
        )

        if len(self.points) > self.max_points:
            self.points.pop(0)

        self._fit()

        return True

    def _fit(self):

        n = len(self.points)

        if n < 2:
            self.slope_us_per_tick = None
            self.intercept_us = None
            self.rms_us = None
            return

        sum_x = 0.0
        sum_y = 0.0

        for x, y in self.points:
            sum_x += x
            sum_y += y

        mean_x = sum_x / n
        mean_y = sum_y / n

        numerator = 0.0
        denominator = 0.0

        for x, y in self.points:

            dx = x - mean_x
            dy = y - mean_y

            numerator += dx * dy
            denominator += dx * dx

        if denominator <= 0.0:
            self.slope_us_per_tick = None
            self.intercept_us = None
            self.rms_us = None
            return

        slope = numerator / denominator
        intercept = mean_y - slope * mean_x

        residual_sq_sum = 0.0

        for x, y in self.points:

            predicted = intercept + slope * x
            residual = y - predicted

            residual_sq_sum += (
                residual * residual
            )

        rms = math.sqrt(
            residual_sq_sum / n
        )

        self.slope_us_per_tick = slope
        self.intercept_us = intercept
        self.rms_us = rms

    def is_ready(self):
        return (
            self.slope_us_per_tick is not None
            and
            self.slope_us_per_tick > 0.0
            and
            len(self.points) >= 2
        )

    def estimate_lsm_unwrapped(self, pico_us):

        if not self.is_ready():
            return None

        return int(
            round(
                (
                    pico_us -
                    self.intercept_us
                )
                /
                self.slope_us_per_tick
            )
        )

    def estimate_lsm_raw(self, pico_us):

        unwrapped = self.estimate_lsm_unwrapped(
            pico_us
        )

        if unwrapped is None:
            return None

        return unwrapped & 0xFFFFFFFF


# ============================================================
# Statistics containers
# ============================================================

def new_interval_stats():

    return {
        "xl": 0,
        "gy": 0,
        "ts": 0,
        "fifo_other": 0,
        "fifo_err": 0,
        "fifo_ovr": 0,
        "fifo_full": 0,
        "fifo_level_max": 0,

        "slot_jump": 0,
        "slot_incomplete": 0,
        "ts_meta_bad": 0,

        "ts_delta_count": 0,
        "ts_delta_sum": 0,
        "ts_delta_min": None,
        "ts_delta_max": 0,

        "mag_attempt": 0,
        "mag_ok": 0,
        "mag_retry_ok": 0,
        "mag_drop": 0,
        "mag_not_ready": 0,

        "tof": 0,
        "tof_err": 0,
        "tof_read_total_us": 0,
        "tof_read_max_us": 0,

        "corr_err": 0,
        "corr_bracket_max_us": 0,
    }


def accumulate(dst, src):

    additive_keys = (
        "xl",
        "gy",
        "ts",
        "fifo_other",
        "fifo_err",
        "slot_jump",
        "slot_incomplete",
        "ts_meta_bad",
        "ts_delta_count",
        "ts_delta_sum",
        "mag_attempt",
        "mag_ok",
        "mag_retry_ok",
        "mag_drop",
        "mag_not_ready",
        "tof",
        "tof_err",
        "tof_read_total_us",
        "corr_err",
    )

    for key in additive_keys:
        dst[key] += src[key]

    # Boolean / latched conditions.
    if src["fifo_ovr"]:
        dst["fifo_ovr"] = 1

    if src["fifo_full"]:
        dst["fifo_full"] = 1

    # Maxima.
    if (
        src["fifo_level_max"] >
        dst["fifo_level_max"]
    ):
        dst["fifo_level_max"] = (
            src["fifo_level_max"]
        )

    if (
        src["ts_delta_max"] >
        dst["ts_delta_max"]
    ):
        dst["ts_delta_max"] = (
            src["ts_delta_max"]
        )

    if (
        src["tof_read_max_us"] >
        dst["tof_read_max_us"]
    ):
        dst["tof_read_max_us"] = (
            src["tof_read_max_us"]
        )

    if (
        src["corr_bracket_max_us"] >
        dst["corr_bracket_max_us"]
    ):
        dst["corr_bracket_max_us"] = (
            src["corr_bracket_max_us"]
        )

    # Minimum timestamp delta.
    src_min = src["ts_delta_min"]

    if src_min is not None:

        if (
            dst["ts_delta_min"] is None
            or
            src_min < dst["ts_delta_min"]
        ):
            dst["ts_delta_min"] = src_min


# ============================================================
# Sensor stack
# ============================================================

class SensorStack:

    def __init__(self):

        self.pico_clock = PicoClock()

        self.imu_bus = I2C(
            0,
            sda=Pin(IMU_SDA),
            scl=Pin(IMU_SCL),
            freq=IMU_FREQ
        )

        self.tof_bus = I2C(
            1,
            sda=Pin(TOF_SDA),
            scl=Pin(TOF_SCL),
            freq=TOF_FREQ
        )

        self.tof = None

        self.freq_fine_raw = 0
        self.freq_fine = 0
        self.freq_factor = 1.0

        self.factory_tick_us = 25.0
        self.factory_odr_hz = float(
            LSM_NOMINAL_ODR_HZ
        )
        self.factory_period_us = (
            1_000_000.0 /
            self.factory_odr_hz
        )

        self.clock_model = ClockModel(
            CLOCK_MODEL_MAX_POINTS
        )

        self.current_slot_cnt = None
        self.current_slot_mask = 0
        self.slot_groups_seen = 0

        self.previous_fifo_timestamp = None

        self.latest_ax = 0.0
        self.latest_ay = 0.0
        self.latest_az = 0.0

        self.latest_gx = 0.0
        self.latest_gy = 0.0
        self.latest_gz = 0.0

        self.latest_mx = 0.0
        self.latest_my = 0.0
        self.latest_mz = 0.0

        self.latest_tof_avg = 0
        self.latest_tof_centre = 0
        self.latest_tof_reflectance = 0

        self.latest_tof_ready_pico_us = 0
        self.latest_tof_est_lsm_ticks = 0

        self.last_mag_bus_activity_us = (
            -1_000_000
        )

    # --------------------------------------------------------
    # Startup / deterministic wiring checks
    # --------------------------------------------------------

    def verify_bus_topology(self):

        imu_devices = self.imu_bus.scan()
        tof_devices = self.tof_bus.scan()

        print(
            "IMU bus:",
            [
                "0x{:02X}".format(x)
                for x in imu_devices
            ]
        )

        print(
            "ToF bus:",
            [
                "0x{:02X}".format(x)
                for x in tof_devices
            ]
        )

        if LSM_ADDR not in imu_devices:

            if 0x6B in imu_devices:
                raise RuntimeError(
                    "LSM6DSOX found at 0x6B, not 0x6A. "
                    "Check ADAG/address wiring."
                )

            raise RuntimeError(
                "LSM6DSOX not found at 0x6A"
            )

        if MAG_ADDR not in imu_devices:

            if 0x1C in imu_devices:
                raise RuntimeError(
                    "LIS3MDL found at 0x1C, not 0x1E. "
                    "For this reproducible build, connect "
                    "ADM directly to 3V3."
                )

            raise RuntimeError(
                "LIS3MDL not found at 0x1E"
            )

        if TOF_ADDR not in tof_devices:
            raise RuntimeError(
                "VL53L5CX not found at 0x29"
            )

        print(
            "PASS: deterministic I2C addresses"
        )

    def verify_identity(self):

        lsm_id = self.imu_bus.readfrom_mem(
            LSM_ADDR,
            LSM_WHO_AM_I,
            1
        )[0]

        mag_id = self.imu_bus.readfrom_mem(
            MAG_ADDR,
            MAG_WHO_AM_I,
            1
        )[0]

        print(
            "LSM6DSOX WHO_AM_I: 0x{:02X}".format(
                lsm_id
            )
        )

        print(
            "LIS3MDL WHO_AM_I:  0x{:02X}".format(
                mag_id
            )
        )

        if lsm_id != 0x6C:
            raise RuntimeError(
                "Unexpected LSM6DSOX WHO_AM_I"
            )

        if mag_id != 0x3D:
            raise RuntimeError(
                "Unexpected LIS3MDL WHO_AM_I"
            )

        print("PASS: sensor identities")

    # --------------------------------------------------------
    # Unit-specific LSM factory timing information
    # --------------------------------------------------------

    def read_factory_timing(self):

        self.freq_fine_raw = (
            self.imu_bus.readfrom_mem(
                LSM_ADDR,
                LSM_INTERNAL_FREQ_FINE,
                1
            )[0]
        )

        self.freq_fine = signed_u8(
            self.freq_fine_raw
        )

        self.freq_factor = (
            1.0 +
            0.0015 * self.freq_fine
        )

        if self.freq_factor <= 0.0:
            raise RuntimeError(
                "Invalid INTERNAL_FREQ_FINE factor"
            )

        # AN5272:
        # t_actual =
        #   1 /
        #   (40000 * (1 + 0.0015*FREQ_FINE))
        self.factory_tick_us = (
            1_000_000.0
            /
            (
                40_000.0 *
                self.freq_factor
            )
        )

        # AN5272:
        # ODR_actual =
        #   6667 * (1 + 0.0015*FREQ_FINE)
        #   / ODRcoeff
        self.factory_odr_hz = (
            6667.0 *
            self.freq_factor /
            LSM_ODR_COEFF
        )

        self.factory_period_us = (
            1_000_000.0 /
            self.factory_odr_hz
        )

        print()
        print(
            "Unit timing discovery:"
        )

        print(
            "  INTERNAL_FREQ_FINE raw: 0x{:02X}".format(
                self.freq_fine_raw
            )
        )

        print(
            "  INTERNAL_FREQ_FINE signed: {}".format(
                self.freq_fine
            )
        )

        print(
            "  Factory-estimated timestamp tick: "
            "{:.6f} us".format(
                self.factory_tick_us
            )
        )

        print(
            "  Factory-estimated real IMU ODR: "
            "{:.6f} Hz".format(
                self.factory_odr_hz
            )
        )

        print(
            "  Factory-estimated IMU period: "
            "{:.3f} us".format(
                self.factory_period_us
            )
        )

        print(
            "  Expected raw FIFO timestamp ticks/slot: "
            "{} (+/-{})".format(
                EXPECTED_SLOT_TICKS,
                TIMESTAMP_TICK_TOLERANCE
            )
        )

    # --------------------------------------------------------
    # LSM configuration
    # --------------------------------------------------------

    def configure_lsm(self):

        # Clean/bypass FIFO while configuring.
        self.imu_bus.writeto_mem(
            LSM_ADDR,
            LSM_FIFO_CTRL4,
            bytes([0x00])
        )

        self.imu_bus.writeto_mem(
            LSM_ADDR,
            LSM_FIFO_CTRL3,
            bytes([0x00])
        )

        # Accelerometer:
        # 104-Hz nominal, +/-4 g.
        self.imu_bus.writeto_mem(
            LSM_ADDR,
            LSM_CTRL1_XL,
            bytes([0x48])
        )

        # Gyroscope:
        # 104-Hz nominal, +/-250 dps.
        self.imu_bus.writeto_mem(
            LSM_ADDR,
            LSM_CTRL2_G,
            bytes([0x40])
        )

        # BDU=1, IF_INC=1.
        self.imu_bus.writeto_mem(
            LSM_ADDR,
            LSM_CTRL3_C,
            bytes([0x44])
        )

        # Enable 32-bit timestamp counter.
        self.imu_bus.writeto_mem(
            LSM_ADDR,
            LSM_CTRL10_C,
            bytes([0x20])
        )

        # Reset timestamp by writing AAh to TIMESTAMP2.
        self.imu_bus.writeto_mem(
            LSM_ADDR,
            LSM_TIMESTAMP2,
            bytes([0xAA])
        )

        time.sleep_ms(5)

        timestamp_after_reset = (
            self.read_lsm_timestamp_raw()
        )

        print(
            "LSM timestamp after reset/start: "
            "{} ticks".format(
                timestamp_after_reset
            )
        )

    # --------------------------------------------------------
    # Magnetometer configuration
    # --------------------------------------------------------

    def configure_mag(self):

        # CTRL_REG1:
        # UHP XY, 20-Hz ODR.
        self.imu_bus.writeto_mem(
            MAG_ADDR,
            MAG_CTRL_REG1,
            bytes([0x74])
        )

        # +/-4 gauss.
        self.imu_bus.writeto_mem(
            MAG_ADDR,
            MAG_CTRL_REG2,
            bytes([0x00])
        )

        # Continuous conversion.
        self.imu_bus.writeto_mem(
            MAG_ADDR,
            MAG_CTRL_REG3,
            bytes([0x00])
        )

        # UHP Z.
        self.imu_bus.writeto_mem(
            MAG_ADDR,
            MAG_CTRL_REG4,
            bytes([0x0C])
        )

        # BDU = 1.
        self.imu_bus.writeto_mem(
            MAG_ADDR,
            MAG_CTRL_REG5,
            bytes([0x40])
        )

        time.sleep_ms(100)

    # --------------------------------------------------------
    # ToF configuration
    # --------------------------------------------------------

    def configure_tof(self):

        print()
        print(
            "Initialising VL53L5CX on I2C1..."
        )

        print(
            "Firmware upload may take a few seconds."
        )

        self.tof = (
            breakout_vl53l5cx.VL53L5CX(
                self.tof_bus
            )
        )

        self.tof.set_resolution(
            breakout_vl53l5cx.RESOLUTION_8X8
        )

        self.tof.set_ranging_frequency_hz(
            15
        )

        self.tof.start_ranging()

        print(
            "VL53L5CX ranging started."
        )

    # --------------------------------------------------------
    # FIFO configuration
    # --------------------------------------------------------

    def enable_fifo(self):

        # Reset/clear FIFO.
        self.imu_bus.writeto_mem(
            LSM_ADDR,
            LSM_FIFO_CTRL4,
            bytes([0x00])
        )

        # Watermark 48 words; currently diagnostic only.
        self.imu_bus.writeto_mem(
            LSM_ADDR,
            LSM_FIFO_CTRL1,
            bytes([48])
        )

        # High watermark bit / compression /
        # stop-on-watermark disabled.
        self.imu_bus.writeto_mem(
            LSM_ADDR,
            LSM_FIFO_CTRL2,
            bytes([0x00])
        )

        # FIFO_CTRL3:
        # high nibble = gyro BDR 104-Hz nominal
        # low nibble  = accel BDR 104-Hz nominal
        fifo_ctrl3 = (
            (LSM_FIFO_BDR_CODE << 4) |
            LSM_FIFO_BDR_CODE
        )

        self.imu_bus.writeto_mem(
            LSM_ADDR,
            LSM_FIFO_CTRL3,
            bytes([fifo_ctrl3])
        )

        # FIFO_CTRL4:
        #
        # bits 7:6:
        #   timestamp decimation code 01
        # bits 5:4:
        #   temperature disabled
        # bit 3:
        #   reserved/0
        # bits 2:0:
        #   continuous mode 110
        fifo_ctrl4 = (
            (
                LSM_TIMESTAMP_DECIMATION_CODE
                << 6
            )
            |
            0x06
        )

        self.imu_bus.writeto_mem(
            LSM_ADDR,
            LSM_FIFO_CTRL4,
            bytes([fifo_ctrl4])
        )

        print(
            "LSM6DSOX FIFO enabled:"
        )

        print(
            "  accel BDR = 104-Hz nominal"
        )

        print(
            "  gyro BDR  = 104-Hz nominal"
        )

        print(
            "  timestamp = one per FIFO time slot"
        )

        print(
            "  continuous FIFO mode"
        )

    # --------------------------------------------------------
    # Low-level reads
    # --------------------------------------------------------

    def read_lsm_timestamp_raw(self):

        raw = self.imu_bus.readfrom_mem(
            LSM_ADDR,
            LSM_TIMESTAMP0,
            4
        )

        return struct.unpack(
            "<I",
            raw
        )[0]

    def fifo_status(self):

        raw = self.imu_bus.readfrom_mem(
            LSM_ADDR,
            LSM_FIFO_STATUS1,
            2
        )

        level = (
            raw[0] |
            (
                (raw[1] & 0x03)
                << 8
            )
        )

        return level, raw[1]

    def read_fifo_record(self):

        tag_raw = self.imu_bus.readfrom_mem(
            LSM_ADDR,
            LSM_FIFO_DATA_OUT_TAG,
            1
        )[0]

        raw = self.imu_bus.readfrom_mem(
            LSM_ADDR,
            LSM_FIFO_DATA_OUT_X_L,
            6
        )

        tag_sensor = (
            tag_raw >> 3
        ) & 0x1F

        tag_cnt = (
            tag_raw >> 1
        ) & 0x03

        tag_parity = (
            tag_raw & 0x01
        )

        return (
            tag_sensor,
            tag_cnt,
            tag_parity,
            raw
        )

    # --------------------------------------------------------
    # Robust LIS3MDL read
    # --------------------------------------------------------

    def read_mag_robust(self):

        retried = False

        for attempt in range(2):

            try:

                status = (
                    self.imu_bus.readfrom_mem(
                        MAG_ADDR,
                        MAG_STATUS_REG,
                        1
                    )[0]
                )

                # ZYXDA: complete fresh XYZ set.
                if not (status & 0x08):
                    return (
                        "not_ready",
                        None,
                        retried
                    )

                raw = (
                    self.imu_bus.readfrom_mem(
                        MAG_ADDR,
                        MAG_OUT_X_L | 0x80,
                        6
                    )
                )

                mx, my, mz = struct.unpack(
                    "<hhh",
                    raw
                )

                mx *= MAG_SCALE_UT
                my *= MAG_SCALE_UT
                mz *= MAG_SCALE_UT

                return (
                    "ok",
                    (mx, my, mz),
                    retried
                )

            except OSError:

                if attempt == 0:
                    retried = True
                    time.sleep_ms(2)

                else:
                    return (
                        "error",
                        None,
                        True
                    )

        return (
            "error",
            None,
            retried
        )

    # --------------------------------------------------------
    # Direct clock-correlation measurement
    # --------------------------------------------------------

    def read_clock_correlation(self):

        pico_a = self.pico_clock.now_us()

        lsm_raw = (
            self.read_lsm_timestamp_raw()
        )

        pico_b = self.pico_clock.now_us()

        pico_mid = (
            pico_a + pico_b
        ) // 2

        bracket_us = (
            pico_b - pico_a
        )

        return (
            pico_mid,
            lsm_raw,
            bracket_us
        )

    # --------------------------------------------------------
    # Runtime derived timing
    # --------------------------------------------------------

    def measured_tick_us(self):

        if not self.clock_model.is_ready():
            return None

        return (
            self.clock_model.slope_us_per_tick
        )

    def measured_odr_hz(self):

        tick_us = self.measured_tick_us()

        if (
            tick_us is None
            or
            tick_us <= 0.0
        ):
            return None

        slot_us = (
            EXPECTED_SLOT_TICKS *
            tick_us
        )

        if slot_us <= 0.0:
            return None

        return (
            1_000_000.0 /
            slot_us
        )

    def clock_ppm_vs_factory(self):

        tick_us = self.measured_tick_us()

        if (
            tick_us is None
            or
            self.factory_tick_us <= 0.0
        ):
            return None

        return (
            (
                tick_us -
                self.factory_tick_us
            )
            /
            self.factory_tick_us
        ) * 1_000_000.0

    # --------------------------------------------------------
    # Service magnetometer
    # --------------------------------------------------------

    def service_mag(self, stats):

        stats["mag_attempt"] += 1

        state, mag, retried = (
            self.read_mag_robust()
        )

        self.last_mag_bus_activity_us = (
            self.pico_clock.now_us()
        )

        if state == "ok":

            (
                self.latest_mx,
                self.latest_my,
                self.latest_mz
            ) = mag

            stats["mag_ok"] += 1

            if retried:
                stats["mag_retry_ok"] += 1

        elif state == "not_ready":

            stats["mag_not_ready"] += 1

        else:

            stats["mag_drop"] += 1

    # --------------------------------------------------------
    # Service ToF
    # --------------------------------------------------------

    def service_tof(self, stats):

        try:

            if not self.tof.data_ready():
                return

            frame_ready_seen_us = (
                self.pico_clock.now_us()
            )

            read_start_us = (
                frame_ready_seen_us
            )

            data = self.tof.get_data()

            distances = data.distance

            self.latest_tof_avg = (
                data.distance_avg
            )

            self.latest_tof_reflectance = (
                data.reflectance_avg
            )

            self.latest_tof_centre = (
                distances[27] +
                distances[28] +
                distances[35] +
                distances[36]
            ) // 4

            self.latest_tof_ready_pico_us = (
                frame_ready_seen_us
            )

            estimated_lsm = (
                self.clock_model.estimate_lsm_raw(
                    frame_ready_seen_us
                )
            )

            if estimated_lsm is not None:
                self.latest_tof_est_lsm_ticks = (
                    estimated_lsm
                )

            duration_us = (
                self.pico_clock.now_us() -
                read_start_us
            )

            stats["tof"] += 1

            stats["tof_read_total_us"] += (
                duration_us
            )

            if (
                duration_us >
                stats["tof_read_max_us"]
            ):
                stats["tof_read_max_us"] = (
                    duration_us
                )

        except OSError:

            stats["tof_err"] += 1

    # --------------------------------------------------------
    # Add a clock-correlation point
    # --------------------------------------------------------

    def service_clock_correlation(
        self,
        stats
    ):

        try:

            (
                pico_mid,
                lsm_raw,
                bracket_us
            ) = self.read_clock_correlation()

            added = self.clock_model.add_point(
                lsm_raw,
                pico_mid
            )

            if not added:
                stats["corr_err"] += 1
                return

            if (
                bracket_us >
                stats["corr_bracket_max_us"]
            ):
                stats[
                    "corr_bracket_max_us"
                ] = bracket_us

            tick_us = self.measured_tick_us()
            ppm = self.clock_ppm_vs_factory()

            if tick_us is None:
                tick_out = 0.0
            else:
                tick_out = tick_us

            if ppm is None:
                ppm_out = 0.0
            else:
                ppm_out = ppm

            rms = self.clock_model.rms_us

            if rms is None:
                rms_out = 0.0
            else:
                rms_out = rms

            print(
                "C,"
                "points={},"
                "pico_us={},"
                "lsm_ticks={},"
                "bracket_us={},"
                "model_tick_us={:.6f},"
                "factory_tick_us={:.6f},"
                "ppm_vs_factory={:.1f},"
                "fit_rms_us={:.2f}".format(
                    self.clock_model.point_count(),
                    pico_mid,
                    lsm_raw,
                    bracket_us,
                    tick_out,
                    self.factory_tick_us,
                    ppm_out,
                    rms_out
                )
            )

        except OSError:

            stats["corr_err"] += 1

    # --------------------------------------------------------
    # Service / drain the entire current FIFO backlog
    # --------------------------------------------------------

    def service_fifo(self, stats):

        try:

            level, status2 = (
                self.fifo_status()
            )

            if (
                level >
                stats["fifo_level_max"]
            ):
                stats["fifo_level_max"] = (
                    level
                )

            # FIFO_STATUS2:
            # bit 6 = FIFO_OVR_IA
            # bit 5 = FIFO_FULL_IA
            # bit 3 = FIFO_OVR_LATCHED
            if status2 & 0x48:
                stats["fifo_ovr"] = 1

            if status2 & 0x20:
                stats["fifo_full"] = 1

            records_to_read = level

            if (
                records_to_read >
                FIFO_MAX_RECORDS_PER_SERVICE
            ):
                records_to_read = (
                    FIFO_MAX_RECORDS_PER_SERVICE
                )

            for _ in range(
                records_to_read
            ):

                (
                    tag,
                    tag_cnt,
                    tag_parity,
                    raw
                ) = self.read_fifo_record()

                # ------------------------------------------
                # TAG_CNT slot integrity
                # ------------------------------------------

                if self.current_slot_cnt is None:

                    self.current_slot_cnt = (
                        tag_cnt
                    )

                    self.current_slot_mask = 0

                elif (
                    tag_cnt !=
                    self.current_slot_cnt
                ):

                    jump = (
                        tag_cnt -
                        self.current_slot_cnt
                    ) & 0x03

                    if jump != 1:
                        stats["slot_jump"] += 1

                    # Ignore the first possibly partial
                    # startup slot.
                    if self.slot_groups_seen > 0:

                        if (
                            self.current_slot_mask !=
                            SLOT_COMPLETE_MASK
                        ):
                            stats[
                                "slot_incomplete"
                            ] += 1

                    self.slot_groups_seen += 1

                    self.current_slot_cnt = (
                        tag_cnt
                    )

                    self.current_slot_mask = 0

                # ------------------------------------------
                # Gyroscope
                # ------------------------------------------

                if tag == LSM_TAG_GYRO:

                    x, y, z = struct.unpack(
                        "<hhh",
                        raw
                    )

                    self.latest_gx = (
                        x * GYRO_SCALE_DPS
                    )

                    self.latest_gy = (
                        y * GYRO_SCALE_DPS
                    )

                    self.latest_gz = (
                        z * GYRO_SCALE_DPS
                    )

                    stats["gy"] += 1

                    self.current_slot_mask |= (
                        SLOT_GYRO_BIT
                    )

                # ------------------------------------------
                # Accelerometer
                # ------------------------------------------

                elif tag == LSM_TAG_ACCEL:

                    x, y, z = struct.unpack(
                        "<hhh",
                        raw
                    )

                    self.latest_ax = (
                        x * ACCEL_SCALE
                    )

                    self.latest_ay = (
                        y * ACCEL_SCALE
                    )

                    self.latest_az = (
                        z * ACCEL_SCALE
                    )

                    stats["xl"] += 1

                    self.current_slot_mask |= (
                        SLOT_ACCEL_BIT
                    )

                # ------------------------------------------
                # Timestamp
                # ------------------------------------------

                elif tag == LSM_TAG_TIMESTAMP:

                    timestamp_ticks = (
                        raw[0] |
                        (raw[1] << 8) |
                        (raw[2] << 16) |
                        (raw[3] << 24)
                    )

                    bdr_shub = (
                        raw[4] & 0x0F
                    )

                    bdr_xl = (
                        raw[5] & 0x0F
                    )

                    bdr_gy = (
                        (raw[5] >> 4)
                        & 0x0F
                    )

                    if not (
                        bdr_shub == 0
                        and
                        bdr_xl ==
                        LSM_FIFO_BDR_CODE
                        and
                        bdr_gy ==
                        LSM_FIFO_BDR_CODE
                    ):
                        stats[
                            "ts_meta_bad"
                        ] += 1

                    stats["ts"] += 1

                    self.current_slot_mask |= (
                        SLOT_TIMESTAMP_BIT
                    )

                    if (
                        self.previous_fifo_timestamp
                        is not None
                    ):

                        delta_ticks = u32_diff(
                            timestamp_ticks,
                            self.previous_fifo_timestamp
                        )

                        if delta_ticks > 0:

                            stats[
                                "ts_delta_count"
                            ] += 1

                            stats[
                                "ts_delta_sum"
                            ] += delta_ticks

                            current_min = stats[
                                "ts_delta_min"
                            ]

                            if (
                                current_min is None
                                or
                                delta_ticks <
                                current_min
                            ):
                                stats[
                                    "ts_delta_min"
                                ] = delta_ticks

                            if (
                                delta_ticks >
                                stats[
                                    "ts_delta_max"
                                ]
                            ):
                                stats[
                                    "ts_delta_max"
                                ] = delta_ticks

                    self.previous_fifo_timestamp = (
                        timestamp_ticks
                    )

                # ------------------------------------------
                # Any unexpected tag
                # ------------------------------------------

                else:

                    stats["fifo_other"] += 1

        except OSError:

            stats["fifo_err"] += 1


# ============================================================
# Reporting helpers
# ============================================================

def timestamp_delta_summary(stats):

    count = stats["ts_delta_count"]

    if count <= 0:
        return 0, 0.0, 0

    minimum = stats["ts_delta_min"]

    if minimum is None:
        minimum = 0

    average = (
        stats["ts_delta_sum"] /
        count
    )

    maximum = stats["ts_delta_max"]

    return minimum, average, maximum


def print_status(
    stack,
    stats,
    self_test_done
):

    (
        ts_min,
        ts_avg,
        ts_max
    ) = timestamp_delta_summary(stats)

    if stats["tof"] > 0:

        tof_avg_read_us = (
            stats["tof_read_total_us"]
            /
            stats["tof"]
        )

    else:

        tof_avg_read_us = 0.0

    measured_tick = (
        stack.measured_tick_us()
    )

    measured_odr = (
        stack.measured_odr_hz()
    )

    ppm = (
        stack.clock_ppm_vs_factory()
    )

    if measured_tick is None:
        measured_tick = 0.0

    if measured_odr is None:
        measured_odr = 0.0

    if ppm is None:
        ppm = 0.0

    ready_text = (
        "YES"
        if self_test_done
        else "PENDING"
    )

    print(
        "S,"
        "xl={},"
        "gy={},"
        "ts={},"
        "fifo_err={},"
        "fifo_ovr={},"
        "fifo_full={},"
        "fifo_other={},"
        "fifo_level_max={},"
        "slot_jump={},"
        "slot_incomplete={},"
        "ts_meta_bad={},"
        "ts_dt_min={},"
        "ts_dt_avg={:.3f},"
        "ts_dt_max={},"
        "mag_ok={},"
        "mag_retry_ok={},"
        "mag_drop={},"
        "mag_not_ready={},"
        "tof={},"
        "tof_err={},"
        "tof_avg_read_us={:.0f},"
        "tof_max_read_us={},"
        "clock_points={},"
        "clock_tick_us={:.6f},"
        "measured_odr_hz={:.3f},"
        "clock_ppm={:.1f},"
        "ready={}".format(
            stats["xl"],
            stats["gy"],
            stats["ts"],
            stats["fifo_err"],
            stats["fifo_ovr"],
            stats["fifo_full"],
            stats["fifo_other"],
            stats["fifo_level_max"],
            stats["slot_jump"],
            stats["slot_incomplete"],
            stats["ts_meta_bad"],
            ts_min,
            ts_avg,
            ts_max,
            stats["mag_ok"],
            stats["mag_retry_ok"],
            stats["mag_drop"],
            stats["mag_not_ready"],
            stats["tof"],
            stats["tof_err"],
            tof_avg_read_us,
            stats["tof_read_max_us"],
            stack.clock_model.point_count(),
            measured_tick,
            measured_odr,
            ppm,
            ready_text
        )
    )


def print_values(stack):

    mag_total = math.sqrt(
        stack.latest_mx *
        stack.latest_mx
        +
        stack.latest_my *
        stack.latest_my
        +
        stack.latest_mz *
        stack.latest_mz
    )

    print(
        "V,"
        "acc=({:.3f},{:.3f},{:.3f})m/s2,"
        "gyro=({:.2f},{:.2f},{:.2f})dps,"
        "mag=({:.2f},{:.2f},{:.2f})uT,"
        "mag_total={:.2f}uT,"
        "tof_avg={}mm,"
        "tof_centre={}mm,"
        "tof_refl={}pct,"
        "tof_ready_pico_us={},"
        "tof_est_lsm_ticks={}".format(
            stack.latest_ax,
            stack.latest_ay,
            stack.latest_az,
            stack.latest_gx,
            stack.latest_gy,
            stack.latest_gz,
            stack.latest_mx,
            stack.latest_my,
            stack.latest_mz,
            mag_total,
            stack.latest_tof_avg,
            stack.latest_tof_centre,
            stack.latest_tof_reflectance,
            stack.latest_tof_ready_pico_us,
            stack.latest_tof_est_lsm_ticks
        )
    )


# ============================================================
# Startup reproducibility self-test
# ============================================================

def evaluate_self_test(
    stack,
    stats,
    elapsed_us
):

    print()
    print(
        "========================================"
    )

    print(
        " REPRODUCIBILITY SELF TEST"
    )

    print(
        "========================================"
    )

    duration_s = (
        elapsed_us / 1_000_000.0
    )

    # --------------------------------------------------------
    # FIFO structural integrity
    # --------------------------------------------------------

    fifo_integrity_ok = (
        stats["fifo_err"] == 0
        and
        stats["fifo_ovr"] == 0
        and
        stats["fifo_full"] == 0
        and
        stats["fifo_other"] == 0
        and
        stats["slot_jump"] == 0
        and
        stats["slot_incomplete"] == 0
        and
        stats["ts_meta_bad"] == 0
    )

    stream_alignment_ok = (
        abs_int(
            stats["xl"] -
            stats["gy"]
        ) <= 1
        and
        abs_int(
            stats["xl"] -
            stats["ts"]
        ) <= 1
    )

    (
        ts_min,
        ts_avg,
        ts_max
    ) = timestamp_delta_summary(stats)

    lower_ticks = (
        EXPECTED_SLOT_TICKS -
        TIMESTAMP_TICK_TOLERANCE
    )

    upper_ticks = (
        EXPECTED_SLOT_TICKS +
        TIMESTAMP_TICK_TOLERANCE
    )

    timestamp_consistency_ok = (
        stats["ts_delta_count"] > 100
        and
        ts_min >= lower_ticks
        and
        ts_max <= upper_ticks
    )

    # --------------------------------------------------------
    # Magnetometer communication
    # --------------------------------------------------------

    if stats["mag_attempt"] > 0:

        mag_success_ratio = (
            stats["mag_ok"] /
            stats["mag_attempt"]
        )

    else:

        mag_success_ratio = 0.0

    mag_ok = (
        mag_success_ratio >=
        SELF_TEST_MIN_MAG_SUCCESS_RATIO
    )

    # --------------------------------------------------------
    # ToF communication
    # --------------------------------------------------------

    if duration_s > 0.0:

        tof_fps = (
            stats["tof"] /
            duration_s
        )

    else:

        tof_fps = 0.0

    tof_ok = (
        tof_fps >=
        SELF_TEST_MIN_TOF_FPS
        and
        stats["tof_err"] == 0
    )

    # --------------------------------------------------------
    # Clock-model validity
    # --------------------------------------------------------

    clock_model_ok = (
        stack.clock_model.point_count() >= 3
        and
        stack.clock_model.is_ready()
        and
        stack.measured_tick_us() is not None
        and
        stack.measured_tick_us() > 0.0
    )

    measured_tick = (
        stack.measured_tick_us()
    )

    measured_odr = (
        stack.measured_odr_hz()
    )

    ppm = (
        stack.clock_ppm_vs_factory()
    )

    rms = stack.clock_model.rms_us

    if measured_tick is None:
        measured_tick = 0.0

    if measured_odr is None:
        measured_odr = 0.0

    if ppm is None:
        ppm = 0.0

    if rms is None:
        rms = 0.0

    trim_agreement_warning = (
        abs(ppm) >
        CLOCK_TRIM_WARNING_PPM
    )

    # --------------------------------------------------------
    # Overall result
    #
    # We deliberately DO NOT require any specific
    # INTERNAL_FREQ_FINE value or hard-coded real ODR.
    # --------------------------------------------------------

    overall_ok = (
        fifo_integrity_ok
        and
        stream_alignment_ok
        and
        timestamp_consistency_ok
        and
        mag_ok
        and
        tof_ok
        and
        clock_model_ok
    )

    # --------------------------------------------------------
    # Report
    # --------------------------------------------------------

    print(
        "Unit-specific timing:"
    )

    print(
        "  FREQ_FINE: {} "
        "(raw 0x{:02X})".format(
            stack.freq_fine,
            stack.freq_fine_raw
        )
    )

    print(
        "  Factory-estimated tick: "
        "{:.6f} us".format(
            stack.factory_tick_us
        )
    )

    print(
        "  Factory-estimated ODR: "
        "{:.6f} Hz".format(
            stack.factory_odr_hz
        )
    )

    print(
        "  Measured clock-model tick: "
        "{:.6f} us".format(
            measured_tick
        )
    )

    print(
        "  Measured clock-model ODR: "
        "{:.6f} Hz".format(
            measured_odr
        )
    )

    print(
        "  Factory vs measured: "
        "{:.1f} ppm".format(
            ppm
        )
    )

    print(
        "  Clock fit points: {}".format(
            stack.clock_model.point_count()
        )
    )

    print(
        "  Clock fit RMS: {:.2f} us".format(
            rms
        )
    )

    print()
    print(
        "FIFO / timestamp:"
    )

    print(
        "  XL/GY/TS records: "
        "{}/{}/{}".format(
            stats["xl"],
            stats["gy"],
            stats["ts"]
        )
    )

    print(
        "  timestamp ticks/slot: "
        "min={} avg={:.3f} max={}".format(
            ts_min,
            ts_avg,
            ts_max
        )
    )

    print(
        "  expected ticks/slot: "
        "{} +/-{}".format(
            EXPECTED_SLOT_TICKS,
            TIMESTAMP_TICK_TOLERANCE
        )
    )

    print(
        "  maximum FIFO backlog: "
        "{} words".format(
            stats["fifo_level_max"]
        )
    )

    print(
        "  FIFO structural integrity: "
        "{}".format(
            pass_fail(
                fifo_integrity_ok
            )
        )
    )

    print(
        "  stream alignment: "
        "{}".format(
            pass_fail(
                stream_alignment_ok
            )
        )
    )

    print(
        "  timestamp consistency: "
        "{}".format(
            pass_fail(
                timestamp_consistency_ok
            )
        )
    )

    print()
    print(
        "Magnetometer:"
    )

    print(
        "  successful/attempted: "
        "{}/{}".format(
            stats["mag_ok"],
            stats["mag_attempt"]
        )
    )

    print(
        "  success ratio: "
        "{:.2%}".format(
            mag_success_ratio
        )
    )

    print(
        "  retries recovered: {}".format(
            stats["mag_retry_ok"]
        )
    )

    print(
        "  dropped: {}".format(
            stats["mag_drop"]
        )
    )

    print(
        "  magnetometer acquisition: "
        "{}".format(
            pass_fail(mag_ok)
        )
    )

    print()
    print(
        "VL53L5CX:"
    )

    print(
        "  frames: {}".format(
            stats["tof"]
        )
    )

    print(
        "  observed frame rate: "
        "{:.3f} fps".format(
            tof_fps
        )
    )

    print(
        "  errors: {}".format(
            stats["tof_err"]
        )
    )

    print(
        "  maximum get_data() time: "
        "{} us".format(
            stats["tof_read_max_us"]
        )
    )

    print(
        "  ToF acquisition: "
        "{}".format(
            pass_fail(tof_ok)
        )
    )

    print()
    print(
        "Clock model:"
    )

    print(
        "  Pico <-> LSM model: "
        "{}".format(
            pass_fail(clock_model_ok)
        )
    )

    if trim_agreement_warning:

        print(
            "  WARNING: measured tick differs "
            "from factory trim by more than "
            "{:.0f} ppm.".format(
                CLOCK_TRIM_WARNING_PPM
            )
        )

        print(
            "  Runtime synchronization will use "
            "the measured clock model."
        )

    else:

        print(
            "  Factory trim agreement: PASS"
        )

    print()
    print(
        "========================================"
    )

    if overall_ok:

        print(
            " SYSTEM READY: PASS"
        )

        print(
            " No hard-coded per-unit ODR is in use."
        )

    else:

        print(
            " SYSTEM READY: FAIL"
        )

        print(
            " Review failed subsystem(s) above."
        )

    print(
        "========================================"
    )

    print()

    return overall_ok


# ============================================================
# Main
# ============================================================

def main():

    print()
    print(
        "Reproducible ToF + 9DoF sensor stack"
    )

    print(
        "Diagnostic firmware baseline v0.5"
    )

    print()

    stack = SensorStack()

    # --------------------------------------------------------
    # Deterministic boot validation
    # --------------------------------------------------------

    stack.verify_bus_topology()
    stack.verify_identity()
    stack.read_factory_timing()

    stack.configure_lsm()
    stack.configure_mag()
    stack.configure_tof()
    stack.enable_fifo()

    print()
    print(
        "Runtime validation started."
    )

    print(
        "Self-test will evaluate after "
        "{} seconds.".format(
            SELF_TEST_DURATION_US //
            1_000_000
        )
    )

    print(
        "The firmware will continue running "
        "after PASS/FAIL."
    )

    print()

    # --------------------------------------------------------
    # Scheduler setup
    # --------------------------------------------------------

    start_us = (
        stack.pico_clock.now_us()
    )

    now_us = start_us

    next_mag_us = (
        now_us +
        MAG_INITIAL_OFFSET_US
    )

    next_fifo_service_us = now_us

    next_tof_poll_us = now_us

    next_correlation_us = (
        now_us +
        CLOCK_CORRELATION_INITIAL_US
    )

    next_status_us = (
        now_us +
        STATUS_PERIOD_US
    )

    next_values_us = (
        now_us +
        VALUES_PERIOD_US
    )

    interval_stats = new_interval_stats()
    self_test_stats = new_interval_stats()

    self_test_done = False
    self_test_passed = False

    # --------------------------------------------------------
    # Runtime loop
    # --------------------------------------------------------

    while True:

        now_us = (
            stack.pico_clock.now_us()
        )

        # ====================================================
        # 1. Magnetometer when due
        # ====================================================

        if now_us >= next_mag_us:

            before = new_interval_stats()

            stack.service_mag(before)

            accumulate(
                interval_stats,
                before
            )

            if not self_test_done:
                accumulate(
                    self_test_stats,
                    before
                )

            next_mag_us += MAG_PERIOD_US

            now_after = (
                stack.pico_clock.now_us()
            )

            if (
                now_after -
                next_mag_us >
                MAG_PERIOD_US
            ):
                next_mag_us = (
                    now_after +
                    MAG_PERIOD_US
                )

        # ====================================================
        # 2. ToF
        #
        # Do not START the relatively long frame transfer
        # immediately before a magnetometer transaction.
        # ====================================================

        now_us = (
            stack.pico_clock.now_us()
        )

        if now_us >= next_tof_poll_us:

            next_tof_poll_us = (
                now_us +
                TOF_POLL_US
            )

            time_to_mag = (
                next_mag_us -
                now_us
            )

            if (
                time_to_mag >
                TOF_MAG_GUARD_US
            ):

                before = new_interval_stats()

                stack.service_tof(before)

                accumulate(
                    interval_stats,
                    before
                )

                if not self_test_done:
                    accumulate(
                        self_test_stats,
                        before
                    )

        # ====================================================
        # 3. If ToF crossed a mag deadline, service mag first
        # ====================================================

        now_us = (
            stack.pico_clock.now_us()
        )

        if now_us >= next_mag_us:

            before = new_interval_stats()

            stack.service_mag(before)

            accumulate(
                interval_stats,
                before
            )

            if not self_test_done:
                accumulate(
                    self_test_stats,
                    before
                )

            next_mag_us += MAG_PERIOD_US

        # ====================================================
        # 4. Pico <-> LSM clock correlation
        # ====================================================

        now_us = (
            stack.pico_clock.now_us()
        )

        if now_us >= next_correlation_us:

            time_to_mag = (
                next_mag_us -
                now_us
            )

            time_since_mag = (
                now_us -
                stack.last_mag_bus_activity_us
            )

            mag_window_clear = (
                time_to_mag >
                CLOCK_CORRELATION_MAG_GUARD_US
                and
                time_since_mag >
                CLOCK_CORRELATION_MAG_GUARD_US
            )

            if mag_window_clear:

                before = new_interval_stats()

                stack.service_clock_correlation(
                    before
                )

                accumulate(
                    interval_stats,
                    before
                )

                if not self_test_done:
                    accumulate(
                        self_test_stats,
                        before
                    )

                next_correlation_us += (
                    CLOCK_CORRELATION_PERIOD_US
                )

        # ====================================================
        # 5. FIFO service
        #
        # FIFO scheduling is deliberately decoupled from the
        # actual IMU ODR. We read the level and drain whatever
        # is available.
        # ====================================================

        now_us = (
            stack.pico_clock.now_us()
        )

        if (
            now_us >=
            next_fifo_service_us
        ):

            time_to_mag = (
                next_mag_us -
                now_us
            )

            time_since_mag = (
                now_us -
                stack.last_mag_bus_activity_us
            )

            mag_window_clear = (
                time_to_mag >
                FIFO_MAG_GUARD_US
                and
                time_since_mag >
                FIFO_MAG_GUARD_US
            )

            if mag_window_clear:

                before = new_interval_stats()

                stack.service_fifo(before)

                accumulate(
                    interval_stats,
                    before
                )

                if not self_test_done:
                    accumulate(
                        self_test_stats,
                        before
                    )

                next_fifo_service_us += (
                    FIFO_SERVICE_PERIOD_US
                )

                now_after = (
                    stack.pico_clock.now_us()
                )

                if (
                    now_after -
                    next_fifo_service_us >
                    FIFO_SERVICE_PERIOD_US
                ):
                    next_fifo_service_us = (
                        now_after +
                        FIFO_SERVICE_PERIOD_US
                    )

        # ====================================================
        # 6. One-second status
        # ====================================================

        now_us = (
            stack.pico_clock.now_us()
        )

        if now_us >= next_status_us:

            print_status(
                stack,
                interval_stats,
                self_test_done
            )

            interval_stats = (
                new_interval_stats()
            )

            next_status_us += (
                STATUS_PERIOD_US
            )

        # ====================================================
        # 7. Startup self-certification
        # ====================================================

        now_us = (
            stack.pico_clock.now_us()
        )

        if (
            not self_test_done
            and
            (
                now_us -
                start_us
            ) >= SELF_TEST_DURATION_US
        ):

            self_test_passed = (
                evaluate_self_test(
                    stack,
                    self_test_stats,
                    now_us - start_us
                )
            )

            self_test_done = True

        # ====================================================
        # 8. Occasional physical values
        # ====================================================

        if now_us >= next_values_us:

            print_values(stack)

            next_values_us += (
                VALUES_PERIOD_US
            )

        # ====================================================
        # 9. Avoid a flat-out Python busy loop
        # ====================================================

        time.sleep_ms(1)


# ============================================================
# Entry point
# ============================================================

main()
