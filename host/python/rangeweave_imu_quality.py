"""Quality checks for Rangeweave LSM6DSOX motion captures.

The relative-rotation estimator deliberately focuses on orientation estimation. This
module adds capture-quality checks that are useful before a motion is admitted to
boresight calibration, starting with configured gyro full-scale utilisation.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Sequence

import rangeweave_imu_relative as imu


Vector3 = imu.Vector3
AXES = ("X", "Y", "Z")
DEFAULT_GYRO_WARNING_FRACTION = 0.80
DEFAULT_GYRO_REJECT_FRACTION = 0.90


class ImuQualityError(ValueError):
    """Raised when IMU quality metadata or samples cannot be interpreted."""


def gyro_full_scale_dps(ctrl2_g: int) -> float:
    """Return the configured LSM6DSOX gyro full-scale magnitude in deg/s."""

    value = int(ctrl2_g) & 0xFF
    fs_125 = bool(value & 0x02)
    fs = (value >> 2) & 0x03
    if fs_125 and fs == 0:
        return 125.0
    return {
        0b00: 250.0,
        0b01: 500.0,
        0b10: 1000.0,
        0b11: 2000.0,
    }[fs]


@dataclass(frozen=True)
class GyroRangeUsage:
    full_scale_dps: float
    peak_body_dps: Vector3
    peak_fraction: Vector3
    max_axis: str
    max_peak_dps: float
    max_fraction: float
    warning_fraction: float
    reject_fraction: float

    @property
    def warning(self) -> bool:
        return self.max_fraction >= self.warning_fraction

    @property
    def rejected(self) -> bool:
        return self.max_fraction >= self.reject_fraction


def analyse_gyro_range(
    samples: Sequence[object],
    *,
    ctrl2_g: int,
    warning_fraction: float = DEFAULT_GYRO_WARNING_FRACTION,
    reject_fraction: float = DEFAULT_GYRO_REJECT_FRACTION,
) -> GyroRangeUsage:
    """Measure peak body-axis gyro utilisation against the configured full scale.

    Peaks are taken from the recorded raw gyro samples after applying the configured
    sensitivity and the physically validated imu_sensor -> device_body mapping. The
    check is intentionally conservative: data at or near full scale may already have
    lost unobserved between-sample peak motion, so boresight calibration should reject
    the capture before relying on integrated orientation.
    """

    samples = tuple(samples)
    if not samples:
        raise ImuQualityError("at least one IMU sample is required")
    warning = float(warning_fraction)
    reject = float(reject_fraction)
    if not (0.0 < warning < reject <= 1.0):
        raise ImuQualityError(
            "gyro range thresholds must satisfy 0 < warning < reject <= 1"
        )

    full_scale = gyro_full_scale_dps(ctrl2_g)
    scale = imu.gyro_dps_per_lsb(ctrl2_g)
    peaks = [0.0, 0.0, 0.0]

    for sample in samples:
        sensor_rate = (
            float(getattr(sample, "gyro_x")) * scale,
            float(getattr(sample, "gyro_y")) * scale,
            float(getattr(sample, "gyro_z")) * scale,
        )
        body_rate = imu.imu_vector_to_body(sensor_rate)
        for axis in range(3):
            peaks[axis] = max(peaks[axis], abs(body_rate[axis]))

    fractions = tuple(value / full_scale for value in peaks)
    max_index = max(range(3), key=lambda index: fractions[index])
    return GyroRangeUsage(
        full_scale_dps=full_scale,
        peak_body_dps=tuple(peaks),  # type: ignore[arg-type]
        peak_fraction=fractions,  # type: ignore[arg-type]
        max_axis=AXES[max_index],
        max_peak_dps=peaks[max_index],
        max_fraction=fractions[max_index],
        warning_fraction=warning,
        reject_fraction=reject,
    )
