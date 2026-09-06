"""LIS3MDL configuration decoding and raw magnetic-data diagnostics.

This module deliberately stops at the ``mag_sensor`` frame.  It does not guess the
assembly-specific ``mag_sensor -> device_body`` rotation and it does not provide a
heading/yaw observation.  Those require physical mapping and calibration evidence.

Protocol v0.1 preserves raw signed 16-bit LIS3MDL samples plus a CTRL_REG1..5
snapshot in STREAM_INFO.  This module decodes that snapshot so host tools can
convert raw counts to engineering units without assuming the reference producer's
current full-scale setting.
"""

from __future__ import annotations

from dataclasses import dataclass
import math
import statistics
from typing import Iterable, Sequence

import rangeweave_protocol as rw


class MagnetometerError(ValueError):
    """Raised when magnetometer metadata or samples are unusable."""


_FS_TABLE = {
    0: (4.0, 6842.0),
    1: (8.0, 3421.0),
    2: (12.0, 2281.0),
    3: (16.0, 1711.0),
}

_ODR_TABLE_HZ = {
    0: 0.625,
    1: 1.25,
    2: 2.5,
    3: 5.0,
    4: 10.0,
    5: 20.0,
    6: 40.0,
    7: 80.0,
}

_FAST_ODR_HZ_BY_MODE = {
    0: 1000.0,  # low power
    1: 560.0,   # medium performance
    2: 300.0,   # high performance
    3: 155.0,   # ultra-high performance
}

_MODE_NAMES = {
    0: "low-power",
    1: "medium-performance",
    2: "high-performance",
    3: "ultra-high-performance",
}

_MEASUREMENT_MODE_NAMES = {
    0: "continuous-conversion",
    1: "single-conversion",
    2: "power-down",
    3: "power-down",
}


@dataclass(frozen=True)
class Lis3mdlConfig:
    ctrl_regs_1_to_5: bytes
    full_scale_gauss: float
    sensitivity_lsb_per_gauss: float
    output_data_rate_hz: float
    xy_performance_mode: str
    z_performance_mode: str
    measurement_mode: str
    low_power: bool
    block_data_update: bool

    @property
    def full_scale_microtesla(self) -> float:
        return self.full_scale_gauss * 100.0


@dataclass(frozen=True)
class MagneticSample:
    time_us: float
    read_duration_us: int
    raw_x: int
    raw_y: int
    raw_z: int
    x_ut: float
    y_ut: float
    z_ut: float
    norm_ut: float
    status_reg: int
    read_flags: int

    @property
    def retry_used(self) -> bool:
        return bool(self.read_flags & rw.MAG_FLAG_RETRY_USED)

    @property
    def data_ready(self) -> bool:
        return bool(self.status_reg & 0x08)

    @property
    def overrun(self) -> bool:
        return bool(self.status_reg & 0x80)


@dataclass(frozen=True)
class MagneticSummary:
    sample_count: int
    span_s: float
    observed_rate_hz: float | None
    median_interval_ms: float | None
    p95_interval_ms: float | None
    max_interval_ms: float | None
    retry_count: int
    not_ready_count: int
    overrun_count: int
    x_range_ut: tuple[float, float]
    y_range_ut: tuple[float, float]
    z_range_ut: tuple[float, float]
    axis_span_ut: tuple[float, float, float]
    max_axis_utilisation_fraction: tuple[float, float, float]
    norm_median_ut: float
    norm_p05_ut: float
    norm_p95_ut: float
    norm_min_ut: float
    norm_max_ut: float


def decode_lis3mdl_config(ctrl_regs_1_to_5: bytes | Sequence[int]) -> Lis3mdlConfig:
    raw = bytes(ctrl_regs_1_to_5)
    if len(raw) != 5:
        raise MagnetometerError(
            f"LIS3MDL CTRL_REG1..5 snapshot must contain 5 bytes, got {len(raw)}"
        )

    ctrl1, ctrl2, ctrl3, ctrl4, ctrl5 = raw
    fs_code = (ctrl2 >> 5) & 0x03
    full_scale_gauss, sensitivity = _FS_TABLE[fs_code]

    xy_mode_code = (ctrl1 >> 5) & 0x03
    z_mode_code = (ctrl4 >> 2) & 0x03
    low_power = bool(ctrl3 & 0x20)
    if low_power:
        odr_hz = 0.625
    elif ctrl1 & 0x02:
        odr_hz = _FAST_ODR_HZ_BY_MODE[xy_mode_code]
    else:
        odr_hz = _ODR_TABLE_HZ[(ctrl1 >> 2) & 0x07]

    measurement_mode = _MEASUREMENT_MODE_NAMES[ctrl3 & 0x03]
    return Lis3mdlConfig(
        ctrl_regs_1_to_5=raw,
        full_scale_gauss=full_scale_gauss,
        sensitivity_lsb_per_gauss=sensitivity,
        output_data_rate_hz=odr_hz,
        xy_performance_mode=_MODE_NAMES[xy_mode_code],
        z_performance_mode=_MODE_NAMES[z_mode_code],
        measurement_mode=measurement_mode,
        low_power=low_power,
        block_data_update=bool(ctrl5 & 0x40),
    )


def config_from_stream_info(info: rw.StreamInfo | None) -> Lis3mdlConfig:
    if info is None:
        raise MagnetometerError("capture is missing STREAM_INFO metadata")
    raw = info.first_value(rw.INFO_MAG_CTRL_REGS_1_TO_5)
    if raw is None:
        raise MagnetometerError("STREAM_INFO is missing LIS3MDL CTRL_REG1..5 metadata")
    return decode_lis3mdl_config(raw)


def raw_to_microtesla(raw_value: int, config: Lis3mdlConfig) -> float:
    # LIS3MDL sensitivity is specified in LSB/gauss; 1 gauss = 100 microtesla.
    return float(raw_value) * (100.0 / config.sensitivity_lsb_per_gauss)


def convert_sample(sample: rw.MagSample, config: Lis3mdlConfig) -> MagneticSample:
    if sample.mcu_after_us < sample.mcu_before_us:
        raise MagnetometerError("MAG read bracket has negative duration")
    x_ut = raw_to_microtesla(sample.mag_x, config)
    y_ut = raw_to_microtesla(sample.mag_y, config)
    z_ut = raw_to_microtesla(sample.mag_z, config)
    return MagneticSample(
        time_us=(float(sample.mcu_before_us) + float(sample.mcu_after_us)) * 0.5,
        read_duration_us=int(sample.mcu_after_us - sample.mcu_before_us),
        raw_x=int(sample.mag_x),
        raw_y=int(sample.mag_y),
        raw_z=int(sample.mag_z),
        x_ut=x_ut,
        y_ut=y_ut,
        z_ut=z_ut,
        norm_ut=math.sqrt(x_ut * x_ut + y_ut * y_ut + z_ut * z_ut),
        status_reg=int(sample.status_reg),
        read_flags=int(sample.read_flags),
    )


def convert_samples(
    samples: Iterable[rw.MagSample],
    config: Lis3mdlConfig,
) -> tuple[MagneticSample, ...]:
    return tuple(convert_sample(sample, config) for sample in samples)


def _percentile(values: Sequence[float], probability: float) -> float:
    if not values:
        raise MagnetometerError("cannot calculate percentile of empty values")
    ordered = sorted(float(value) for value in values)
    if len(ordered) == 1:
        return ordered[0]
    position = float(probability) * (len(ordered) - 1)
    low = int(math.floor(position))
    high = int(math.ceil(position))
    if low == high:
        return ordered[low]
    fraction = position - low
    return ordered[low] * (1.0 - fraction) + ordered[high] * fraction


def summarise(
    samples: Sequence[MagneticSample],
    config: Lis3mdlConfig,
) -> MagneticSummary:
    if not samples:
        raise MagnetometerError("capture contains no MAG samples")

    times = [float(sample.time_us) for sample in samples]
    intervals_us = [b - a for a, b in zip(times, times[1:]) if b >= a]
    span_us = max(0.0, times[-1] - times[0])
    observed_rate = None
    if len(times) >= 2 and span_us > 0.0:
        observed_rate = (len(times) - 1) * 1_000_000.0 / span_us

    xs = [sample.x_ut for sample in samples]
    ys = [sample.y_ut for sample in samples]
    zs = [sample.z_ut for sample in samples]
    norms = [sample.norm_ut for sample in samples]
    ranges = (
        (min(xs), max(xs)),
        (min(ys), max(ys)),
        (min(zs), max(zs)),
    )
    spans = tuple(high - low for low, high in ranges)
    fs_ut = config.full_scale_microtesla
    utilisations = (
        max(abs(value) for value in xs) / fs_ut,
        max(abs(value) for value in ys) / fs_ut,
        max(abs(value) for value in zs) / fs_ut,
    )

    interval_ms = [value / 1000.0 for value in intervals_us]
    return MagneticSummary(
        sample_count=len(samples),
        span_s=span_us / 1_000_000.0,
        observed_rate_hz=observed_rate,
        median_interval_ms=(statistics.median(interval_ms) if interval_ms else None),
        p95_interval_ms=(_percentile(interval_ms, 0.95) if interval_ms else None),
        max_interval_ms=(max(interval_ms) if interval_ms else None),
        retry_count=sum(1 for sample in samples if sample.retry_used),
        not_ready_count=sum(1 for sample in samples if not sample.data_ready),
        overrun_count=sum(1 for sample in samples if sample.overrun),
        x_range_ut=ranges[0],
        y_range_ut=ranges[1],
        z_range_ut=ranges[2],
        axis_span_ut=spans,
        max_axis_utilisation_fraction=utilisations,
        norm_median_ut=statistics.median(norms),
        norm_p05_ut=_percentile(norms, 0.05),
        norm_p95_ut=_percentile(norms, 0.95),
        norm_min_ut=min(norms),
        norm_max_ut=max(norms),
    )
