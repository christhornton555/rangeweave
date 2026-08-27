"""Short-baseline relative body rotation from Rangeweave LSM6DSOX captures.

This module is intentionally narrower than a full attitude filter. It estimates the
relative rotation between two stationary poses in a hold-move-hold capture using:
- protocol CLOCK_SYNC observations for the LSM tick -> seconds scale;
- an experimentally validated imu_sensor -> device_body rotation for the reference rig;
- data-selected stationary windows near each end of the capture;
- endpoint gyro-bias estimates with linear interpolation during integration;
- gyro integration in device_body coordinates; and
- stationary accelerometer vectors only as an independent gravity-direction check.

It does not create a world frame, fuse magnetometer data, or define a persistent
orientation state.
"""

from __future__ import annotations

from bisect import bisect_left
from dataclasses import dataclass
import math
import statistics
from typing import Sequence

import rangeweave_extrinsics as ext


Vector3 = ext.Vector3
Matrix3 = ext.Matrix3

# Physical validation on the current rigid Rangeweave breadboard prototype established:
# device_body +X -> imu_sensor -X
# device_body +Y -> imu_sensor -Z
# device_body +Z -> imu_sensor -Y
#
# Therefore v_body = R_body_from_imu * v_imu:
REFERENCE_ROTATION_BODY_FROM_IMU: Matrix3 = (
    (-1.0, 0.0, 0.0),
    (0.0, 0.0, -1.0),
    (0.0, -1.0, 0.0),
)
REFERENCE_IMU_MAPPING_ROLE = "physically-validated-reference-rig"


class RelativeRotationError(ValueError):
    """Raised when a capture cannot support relative-body rotation estimation."""


@dataclass(frozen=True)
class ClockFit:
    observation_count: int
    reference_tick: float
    reference_us: float
    us_per_tick: float
    rms_residual_us: float
    max_abs_residual_us: float
    max_half_bracket_us: float

    def tick_to_us(self, tick: int | float) -> float:
        return self.reference_us + self.us_per_tick * (float(tick) - self.reference_tick)


@dataclass(frozen=True)
class StationaryWindow:
    start_index: int
    end_index: int  # exclusive
    start_time_s: float
    end_time_s: float
    midpoint_time_s: float
    gyro_median_body_dps: Vector3
    gyro_mad_body_dps: Vector3
    gyro_spread_dps: float
    accel_median_body_g: Vector3
    accel_median_norm_g: float
    accel_median_deviation_g: float
    score: float


@dataclass(frozen=True)
class RelativeRotationEstimate:
    reference_from_body: Matrix3
    rotation_angle_deg: float
    rotation_xyz_deg: Vector3
    rotation_axis_reference: Vector3
    integrated_duration_s: float
    gravity_direction_change_deg: float
    gravity_closure_error_deg: float
    initial_bias_body_dps: Vector3
    final_bias_body_dps: Vector3
    initial_stationary: StationaryWindow
    final_stationary: StationaryWindow
    clock_fit: ClockFit
    sample_count: int
    imu_mapping_role: str


def accel_g_per_lsb(ctrl1_xl: int) -> float:
    fs = (int(ctrl1_xl) >> 2) & 0x03
    mg_per_lsb = {
        0b00: 0.061,
        0b01: 0.488,
        0b10: 0.122,
        0b11: 0.244,
    }[fs]
    return mg_per_lsb / 1000.0


def gyro_dps_per_lsb(ctrl2_g: int) -> float:
    value = int(ctrl2_g)
    fs_125 = bool(value & 0x02)
    fs = (value >> 2) & 0x03
    if fs_125 and fs == 0:
        mdps_per_lsb = 4.375
    else:
        mdps_per_lsb = {
            0b00: 8.75,
            0b01: 17.50,
            0b10: 35.0,
            0b11: 70.0,
        }[fs]
    return mdps_per_lsb / 1000.0


def imu_vector_to_body(vector: Sequence[float]) -> Vector3:
    values = tuple(float(value) for value in vector)
    if len(values) != 3:
        raise RelativeRotationError("IMU vector must contain exactly three values")
    return ext.matrix_vector(REFERENCE_ROTATION_BODY_FROM_IMU, values)  # type: ignore[arg-type]


def fit_lsm_clock(clock_syncs: Sequence[object]) -> ClockFit:
    observations = []
    half_brackets = []
    for item in clock_syncs:
        before = float(getattr(item, "mcu_before_us"))
        after = float(getattr(item, "mcu_after_us"))
        tick = float(getattr(item, "lsm_tick"))
        if not (math.isfinite(before) and math.isfinite(after) and math.isfinite(tick)):
            continue
        if after < before:
            continue
        observations.append((tick, (before + after) / 2.0))
        half_brackets.append((after - before) / 2.0)

    if len(observations) < 2:
        raise RelativeRotationError("at least two valid CLOCK_SYNC observations are required")

    observations.sort()
    deduped = []
    for tick, midpoint in observations:
        if deduped and tick == deduped[-1][0]:
            continue
        deduped.append((tick, midpoint))
    if len(deduped) < 2:
        raise RelativeRotationError("CLOCK_SYNC observations do not span distinct LSM ticks")

    xbar = sum(item[0] for item in deduped) / len(deduped)
    ybar = sum(item[1] for item in deduped) / len(deduped)
    denominator = sum((tick - xbar) ** 2 for tick, _ in deduped)
    if denominator <= 0.0:
        raise RelativeRotationError("CLOCK_SYNC regression has zero tick span")
    slope = (
        sum((tick - xbar) * (midpoint - ybar) for tick, midpoint in deduped)
        / denominator
    )
    if not math.isfinite(slope) or slope <= 0.0:
        raise RelativeRotationError("CLOCK_SYNC regression produced a non-positive tick scale")

    residuals = [
        midpoint - (ybar + slope * (tick - xbar))
        for tick, midpoint in deduped
    ]
    rms = math.sqrt(sum(value * value for value in residuals) / len(residuals))
    return ClockFit(
        observation_count=len(deduped),
        reference_tick=xbar,
        reference_us=ybar,
        us_per_tick=slope,
        rms_residual_us=rms,
        max_abs_residual_us=max(abs(value) for value in residuals),
        max_half_bracket_us=max(half_brackets) if half_brackets else 0.0,
    )


def _median(values: Sequence[float]) -> float:
    return float(statistics.median(values))


def _vector_median(vectors: Sequence[Vector3]) -> Vector3:
    return tuple(_median([vector[index] for vector in vectors]) for index in range(3))  # type: ignore[return-value]


def _norm(vector: Sequence[float]) -> float:
    return math.sqrt(sum(float(value) * float(value) for value in vector))


def _normalise(vector: Sequence[float]) -> Vector3:
    length = _norm(vector)
    if length <= 0.0:
        raise RelativeRotationError("cannot normalise zero vector")
    return tuple(float(value) / length for value in vector)  # type: ignore[return-value]


def _angle_deg(a: Sequence[float], b: Sequence[float]) -> float:
    na = _normalise(a)
    nb = _normalise(b)
    dot = max(-1.0, min(1.0, sum(na[i] * nb[i] for i in range(3))))
    return math.degrees(math.acos(dot))


def _window_stats(
    start: int,
    end: int,
    times_s: Sequence[float],
    gyros_body_dps: Sequence[Vector3],
    accels_body_g: Sequence[Vector3],
) -> StationaryWindow:
    gyro_slice = gyros_body_dps[start:end]
    accel_slice = accels_body_g[start:end]
    gyro_median = _vector_median(gyro_slice)
    gyro_mad = tuple(
        _median([abs(vector[index] - gyro_median[index]) for vector in gyro_slice])
        for index in range(3)
    )
    gyro_spread = _norm(gyro_mad)

    accel_median = _vector_median(accel_slice)
    accel_deviation = _median(
        [
            _norm(tuple(vector[index] - accel_median[index] for index in range(3)))
            for vector in accel_slice
        ]
    )
    accel_norm = _norm(accel_median)

    score = gyro_spread / 0.50 + accel_deviation / 0.010
    return StationaryWindow(
        start_index=start,
        end_index=end,
        start_time_s=times_s[start],
        end_time_s=times_s[end - 1],
        midpoint_time_s=(times_s[start] + times_s[end - 1]) / 2.0,
        gyro_median_body_dps=gyro_median,
        gyro_mad_body_dps=gyro_mad,  # type: ignore[arg-type]
        gyro_spread_dps=gyro_spread,
        accel_median_body_g=accel_median,
        accel_median_norm_g=accel_norm,
        accel_median_deviation_g=accel_deviation,
        score=score,
    )


def _select_stationary_window(
    times_s: Sequence[float],
    gyros_body_dps: Sequence[Vector3],
    accels_body_g: Sequence[Vector3],
    *,
    region_start_s: float,
    region_end_s: float,
    window_seconds: float,
) -> StationaryWindow:
    best = None
    count = len(times_s)
    for start in range(count):
        if times_s[start] < region_start_s:
            continue
        if times_s[start] > region_end_s:
            break
        target = times_s[start] + window_seconds
        if target > region_end_s:
            continue
        end = bisect_left(times_s, target, lo=start + 1)
        if end < count:
            end += 1
        if end - start < 10:
            continue
        if times_s[end - 1] > region_end_s:
            continue
        candidate = _window_stats(start, end, times_s, gyros_body_dps, accels_body_g)
        if not (0.75 <= candidate.accel_median_norm_g <= 1.25):
            continue
        if best is None or candidate.score < best.score:
            best = candidate

    if best is None:
        raise RelativeRotationError("could not find a candidate stationary window")
    if best.gyro_spread_dps > 1.5:
        raise RelativeRotationError(
            "best stationary window has too much gyro variation "
            f"({best.gyro_spread_dps:.2f} deg/s robust spread)"
        )
    if best.accel_median_deviation_g > 0.04:
        raise RelativeRotationError(
            "best stationary window has too much accelerometer variation "
            f"({best.accel_median_deviation_g:.3f} g median deviation)"
        )
    return best


def _skew_exp(rotation_vector_rad: Vector3) -> Matrix3:
    angle = _norm(rotation_vector_rad)
    if angle < 1.0e-12:
        x, y, z = rotation_vector_rad
        return (
            (1.0, -z, y),
            (z, 1.0, -x),
            (-y, x, 1.0),
        )
    axis = tuple(value / angle for value in rotation_vector_rad)
    x, y, z = axis
    c = math.cos(angle)
    s = math.sin(angle)
    one_c = 1.0 - c
    return (
        (c + x * x * one_c, x * y * one_c - z * s, x * z * one_c + y * s),
        (y * x * one_c + z * s, c + y * y * one_c, y * z * one_c - x * s),
        (z * x * one_c - y * s, z * y * one_c + x * s, c + z * z * one_c),
    )


def _rotation_xyz_deg(matrix: Matrix3) -> Vector3:
    sin_ry = max(-1.0, min(1.0, -matrix[2][0]))
    ry = math.asin(sin_ry)
    cos_ry = math.cos(ry)
    if abs(cos_ry) < 1.0e-8:
        raise RelativeRotationError("relative rotation is too close to XYZ Euler gimbal lock")
    rx = math.atan2(matrix[2][1], matrix[2][2])
    rz = math.atan2(matrix[1][0], matrix[0][0])
    return (math.degrees(rx), math.degrees(ry), math.degrees(rz))


def _rotation_axis(matrix: Matrix3) -> Vector3:
    angle = math.radians(ext.rotation_angle_deg(matrix))
    if angle < 1.0e-9:
        return (1.0, 0.0, 0.0)
    denominator = 2.0 * math.sin(angle)
    axis = (
        (matrix[2][1] - matrix[1][2]) / denominator,
        (matrix[0][2] - matrix[2][0]) / denominator,
        (matrix[1][0] - matrix[0][1]) / denominator,
    )
    return _normalise(axis)


def estimate_relative_body_rotation(
    samples: Sequence[object],
    clock_syncs: Sequence[object],
    *,
    ctrl1_xl: int,
    ctrl2_g: int,
    stationary_window_seconds: float = 0.60,
    end_search_fraction: float = 0.45,
) -> RelativeRotationEstimate:
    """Estimate ``reference_from_body`` for one hold-move-hold capture.

    ``reference_from_body`` maps a vector expressed in the final/current
    ``device_body`` frame into the initial/reference body frame.
    """

    samples = tuple(samples)
    if len(samples) < 40:
        raise RelativeRotationError("at least 40 IMU samples are required")
    if not (0.25 <= float(stationary_window_seconds) <= 2.0):
        raise RelativeRotationError("stationary_window_seconds must be in [0.25, 2.0]")
    if not (0.30 <= float(end_search_fraction) < 0.50):
        raise RelativeRotationError("end_search_fraction must be in [0.30, 0.50)")

    clock_fit = fit_lsm_clock(clock_syncs)
    ticks = [int(getattr(sample, "lsm_tick")) for sample in samples]
    if any(b <= a for a, b in zip(ticks, ticks[1:])):
        raise RelativeRotationError("IMU sample timestamps must be strictly increasing")

    first_us = clock_fit.tick_to_us(ticks[0])
    times_s = [(clock_fit.tick_to_us(tick) - first_us) / 1_000_000.0 for tick in ticks]
    duration = times_s[-1] - times_s[0]
    if duration <= 2.0 * stationary_window_seconds:
        raise RelativeRotationError("capture is too short for two stationary windows")

    accel_scale = accel_g_per_lsb(ctrl1_xl)
    gyro_scale = gyro_dps_per_lsb(ctrl2_g)

    accels_body_g = []
    gyros_body_dps = []
    for sample in samples:
        accel_sensor = (
            float(getattr(sample, "accel_x")) * accel_scale,
            float(getattr(sample, "accel_y")) * accel_scale,
            float(getattr(sample, "accel_z")) * accel_scale,
        )
        gyro_sensor = (
            float(getattr(sample, "gyro_x")) * gyro_scale,
            float(getattr(sample, "gyro_y")) * gyro_scale,
            float(getattr(sample, "gyro_z")) * gyro_scale,
        )
        accels_body_g.append(imu_vector_to_body(accel_sensor))
        gyros_body_dps.append(imu_vector_to_body(gyro_sensor))

    first_region_end = times_s[0] + end_search_fraction * duration
    final_region_start = times_s[-1] - end_search_fraction * duration
    initial_window = _select_stationary_window(
        times_s,
        gyros_body_dps,
        accels_body_g,
        region_start_s=times_s[0],
        region_end_s=first_region_end,
        window_seconds=stationary_window_seconds,
    )
    final_window = _select_stationary_window(
        times_s,
        gyros_body_dps,
        accels_body_g,
        region_start_s=final_region_start,
        region_end_s=times_s[-1],
        window_seconds=stationary_window_seconds,
    )

    start_index = (initial_window.start_index + initial_window.end_index - 1) // 2
    final_index = (final_window.start_index + final_window.end_index - 1) // 2
    if final_index <= start_index:
        raise RelativeRotationError("stationary windows are not ordered in time")

    t0 = times_s[start_index]
    t1 = times_s[final_index]
    if t1 <= t0:
        raise RelativeRotationError("integration interval is empty")

    bias0 = initial_window.gyro_median_body_dps
    bias1 = final_window.gyro_median_body_dps

    def corrected_rate(index: int) -> Vector3:
        fraction = (times_s[index] - t0) / (t1 - t0)
        fraction = max(0.0, min(1.0, fraction))
        bias = tuple(bias0[axis] + fraction * (bias1[axis] - bias0[axis]) for axis in range(3))
        return tuple(gyros_body_dps[index][axis] - bias[axis] for axis in range(3))  # type: ignore[return-value]

    rotation = ext.identity_matrix()
    for index in range(start_index, final_index):
        dt = times_s[index + 1] - times_s[index]
        if dt <= 0.0 or dt > 0.1:
            raise RelativeRotationError(f"invalid IMU timestep {dt:.6f} s")
        rate_a = corrected_rate(index)
        rate_b = corrected_rate(index + 1)
        average_rad_s = tuple(math.radians((rate_a[axis] + rate_b[axis]) / 2.0) for axis in range(3))
        increment = _skew_exp(tuple(value * dt for value in average_rad_s))  # type: ignore[arg-type]
        rotation = ext.matrix_multiply(rotation, increment)

    xyz = _rotation_xyz_deg(rotation)
    angle = ext.rotation_angle_deg(rotation)
    axis = _rotation_axis(rotation)

    start_gravity = _normalise(initial_window.accel_median_body_g)
    final_gravity = _normalise(final_window.accel_median_body_g)
    gravity_change = _angle_deg(start_gravity, final_gravity)
    final_gravity_in_reference = ext.matrix_vector(rotation, final_gravity)
    gravity_closure = _angle_deg(start_gravity, final_gravity_in_reference)

    return RelativeRotationEstimate(
        reference_from_body=rotation,
        rotation_angle_deg=angle,
        rotation_xyz_deg=xyz,
        rotation_axis_reference=axis,
        integrated_duration_s=t1 - t0,
        gravity_direction_change_deg=gravity_change,
        gravity_closure_error_deg=gravity_closure,
        initial_bias_body_dps=bias0,
        final_bias_body_dps=bias1,
        initial_stationary=initial_window,
        final_stationary=final_window,
        clock_fit=clock_fit,
        sample_count=len(samples),
        imu_mapping_role=REFERENCE_IMU_MAPPING_ROLE,
    )
