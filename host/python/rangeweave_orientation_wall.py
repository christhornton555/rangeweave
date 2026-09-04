"""Continuous fixed-wall validation for Rangeweave persistent orientation.

This module combines three already-defined transforms/timelines without claiming
translation or SLAM:

    n_tof
      -> R_body_from_tof
      -> R_reference_from_body(t)
      -> n_reference

For a static physical wall, the resulting normal should remain approximately
constant while the sensing head rotates in place.

Protocol v0.1 timestamps ToF with ``mcu_ready_us``: the MCU time at which software
observed the sensor's data-ready condition. The physical ranging result existed no
later than that time, but the exact internal ranging instant is not exposed. This
module therefore keeps any applied ToF time offset explicit.
"""

from __future__ import annotations

from bisect import bisect_left
from dataclasses import dataclass
import math
import statistics
from typing import Sequence

import rangeweave_extrinsics as ext
import rangeweave_orientation as orientation
import rangeweave_plane_alignment as plane


Vector3 = ext.Vector3
Matrix3 = ext.Matrix3
Quaternion = orientation.Quaternion


class WallOrientationError(ValueError):
    """Raised when a capture cannot support fixed-wall orientation validation."""


@dataclass(frozen=True)
class InterpolatedOrientation:
    reference_from_body: Matrix3
    quaternion_reference_from_body: Quaternion
    accel_weight: float
    gravity_innovation_deg: float
    bracket_gap_s: float


@dataclass(frozen=True)
class WallNormalObservation:
    time_s: float
    mcu_ready_us: int
    normal_tof: Vector3
    normal_body: Vector3
    normal_reference: Vector3
    plane_rms_mm: float
    plane_max_abs_mm: float
    valid_zones: int
    accel_weight: float
    gravity_innovation_deg: float
    orientation_bracket_gap_s: float


@dataclass(frozen=True)
class WallNormalStability:
    observations: tuple[WallNormalObservation, ...]
    total_tof_grids: int
    rejected_missing_distance: int
    rejected_geometry_shape: int
    rejected_plane_quality: int
    rejected_outside_orientation: int
    reference_normal: Vector3
    residual_median_deg: float
    residual_rms_deg: float
    residual_p95_deg: float
    residual_max_deg: float
    start_normal: Vector3
    end_normal: Vector3
    start_end_error_deg: float
    start_count: int
    end_count: int
    orientation_excursion_deg: float
    tof_time_offset_ms: float


@dataclass(frozen=True)
class TimeOffsetCandidate:
    offset_ms: float
    observation_count: int
    residual_rms_deg: float
    residual_p95_deg: float
    residual_max_deg: float


def _vector3(values: Sequence[float]) -> Vector3:
    result = tuple(float(value) for value in values)
    if len(result) != 3 or not all(math.isfinite(value) for value in result):
        raise WallOrientationError("vector must contain three finite values")
    return result  # type: ignore[return-value]


def _norm(values: Sequence[float]) -> float:
    return math.sqrt(sum(float(value) * float(value) for value in values))


def _normalise(values: Sequence[float]) -> Vector3:
    result = _vector3(values)
    length = _norm(result)
    if length <= 1.0e-12:
        raise WallOrientationError("cannot normalise a zero vector")
    return tuple(value / length for value in result)  # type: ignore[return-value]


def _dot(a: Sequence[float], b: Sequence[float]) -> float:
    return sum(float(a[index]) * float(b[index]) for index in range(3))


def angle_deg(a: Sequence[float], b: Sequence[float]) -> float:
    na = _normalise(a)
    nb = _normalise(b)
    cosine = max(-1.0, min(1.0, _dot(na, nb)))
    return math.degrees(math.acos(cosine))


def mean_direction(vectors: Sequence[Sequence[float]]) -> Vector3:
    if not vectors:
        raise WallOrientationError("cannot average an empty direction sequence")
    total = [0.0, 0.0, 0.0]
    for vector in vectors:
        unit = _normalise(vector)
        for axis in range(3):
            total[axis] += unit[axis]
    if _norm(total) <= 1.0e-9:
        raise WallOrientationError("wall-normal directions cancel and cannot define a mean")
    return _normalise(total)


def percentile(values: Sequence[float], probability: float) -> float:
    if not values:
        raise WallOrientationError("cannot calculate a percentile of an empty sequence")
    p = float(probability)
    if not 0.0 <= p <= 1.0:
        raise WallOrientationError("percentile probability must be in [0, 1]")
    ordered = sorted(float(value) for value in values)
    if len(ordered) == 1:
        return ordered[0]
    position = p * (len(ordered) - 1)
    lower = int(math.floor(position))
    upper = int(math.ceil(position))
    if lower == upper:
        return ordered[lower]
    fraction = position - lower
    return ordered[lower] * (1.0 - fraction) + ordered[upper] * fraction


def quaternion_slerp(
    left: Sequence[float],
    right: Sequence[float],
    fraction: float,
) -> Quaternion:
    """Shortest-path SLERP for scalar-first Hamilton quaternions."""

    q0 = orientation.quaternion_normalise(left)
    q1 = orientation.quaternion_normalise(right)
    f = max(0.0, min(1.0, float(fraction)))
    dot = sum(q0[index] * q1[index] for index in range(4))
    if dot < 0.0:
        q1 = tuple(-value for value in q1)  # type: ignore[assignment]
        dot = -dot
    dot = max(-1.0, min(1.0, dot))
    if dot > 0.9995:
        blended = tuple(q0[index] + f * (q1[index] - q0[index]) for index in range(4))
        return orientation.quaternion_normalise(blended)
    theta = math.acos(dot)
    sin_theta = math.sin(theta)
    a = math.sin((1.0 - f) * theta) / sin_theta
    b = math.sin(f * theta) / sin_theta
    return orientation.quaternion_normalise(
        tuple(a * q0[index] + b * q1[index] for index in range(4))
    )


def interpolate_orientation(
    samples: Sequence[orientation.OrientationSample],
    time_s: float,
) -> InterpolatedOrientation:
    samples = tuple(samples)
    if len(samples) < 2:
        raise WallOrientationError("at least two orientation samples are required")
    times = [sample.time_s for sample in samples]
    target = float(time_s)
    if target < times[0] or target > times[-1]:
        raise WallOrientationError("requested time is outside the orientation run")
    index = bisect_left(times, target)
    if index == 0:
        sample = samples[0]
        return InterpolatedOrientation(
            reference_from_body=sample.reference_from_body,
            quaternion_reference_from_body=sample.quaternion_reference_from_body,
            accel_weight=sample.accel_weight,
            gravity_innovation_deg=sample.gravity_innovation_deg,
            bracket_gap_s=0.0,
        )
    if index >= len(samples):
        sample = samples[-1]
        return InterpolatedOrientation(
            reference_from_body=sample.reference_from_body,
            quaternion_reference_from_body=sample.quaternion_reference_from_body,
            accel_weight=sample.accel_weight,
            gravity_innovation_deg=sample.gravity_innovation_deg,
            bracket_gap_s=0.0,
        )

    left = samples[index - 1]
    right = samples[index]
    gap = right.time_s - left.time_s
    if gap <= 0.0:
        raise WallOrientationError("orientation sample times are not strictly increasing")
    fraction = (target - left.time_s) / gap
    q = quaternion_slerp(
        left.quaternion_reference_from_body,
        right.quaternion_reference_from_body,
        fraction,
    )
    weight = left.accel_weight + fraction * (right.accel_weight - left.accel_weight)
    innovation = (
        left.gravity_innovation_deg
        + fraction * (right.gravity_innovation_deg - left.gravity_innovation_deg)
    )
    return InterpolatedOrientation(
        reference_from_body=orientation.quaternion_to_matrix(q),
        quaternion_reference_from_body=q,
        accel_weight=weight,
        gravity_innovation_deg=innovation,
        bracket_gap_s=gap,
    )


def _orientation_excursion_deg(samples: Sequence[orientation.OrientationSample]) -> float:
    if not samples:
        return 0.0
    start = samples[0].reference_from_body
    start_transpose = ext.transpose(start)
    return max(
        ext.rotation_angle_deg(ext.matrix_multiply(start_transpose, sample.reference_from_body))
        for sample in samples
    )


def evaluate_wall_stability(
    tof_grids: Sequence[object],
    imu_samples: Sequence[object],
    orientation_run: orientation.OrientationRun,
    rotation_body_from_tof: Matrix3,
    *,
    tof_time_offset_ms: float = 0.0,
    min_valid_zones: int = 48,
    max_plane_rms_mm: float = 10.0,
    max_plane_residual_mm: float = 30.0,
    start_end_window_s: float = 3.0,
) -> WallNormalStability:
    """Evaluate transformed wall-normal stability over one continuous capture."""

    tof_grids = tuple(tof_grids)
    imu_samples = tuple(imu_samples)
    if not tof_grids:
        raise WallOrientationError("capture contains no ToF grids")
    if not imu_samples:
        raise WallOrientationError("capture contains no IMU samples")
    if not 6 <= int(min_valid_zones) <= 64:
        raise WallOrientationError("min_valid_zones must be in [6, 64]")
    if float(max_plane_rms_mm) <= 0.0 or float(max_plane_residual_mm) <= 0.0:
        raise WallOrientationError("plane-quality limits must be positive")
    if float(start_end_window_s) <= 0.0:
        raise WallOrientationError("start_end_window_s must be positive")

    offset_us = float(tof_time_offset_ms) * 1000.0
    first_imu_tick = int(getattr(imu_samples[0], "lsm_tick"))
    origin_mcu_us = orientation_run.clock_fit.tick_to_us(first_imu_tick)

    observations = []
    missing_distance = 0
    geometry_shape = 0
    plane_quality = 0
    outside_orientation = 0

    for grid in tof_grids:
        distances = getattr(grid, "distance_mm", None)
        if distances is None:
            missing_distance += 1
            continue
        rows = int(getattr(grid, "rows"))
        cols = int(getattr(grid, "cols"))
        if rows != 8 or cols != 8 or len(distances) != 64:
            geometry_shape += 1
            continue

        ready_us = int(getattr(grid, "mcu_ready_us"))
        time_s = ((float(ready_us) + offset_us) - origin_mcu_us) / 1_000_000.0
        if (
            time_s < orientation_run.samples[0].time_s
            or time_s > orientation_run.samples[-1].time_s
        ):
            outside_orientation += 1
            continue

        try:
            fitted = plane.fit_plane_from_distances(distances)
        except (plane.PlaneAlignmentError, ValueError):
            plane_quality += 1
            continue
        if (
            fitted.valid_zones < int(min_valid_zones)
            or fitted.rms_residual_mm > float(max_plane_rms_mm)
            or fitted.max_abs_residual_mm > float(max_plane_residual_mm)
        ):
            plane_quality += 1
            continue

        interpolated = interpolate_orientation(orientation_run.samples, time_s)
        n_tof: Vector3 = (
            fitted.normal_x,
            fitted.normal_y,
            fitted.normal_z,
        )
        n_body = _normalise(ext.matrix_vector(rotation_body_from_tof, n_tof))
        n_reference = _normalise(
            ext.matrix_vector(interpolated.reference_from_body, n_body)
        )
        observations.append(
            WallNormalObservation(
                time_s=time_s,
                mcu_ready_us=ready_us,
                normal_tof=n_tof,
                normal_body=n_body,
                normal_reference=n_reference,
                plane_rms_mm=fitted.rms_residual_mm,
                plane_max_abs_mm=fitted.max_abs_residual_mm,
                valid_zones=fitted.valid_zones,
                accel_weight=interpolated.accel_weight,
                gravity_innovation_deg=interpolated.gravity_innovation_deg,
                orientation_bracket_gap_s=interpolated.bracket_gap_s,
            )
        )

    if len(observations) < 10:
        raise WallOrientationError(
            f"only {len(observations)} usable ToF wall observations remain after quality/timing checks"
        )

    reference = mean_direction([item.normal_reference for item in observations])
    residuals = [angle_deg(reference, item.normal_reference) for item in observations]
    residual_rms = math.sqrt(sum(value * value for value in residuals) / len(residuals))

    first_time = observations[0].time_s
    last_time = observations[-1].time_s
    window = float(start_end_window_s)
    start_items = [item for item in observations if item.time_s <= first_time + window]
    end_items = [item for item in observations if item.time_s >= last_time - window]
    if not start_items or not end_items:
        raise WallOrientationError("could not form start/end wall-normal windows")
    start_normal = mean_direction([item.normal_reference for item in start_items])
    end_normal = mean_direction([item.normal_reference for item in end_items])

    return WallNormalStability(
        observations=tuple(observations),
        total_tof_grids=len(tof_grids),
        rejected_missing_distance=missing_distance,
        rejected_geometry_shape=geometry_shape,
        rejected_plane_quality=plane_quality,
        rejected_outside_orientation=outside_orientation,
        reference_normal=reference,
        residual_median_deg=float(statistics.median(residuals)),
        residual_rms_deg=residual_rms,
        residual_p95_deg=percentile(residuals, 0.95),
        residual_max_deg=max(residuals),
        start_normal=start_normal,
        end_normal=end_normal,
        start_end_error_deg=angle_deg(start_normal, end_normal),
        start_count=len(start_items),
        end_count=len(end_items),
        orientation_excursion_deg=_orientation_excursion_deg(orientation_run.samples),
        tof_time_offset_ms=float(tof_time_offset_ms),
    )


def scan_time_offsets(
    tof_grids: Sequence[object],
    imu_samples: Sequence[object],
    orientation_run: orientation.OrientationRun,
    rotation_body_from_tof: Matrix3,
    offsets_ms: Sequence[float],
    **kwargs,
) -> tuple[TimeOffsetCandidate, ...]:
    """Exploratory timing-offset scan; candidates are sorted by RMS residual.

    This is diagnostic only. A best offset from one capture must not silently become
    a calibration parameter without repeatability evidence.
    """

    candidates = []
    for offset_ms in offsets_ms:
        try:
            result = evaluate_wall_stability(
                tof_grids,
                imu_samples,
                orientation_run,
                rotation_body_from_tof,
                tof_time_offset_ms=float(offset_ms),
                **kwargs,
            )
        except WallOrientationError:
            continue
        candidates.append(
            TimeOffsetCandidate(
                offset_ms=float(offset_ms),
                observation_count=len(result.observations),
                residual_rms_deg=result.residual_rms_deg,
                residual_p95_deg=result.residual_p95_deg,
                residual_max_deg=result.residual_max_deg,
            )
        )
    return tuple(sorted(candidates, key=lambda item: item.residual_rms_deg))
