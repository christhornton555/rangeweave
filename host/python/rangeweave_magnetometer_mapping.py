"""Diagnostic ``mag_sensor -> device_body`` candidate evaluation.

This module is intentionally conservative.  It searches only the 24 proper
signed-permutation rotations (axis swaps/sign flips with determinant +1), which is
useful for orthogonally mounted breakout boards and for planning the physical axis
validation.  It does not promote a calibration artifact and it does not claim that
an arbitrary assembly must be representable by one of these matrices.

A candidate is evaluated by mapping the native magnetic vector into ``device_body``
and then through the already-estimated ``R_reference_from_body(t)``.  In a locally
static magnetic environment the resulting reference-frame field direction should be
approximately constant while the rig rotates.  Hard/soft-iron distortion and magnetic
sample timing uncertainty are deliberately left visible in the residuals.
"""

from __future__ import annotations

from dataclasses import dataclass
from itertools import permutations, product
import math
import statistics
from typing import Sequence

import rangeweave_extrinsics as ext
import rangeweave_magnetometer as mag
import rangeweave_orientation as orientation
import rangeweave_orientation_wall as orientation_wall


class MagnetometerMappingError(ValueError):
    """Raised when a mapping diagnostic cannot be evaluated safely."""


@dataclass(frozen=True)
class AxisMappingCandidate:
    name: str
    rotation_body_from_mag: ext.Matrix3


@dataclass(frozen=True)
class MappingScore:
    mapping: AxisMappingCandidate
    time_offset_ms: float
    sample_count: int
    mean_direction_reference: ext.Vector3
    direction_median_deg: float
    direction_rms_deg: float
    direction_p95_deg: float
    direction_max_deg: float
    vector_rms_ut: float


def _permutation_parity(items: Sequence[int]) -> int:
    inversions = sum(
        1
        for left in range(len(items))
        for right in range(left + 1, len(items))
        if items[left] > items[right]
    )
    return -1 if inversions % 2 else 1


def _determinant(matrix: ext.Matrix3) -> float:
    return (
        matrix[0][0] * (matrix[1][1] * matrix[2][2] - matrix[1][2] * matrix[2][1])
        - matrix[0][1] * (matrix[1][0] * matrix[2][2] - matrix[1][2] * matrix[2][0])
        + matrix[0][2] * (matrix[1][0] * matrix[2][1] - matrix[1][1] * matrix[2][0])
    )


def proper_signed_permutation_mappings() -> tuple[AxisMappingCandidate, ...]:
    """Return all 24 right-handed axis-swap/sign-flip rotations."""

    labels = ("X", "Y", "Z")
    candidates = []
    for permutation in permutations(range(3)):
        parity = _permutation_parity(permutation)
        for signs in product((-1, 1), repeat=3):
            if parity * signs[0] * signs[1] * signs[2] != 1:
                continue
            rows = []
            clauses = []
            for body_axis, sensor_axis in enumerate(permutation):
                row = [0.0, 0.0, 0.0]
                row[sensor_axis] = float(signs[body_axis])
                rows.append(tuple(row))
                clauses.append(
                    "body {}={}mag {}".format(
                        labels[body_axis],
                        "+" if signs[body_axis] > 0 else "-",
                        labels[sensor_axis],
                    )
                )
            matrix: ext.Matrix3 = tuple(rows)  # type: ignore[assignment]
            if abs(_determinant(matrix) - 1.0) > 1.0e-9:
                raise MagnetometerMappingError("generated mapping is not a proper rotation")
            candidates.append(AxisMappingCandidate("; ".join(clauses), matrix))
    if len(candidates) != 24:
        raise MagnetometerMappingError(
            f"expected 24 proper signed-permutation mappings, generated {len(candidates)}"
        )
    return tuple(sorted(candidates, key=lambda item: item.name))


def _mean_vector(vectors: Sequence[ext.Vector3]) -> ext.Vector3:
    if not vectors:
        raise MagnetometerMappingError("cannot average an empty vector sequence")
    return tuple(
        sum(vector[axis] for vector in vectors) / len(vectors)
        for axis in range(3)
    )  # type: ignore[return-value]


def score_aligned_vectors(
    sensor_vectors_ut: Sequence[ext.Vector3],
    reference_from_body: Sequence[ext.Matrix3],
    mapping: AxisMappingCandidate,
    *,
    time_offset_ms: float = 0.0,
) -> MappingScore:
    """Score one candidate using already time-aligned body attitudes."""

    if len(sensor_vectors_ut) != len(reference_from_body):
        raise MagnetometerMappingError("magnetic vectors and attitudes must have equal length")
    if len(sensor_vectors_ut) < 3:
        raise MagnetometerMappingError("at least three aligned magnetic samples are required")

    vectors_reference = []
    for vector_sensor, rotation_reference_from_body in zip(
        sensor_vectors_ut, reference_from_body
    ):
        vector_body = ext.matrix_vector(mapping.rotation_body_from_mag, vector_sensor)
        vectors_reference.append(
            ext.matrix_vector(rotation_reference_from_body, vector_body)
        )

    mean_direction = orientation_wall.mean_direction(vectors_reference)
    residuals = [
        orientation_wall.angle_deg(mean_direction, vector)
        for vector in vectors_reference
    ]
    mean_vector = _mean_vector(vectors_reference)
    vector_squared_errors = []
    for vector in vectors_reference:
        vector_squared_errors.append(
            sum((vector[axis] - mean_vector[axis]) ** 2 for axis in range(3))
        )

    return MappingScore(
        mapping=mapping,
        time_offset_ms=float(time_offset_ms),
        sample_count=len(vectors_reference),
        mean_direction_reference=mean_direction,
        direction_median_deg=float(statistics.median(residuals)),
        direction_rms_deg=math.sqrt(sum(value * value for value in residuals) / len(residuals)),
        direction_p95_deg=orientation_wall.percentile(residuals, 0.95),
        direction_max_deg=max(residuals),
        vector_rms_ut=math.sqrt(sum(vector_squared_errors) / len(vector_squared_errors)),
    )


def _aligned_for_offset(
    samples: Sequence[mag.MagneticSample],
    imu_samples: Sequence[object],
    orientation_run: orientation.OrientationRun,
    offset_ms: float,
) -> tuple[tuple[ext.Vector3, ...], tuple[ext.Matrix3, ...]]:
    imu_samples = tuple(imu_samples)
    if not imu_samples:
        raise MagnetometerMappingError("at least one IMU sample is required")
    if len(orientation_run.samples) < 2:
        raise MagnetometerMappingError("orientation run contains fewer than two samples")

    first_imu_tick = int(getattr(imu_samples[0], "lsm_tick"))
    origin_mcu_us = float(orientation_run.clock_fit.tick_to_us(first_imu_tick))
    vectors = []
    rotations = []
    for sample in samples:
        if not sample.data_ready:
            continue
        time_s = (
            float(sample.time_us) + float(offset_ms) * 1000.0 - origin_mcu_us
        ) / 1_000_000.0
        if (
            time_s < orientation_run.samples[0].time_s
            or time_s > orientation_run.samples[-1].time_s
        ):
            continue
        try:
            interpolated = orientation_wall.interpolate_orientation(
                orientation_run.samples, time_s
            )
        except orientation_wall.WallOrientationError:
            continue
        vectors.append((sample.x_ut, sample.y_ut, sample.z_ut))
        rotations.append(interpolated.reference_from_body)
    return tuple(vectors), tuple(rotations)


def scan_signed_permutation_mappings(
    samples: Sequence[mag.MagneticSample],
    imu_samples: Sequence[object],
    orientation_run: orientation.OrientationRun,
    offsets_ms: Sequence[float],
) -> tuple[MappingScore, ...]:
    """Jointly scan proper axis mappings and a constant MAG observation-time offset.

    Results are diagnostic only and sorted by angular RMS.  A low score from one
    capture must not be promoted as a physical mapping without a purpose-made axis
    validation and later hard/soft-iron calibration evidence.
    """

    mappings = proper_signed_permutation_mappings()
    scores = []
    for offset_ms in offsets_ms:
        vectors, rotations = _aligned_for_offset(
            samples, imu_samples, orientation_run, float(offset_ms)
        )
        if len(vectors) < 10:
            continue
        for mapping in mappings:
            try:
                scores.append(
                    score_aligned_vectors(
                        vectors,
                        rotations,
                        mapping,
                        time_offset_ms=float(offset_ms),
                    )
                )
            except (MagnetometerMappingError, orientation_wall.WallOrientationError):
                continue
    if not scores:
        raise MagnetometerMappingError("no usable mapping/timing candidates could be evaluated")
    return tuple(
        sorted(
            scores,
            key=lambda item: (
                item.direction_rms_deg,
                item.direction_p95_deg,
                item.vector_rms_ut,
                abs(item.time_offset_ms),
                item.mapping.name,
            ),
        )
    )


def best_distinct_mappings(scores: Sequence[MappingScore]) -> tuple[MappingScore, ...]:
    """Return each mapping once, retaining its best scanned timing offset."""

    best = {}
    for score in scores:
        best.setdefault(score.mapping.name, score)
    return tuple(
        sorted(
            best.values(),
            key=lambda item: (
                item.direction_rms_deg,
                item.direction_p95_deg,
                item.vector_rms_ut,
            ),
        )
    )
