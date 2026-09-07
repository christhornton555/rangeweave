"""Diagnostic native-frame magnetometer calibration helpers.

This module intentionally stops short of producing a promoted calibration artifact.
It provides a simple hard-iron-only sphere fit in ``mag_sensor`` coordinates so
Phase 4 can test whether the large orientation-dependent |B| variation is mostly
explained by a fixed additive bias before introducing a full soft-iron ellipsoid fit.
"""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Iterable, Sequence


class MagnetometerCalibrationError(ValueError):
    """Raised when a diagnostic calibration fit is not numerically credible."""


Vector3 = tuple[float, float, float]


@dataclass(frozen=True)
class HardIronSphereFit:
    center_ut: Vector3
    radius_ut: float
    sample_count: int
    raw_norm_p05_ut: float
    raw_norm_median_ut: float
    raw_norm_p95_ut: float
    corrected_norm_p05_ut: float
    corrected_norm_median_ut: float
    corrected_norm_p95_ut: float
    radial_residual_rms_ut: float
    radial_residual_p95_abs_ut: float
    radial_residual_max_abs_ut: float

    @property
    def center_magnitude_ut(self) -> float:
        x, y, z = self.center_ut
        return math.sqrt(x * x + y * y + z * z)

    @property
    def raw_robust_span_ut(self) -> float:
        return self.raw_norm_p95_ut - self.raw_norm_p05_ut

    @property
    def corrected_robust_span_ut(self) -> float:
        return self.corrected_norm_p95_ut - self.corrected_norm_p05_ut

    @property
    def robust_span_reduction_fraction(self) -> float | None:
        raw = self.raw_robust_span_ut
        if raw <= 0.0:
            return None
        return 1.0 - self.corrected_robust_span_ut / raw


def _percentile(values: Sequence[float], probability: float) -> float:
    if not values:
        raise MagnetometerCalibrationError("cannot calculate percentile of empty values")
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


def _median(values: Sequence[float]) -> float:
    ordered = sorted(float(value) for value in values)
    if not ordered:
        raise MagnetometerCalibrationError("cannot calculate median of empty values")
    middle = len(ordered) // 2
    if len(ordered) % 2:
        return ordered[middle]
    return 0.5 * (ordered[middle - 1] + ordered[middle])


def _solve_linear_system(matrix: Sequence[Sequence[float]], rhs: Sequence[float]) -> list[float]:
    n = len(rhs)
    if len(matrix) != n or any(len(row) != n for row in matrix):
        raise MagnetometerCalibrationError("linear system must be square")

    augmented = [list(map(float, row)) + [float(rhs[i])] for i, row in enumerate(matrix)]
    scale = max((abs(value) for row in matrix for value in row), default=0.0)
    if scale == 0.0:
        raise MagnetometerCalibrationError("degenerate calibration geometry")
    tolerance = scale * 1e-12

    for column in range(n):
        pivot_row = max(range(column, n), key=lambda row: abs(augmented[row][column]))
        pivot = augmented[pivot_row][column]
        if abs(pivot) <= tolerance:
            raise MagnetometerCalibrationError(
                "hard-iron sphere fit is singular; orientation coverage is insufficient"
            )
        if pivot_row != column:
            augmented[column], augmented[pivot_row] = augmented[pivot_row], augmented[column]

        pivot = augmented[column][column]
        for j in range(column, n + 1):
            augmented[column][j] /= pivot

        for row in range(n):
            if row == column:
                continue
            factor = augmented[row][column]
            if factor == 0.0:
                continue
            for j in range(column, n + 1):
                augmented[row][j] -= factor * augmented[column][j]

    return [augmented[i][n] for i in range(n)]


def _least_squares(rows: Sequence[Sequence[float]], targets: Sequence[float]) -> list[float]:
    if not rows or len(rows) != len(targets):
        raise MagnetometerCalibrationError("least-squares inputs are empty or mismatched")
    width = len(rows[0])
    if width == 0 or any(len(row) != width for row in rows):
        raise MagnetometerCalibrationError("least-squares design matrix is malformed")

    normal = [[0.0 for _ in range(width)] for _ in range(width)]
    rhs = [0.0 for _ in range(width)]
    for row, target in zip(rows, targets):
        for i in range(width):
            rhs[i] += row[i] * target
            for j in range(width):
                normal[i][j] += row[i] * row[j]
    return _solve_linear_system(normal, rhs)


def _norm(vector: Vector3) -> float:
    x, y, z = vector
    return math.sqrt(x * x + y * y + z * z)


def fit_hard_iron_sphere(vectors_ut: Iterable[Sequence[float]]) -> HardIronSphereFit:
    """Fit a sphere to native magnetic vectors using a linear least-squares model.

    The sphere centre is a diagnostic hard-iron estimate.  This fit assumes one
    approximately constant external field magnitude and does not model soft-iron
    distortion or spatial/temporal environmental field changes.
    """

    vectors = [tuple(float(component) for component in vector) for vector in vectors_ut]
    if len(vectors) < 8:
        raise MagnetometerCalibrationError("hard-iron sphere fit requires at least 8 samples")
    if any(len(vector) != 3 for vector in vectors):
        raise MagnetometerCalibrationError("magnetic vectors must contain exactly three components")

    mean = tuple(sum(vector[axis] for vector in vectors) / len(vectors) for axis in range(3))
    centered = [
        (vector[0] - mean[0], vector[1] - mean[1], vector[2] - mean[2])
        for vector in vectors
    ]
    rms_scale = math.sqrt(sum(_norm(vector) ** 2 for vector in centered) / len(centered))
    if rms_scale <= 1e-12:
        raise MagnetometerCalibrationError("magnetic samples contain no usable orientation spread")

    normalized = [
        (vector[0] / rms_scale, vector[1] / rms_scale, vector[2] / rms_scale)
        for vector in centered
    ]
    rows = []
    targets = []
    for x, y, z in normalized:
        rows.append((2.0 * x, 2.0 * y, 2.0 * z, 1.0))
        targets.append(x * x + y * y + z * z)

    cx_n, cy_n, cz_n, q_n = _least_squares(rows, targets)
    radius_sq_n = q_n + cx_n * cx_n + cy_n * cy_n + cz_n * cz_n
    if radius_sq_n <= 0.0:
        raise MagnetometerCalibrationError("hard-iron sphere fit produced a non-positive radius")

    center = (
        mean[0] + rms_scale * cx_n,
        mean[1] + rms_scale * cy_n,
        mean[2] + rms_scale * cz_n,
    )
    radius = rms_scale * math.sqrt(radius_sq_n)

    raw_norms = [_norm(vector) for vector in vectors]
    corrected_norms = [
        _norm((vector[0] - center[0], vector[1] - center[1], vector[2] - center[2]))
        for vector in vectors
    ]
    residuals = [norm - radius for norm in corrected_norms]
    absolute_residuals = [abs(value) for value in residuals]
    residual_rms = math.sqrt(sum(value * value for value in residuals) / len(residuals))

    return HardIronSphereFit(
        center_ut=center,
        radius_ut=radius,
        sample_count=len(vectors),
        raw_norm_p05_ut=_percentile(raw_norms, 0.05),
        raw_norm_median_ut=_median(raw_norms),
        raw_norm_p95_ut=_percentile(raw_norms, 0.95),
        corrected_norm_p05_ut=_percentile(corrected_norms, 0.05),
        corrected_norm_median_ut=_median(corrected_norms),
        corrected_norm_p95_ut=_percentile(corrected_norms, 0.95),
        radial_residual_rms_ut=residual_rms,
        radial_residual_p95_abs_ut=_percentile(absolute_residuals, 0.95),
        radial_residual_max_abs_ut=max(absolute_residuals),
    )


def subtract_hard_iron(vector_ut: Sequence[float], fit: HardIronSphereFit) -> Vector3:
    if len(vector_ut) != 3:
        raise MagnetometerCalibrationError("magnetic vector must contain exactly three components")
    return (
        float(vector_ut[0]) - fit.center_ut[0],
        float(vector_ut[1]) - fit.center_ut[1],
        float(vector_ut[2]) - fit.center_ut[2],
    )
