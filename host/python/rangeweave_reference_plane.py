"""Orthogonal best-fit planes for orientation-compensated Rangeweave point clouds.

This module is intentionally independent of the near-frontal ``Z = aX + bY + c``
fit used by the ToF calibration path.  It fits an arbitrary 3D plane by finding
the smallest-eigenvalue direction of the centred point covariance matrix.

For a static flat wall expressed in ``local_reference``, the fitted plane normal
is a direct diagnostic of residual *orientation* error.  Plane offset is not a
stable world-position measurement until sensing-head translation is estimated.
"""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Iterable, Sequence

import rangeweave_geometry as geometry


Vector3 = tuple[float, float, float]


class ReferencePlaneError(ValueError):
    """Raised when a point set cannot define a reliable plane."""


@dataclass(frozen=True)
class PlaneFit:
    normal: Vector3
    centroid_mm: Vector3
    offset_mm: float
    rms_residual_mm: float
    max_abs_residual_mm: float
    point_count: int

    @property
    def rotation_x_deg(self) -> float:
        # Same normal convention used elsewhere in Rangeweave:
        # nx = sin(Ry) cos(Rx), ny = -sin(Rx), nz = cos(Ry) cos(Rx).
        return math.degrees(math.asin(max(-1.0, min(1.0, -self.normal[1]))))

    @property
    def rotation_y_deg(self) -> float:
        return math.degrees(math.atan2(self.normal[0], self.normal[2]))


def _dot(a: Sequence[float], b: Sequence[float]) -> float:
    return sum(float(a[index]) * float(b[index]) for index in range(3))


def _norm(values: Sequence[float]) -> float:
    return math.sqrt(sum(float(value) * float(value) for value in values))


def _normalise(values: Sequence[float]) -> Vector3:
    result = tuple(float(value) for value in values)
    if len(result) != 3 or not all(math.isfinite(value) for value in result):
        raise ReferencePlaneError("normal must contain three finite values")
    length = _norm(result)
    if length <= 1.0e-15:
        raise ReferencePlaneError("cannot normalise a zero vector")
    return tuple(value / length for value in result)  # type: ignore[return-value]


def angle_deg(a: Sequence[float], b: Sequence[float]) -> float:
    na = _normalise(a)
    nb = _normalise(b)
    cosine = max(-1.0, min(1.0, _dot(na, nb)))
    return math.degrees(math.acos(cosine))


def mean_direction(
    vectors: Iterable[Sequence[float]],
    *,
    preferred_normal: Sequence[float] = (0.0, 0.0, 1.0),
) -> Vector3:
    preferred = _normalise(preferred_normal)
    total = [0.0, 0.0, 0.0]
    count = 0
    for raw in vectors:
        vector = _normalise(raw)
        if _dot(vector, preferred) < 0.0:
            vector = tuple(-value for value in vector)  # type: ignore[assignment]
        for axis in range(3):
            total[axis] += vector[axis]
        count += 1
    if count == 0:
        raise ReferencePlaneError("cannot average an empty normal sequence")
    return _normalise(total)


def _smallest_eigenvector_symmetric_3x3(matrix: Sequence[Sequence[float]]) -> Vector3:
    """Return the smallest-eigenvalue eigenvector using Jacobi rotations."""

    a = [[float(matrix[row][col]) for col in range(3)] for row in range(3)]
    v = [[1.0 if row == col else 0.0 for col in range(3)] for row in range(3)]

    for _ in range(40):
        p, q = max(
            ((0, 1), (0, 2), (1, 2)),
            key=lambda pair: abs(a[pair[0]][pair[1]]),
        )
        apq = a[p][q]
        if abs(apq) <= 1.0e-12:
            break
        app = a[p][p]
        aqq = a[q][q]
        phi = 0.5 * math.atan2(2.0 * apq, aqq - app)
        c = math.cos(phi)
        s = math.sin(phi)

        for r in range(3):
            if r == p or r == q:
                continue
            arp = a[r][p]
            arq = a[r][q]
            a[r][p] = a[p][r] = c * arp - s * arq
            a[r][q] = a[q][r] = s * arp + c * arq

        a[p][p] = c * c * app - 2.0 * s * c * apq + s * s * aqq
        a[q][q] = s * s * app + 2.0 * s * c * apq + c * c * aqq
        a[p][q] = a[q][p] = 0.0

        for r in range(3):
            vrp = v[r][p]
            vrq = v[r][q]
            v[r][p] = c * vrp - s * vrq
            v[r][q] = s * vrp + c * vrq

    eigenvalues = [a[index][index] for index in range(3)]
    order = sorted(range(3), key=lambda index: eigenvalues[index])
    if eigenvalues[order[1]] <= 1.0e-9:
        raise ReferencePlaneError("points do not span a two-dimensional plane")
    index = order[0]
    return _normalise((v[0][index], v[1][index], v[2][index]))


def fit_plane(
    points: Sequence[geometry.Point3 | None],
    *,
    preferred_normal: Sequence[float] = (0.0, 0.0, 1.0),
    min_points: int = 6,
) -> PlaneFit:
    valid = [point for point in points if point is not None]
    if len(valid) < int(min_points):
        raise ReferencePlaneError(
            f"at least {int(min_points)} valid points are required, got {len(valid)}"
        )

    count = float(len(valid))
    centroid = (
        sum(point.x_mm for point in valid) / count,
        sum(point.y_mm for point in valid) / count,
        sum(point.z_mm for point in valid) / count,
    )
    covariance = [[0.0, 0.0, 0.0] for _ in range(3)]
    for point in valid:
        delta = (
            point.x_mm - centroid[0],
            point.y_mm - centroid[1],
            point.z_mm - centroid[2],
        )
        for row in range(3):
            for col in range(row, 3):
                covariance[row][col] += delta[row] * delta[col]
    for row in range(3):
        for col in range(row, 3):
            covariance[row][col] /= count
            covariance[col][row] = covariance[row][col]

    normal = _smallest_eigenvector_symmetric_3x3(covariance)
    preferred = _normalise(preferred_normal)
    if _dot(normal, preferred) < 0.0:
        normal = tuple(-value for value in normal)  # type: ignore[assignment]

    offset = _dot(normal, centroid)
    residuals = [
        _dot(normal, (point.x_mm, point.y_mm, point.z_mm)) - offset
        for point in valid
    ]
    rms = math.sqrt(sum(value * value for value in residuals) / len(residuals))
    return PlaneFit(
        normal=normal,
        centroid_mm=centroid,
        offset_mm=offset,
        rms_residual_mm=rms,
        max_abs_residual_mm=max(abs(value) for value in residuals),
        point_count=len(valid),
    )
