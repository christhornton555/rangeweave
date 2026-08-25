"""Live/physical flat-plane diagnostics in the Rangeweave ``tof_optical`` frame.

This module is geometry-only. It does not change raw capture semantics and it does
not assume that the completed device body's nominal forward direction is exactly
aligned with the VL53L5CX optical frame.
"""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Sequence

import rangeweave_geometry as geometry


class PlaneAlignmentError(ValueError):
    """Raised when a usable plane cannot be fitted to one ToF grid."""


@dataclass(frozen=True)
class PlaneAlignment:
    normal_x: float
    normal_y: float
    normal_z: float
    offset_mm: float
    rotation_x_deg: float
    rotation_y_deg: float
    rms_residual_mm: float
    max_abs_residual_mm: float
    valid_zones: int


def _solve_3x3(matrix, vector):
    augmented = [
        [float(matrix[row][col]) for col in range(3)] + [float(vector[row])]
        for row in range(3)
    ]
    for col in range(3):
        pivot = max(range(col, 3), key=lambda row: abs(augmented[row][col]))
        if abs(augmented[pivot][col]) <= 1.0e-12:
            raise PlaneAlignmentError("plane fit is singular")
        if pivot != col:
            augmented[col], augmented[pivot] = augmented[pivot], augmented[col]

        scale = augmented[col][col]
        augmented[col] = [value / scale for value in augmented[col]]
        for row in range(3):
            if row == col:
                continue
            factor = augmented[row][col]
            augmented[row] = [
                augmented[row][index] - factor * augmented[col][index]
                for index in range(4)
            ]
    return tuple(augmented[row][3] for row in range(3))


def fit_plane_from_distances(
    distances_mm: Sequence[float],
    profile: geometry.TofGeometryProfile | None = None,
) -> PlaneAlignment:
    """Fit a near-frontal plane to one producer-native 8x8 distance grid.

    Points are first projected with the selected Rangeweave geometry profile.
    The fit uses ``Z = aX + bY + c`` and reports point-to-plane residuals in mm.
    This intentionally describes the target relative to ``tof_optical``; it says
    nothing about the completed device body's mechanical forward direction.
    """

    if len(distances_mm) != geometry.ZONE_COUNT:
        raise PlaneAlignmentError(
            f"expected {geometry.ZONE_COUNT} distances, got {len(distances_mm)}"
        )

    points = [
        point
        for point in geometry.project_distances_mm(distances_mm, profile)
        if point is not None
    ]
    if len(points) < 6:
        raise PlaneAlignmentError("at least six valid ToF zones are required")

    sx2 = sum(point.x_mm * point.x_mm for point in points)
    sxy = sum(point.x_mm * point.y_mm for point in points)
    sy2 = sum(point.y_mm * point.y_mm for point in points)
    sx = sum(point.x_mm for point in points)
    sy = sum(point.y_mm for point in points)
    count = float(len(points))
    sxz = sum(point.x_mm * point.z_mm for point in points)
    syz = sum(point.y_mm * point.z_mm for point in points)
    sz = sum(point.z_mm for point in points)

    a, b, c = _solve_3x3(
        (
            (sx2, sxy, sx),
            (sxy, sy2, sy),
            (sx, sy, count),
        ),
        (sxz, syz, sz),
    )

    length = math.sqrt(a * a + b * b + 1.0)
    nx = -a / length
    ny = -b / length
    nz = 1.0 / length
    offset = c / length

    # This is the same normal convention used by KnownPlanePose:
    # nx = sin(Ry) cos(Rx), ny = -sin(Rx), nz = cos(Ry) cos(Rx).
    rx = math.degrees(math.asin(max(-1.0, min(1.0, -ny))))
    ry = math.degrees(math.atan2(nx, nz))

    residuals = [
        nx * point.x_mm + ny * point.y_mm + nz * point.z_mm - offset
        for point in points
    ]
    rms = math.sqrt(sum(value * value for value in residuals) / len(residuals))
    max_abs = max(abs(value) for value in residuals)

    return PlaneAlignment(
        normal_x=nx,
        normal_y=ny,
        normal_z=nz,
        offset_mm=offset,
        rotation_x_deg=rx,
        rotation_y_deg=ry,
        rms_residual_mm=rms,
        max_abs_residual_mm=max_abs,
        valid_zones=len(points),
    )
