"""Physical-workflow geometry helpers for Rangeweave ToF calibration.

This module converts a reproducible centre-pivot board pose into the known-plane
representation consumed by ``rangeweave_tof_calibration``. It deliberately uses
explicit rotations about frozen ``tof_optical`` axes instead of ambiguous
pitch/yaw names.
"""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Sequence

import rangeweave_tof_calibration as calibration


PIVOT_POSE_CONVENTION = "tof-optical-centre-pivot-rx-then-ry-v1"


@dataclass(frozen=True)
class PivotPlanePose:
    """Known calibration-board pose about a pivot on the ``tof_optical`` Z axis.

    The pivot is ``P = (0, 0, pivot_distance_mm)``. A fronto-parallel board has
    plane normal ``n0 = (0, 0, +1)``. The normal is actively rotated about the
    fixed ``tof_optical`` axes, first by ``rotation_x_deg`` about +X and then by
    ``rotation_y_deg`` about +Y, using the right-hand rule.

    Because ``tof_optical`` +Y points down, +Rx makes the board's lower (+Y)
    side farther from the sensor. +Ry makes the board's right (+X) side closer.
    """

    pivot_distance_mm: float
    rotation_x_deg: float = 0.0
    rotation_y_deg: float = 0.0

    def __post_init__(self) -> None:
        values = (
            float(self.pivot_distance_mm),
            float(self.rotation_x_deg),
            float(self.rotation_y_deg),
        )
        if not all(math.isfinite(value) for value in values):
            raise calibration.CalibrationError("pivot pose values must be finite")
        if self.pivot_distance_mm <= 0.0:
            raise calibration.CalibrationError(
                "pivot_distance_mm must be greater than zero"
            )
        # Validate that the chosen rotations leave the canonical +Z-oriented
        # plane with a positive forward component and positive plane offset.
        self.normal_and_offset()

    def normal_and_offset(self) -> tuple[float, float, float, float]:
        """Return unit ``(nx, ny, nz)`` and plane offset ``d`` in millimetres.

        Rotation order is executable project policy:

            n = Ry(rotation_y_deg) * Rx(rotation_x_deg) * (0, 0, 1)

        With the pivot fixed at ``P = (0, 0, D)``, the plane equation
        ``n dot X = d`` has ``d = n dot P = nz * D``.
        """

        rx = math.radians(float(self.rotation_x_deg))
        ry = math.radians(float(self.rotation_y_deg))

        sin_x = math.sin(rx)
        cos_x = math.cos(rx)
        sin_y = math.sin(ry)
        cos_y = math.cos(ry)

        # Active right-handed Rx first, then active right-handed Ry, both
        # expressed in the fixed tof_optical basis.
        nx = sin_y * cos_x
        ny = -sin_x
        nz = cos_y * cos_x

        if nz <= 0.0:
            raise calibration.CalibrationError(
                "pivot pose must keep the calibration-plane normal in +tof_optical Z"
            )

        offset_mm = nz * float(self.pivot_distance_mm)
        return nx, ny, nz, offset_mm


def calibration_plane_from_pivot_pose(
    pose: PivotPlanePose,
    distances_mm: Sequence[float | None],
    *,
    label: str = "",
) -> calibration.CalibrationPlane:
    """Attach one 64-zone observation to a canonical pivot-board pose."""

    nx, ny, nz, offset_mm = pose.normal_and_offset()
    return calibration.CalibrationPlane(
        normal_x=nx,
        normal_y=ny,
        normal_z=nz,
        offset_mm=offset_mm,
        distances_mm=tuple(distances_mm),
        label=label,
    )
