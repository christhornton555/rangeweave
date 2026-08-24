"""Physical-workflow geometry helpers for optional Rangeweave ToF calibration.

This module converts a measured calibration-board pose into the known-plane
representation consumed by ``rangeweave_tof_calibration``. It deliberately uses
explicit rotations about frozen ``tof_optical`` axes instead of ambiguous
pitch/yaw names.

Per-device geometry calibration is an optional precision-refinement step.
Uncalibrated systems continue to use Rangeweave's built-in ST nominal fallback.
"""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Sequence

import rangeweave_tof_calibration as calibration


PLANE_POSE_CONVENTION = "tof-optical-known-point-rx-then-ry-v1"


@dataclass(frozen=True)
class KnownPlanePose:
    """Known calibration-board plane pose in ``tof_optical``.

    ``point_*_mm`` is any measured point known to lie on the board plane. In a
    simple builder setup this will normally be the marked board centre. A
    fronto-parallel board has normal ``n0 = (0, 0, +1)``. The normal is actively
    rotated about the fixed ``tof_optical`` axes, first by ``rotation_x_deg``
    about +X and then by ``rotation_y_deg`` about +Y, using the right-hand rule.

    The measured point is not required to stay fixed between captures. Therefore
    no centre pivot, gimbal, hinge geometry, or fixed sensor-board distance is
    required by the calibration model.

    Because ``tof_optical`` +Y points down, +Rx makes the board's lower (+Y)
    side farther from the sensor. +Ry makes the board's right (+X) side closer.
    """

    point_x_mm: float
    point_y_mm: float
    point_z_mm: float
    rotation_x_deg: float = 0.0
    rotation_y_deg: float = 0.0

    def __post_init__(self) -> None:
        values = (
            float(self.point_x_mm),
            float(self.point_y_mm),
            float(self.point_z_mm),
            float(self.rotation_x_deg),
            float(self.rotation_y_deg),
        )
        if not all(math.isfinite(value) for value in values):
            raise calibration.CalibrationError("plane pose values must be finite")
        if self.point_z_mm <= 0.0:
            raise calibration.CalibrationError("point_z_mm must be greater than zero")
        self.normal_and_offset()

    @classmethod
    def centre_on_optical_axis(
        cls,
        distance_mm: float,
        *,
        rotation_x_deg: float = 0.0,
        rotation_y_deg: float = 0.0,
    ) -> "KnownPlanePose":
        """Convenience pose for a marked board centre at ``(0, 0, distance)``.

        The distance may differ between captures. This helper does not imply that
        the board rotates about that point.
        """

        return cls(
            point_x_mm=0.0,
            point_y_mm=0.0,
            point_z_mm=distance_mm,
            rotation_x_deg=rotation_x_deg,
            rotation_y_deg=rotation_y_deg,
        )

    def normal_and_offset(self) -> tuple[float, float, float, float]:
        """Return unit ``(nx, ny, nz)`` and plane offset ``d`` in millimetres.

        Rotation order is executable project policy:

            n = Ry(rotation_y_deg) * Rx(rotation_x_deg) * (0, 0, 1)

        For any known point ``P`` on the measured board plane, the plane equation
        ``n dot X = d`` has ``d = n dot P``.
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
                "plane pose must keep the calibration-plane normal in +tof_optical Z"
            )

        offset_mm = (
            nx * float(self.point_x_mm)
            + ny * float(self.point_y_mm)
            + nz * float(self.point_z_mm)
        )
        if offset_mm <= 0.0:
            raise calibration.CalibrationError(
                "known plane point/orientation produces a non-positive plane offset"
            )
        return nx, ny, nz, offset_mm


def calibration_plane_from_pose(
    pose: KnownPlanePose,
    distances_mm: Sequence[float | None],
    *,
    label: str = "",
) -> calibration.CalibrationPlane:
    """Attach one 64-zone observation to a measured calibration-board pose."""

    nx, ny, nz, offset_mm = pose.normal_and_offset()
    return calibration.CalibrationPlane(
        normal_x=nx,
        normal_y=ny,
        normal_z=nz,
        offset_mm=offset_mm,
        distances_mm=tuple(distances_mm),
        label=label,
    )
