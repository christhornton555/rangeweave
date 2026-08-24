"""Generic plane-based calibration for Rangeweave ToF zone geometry.

The solver estimates each producer-native zone independently. It does not impose
symmetry, equal spacing, monotonic spacing, or inward/outward curvature.
"""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Sequence

import rangeweave_geometry as geometry


CALIBRATION_METHOD = "known-plane-independent-zone-least-squares-v1"


class CalibrationError(ValueError):
    """Raised when calibration observations cannot determine a geometry profile."""


@dataclass(frozen=True)
class CalibrationPlane:
    """One known plane and the 64 axial-Z measurements observed against it.

    Plane equation in ``tof_optical``:
        normal_x * X + normal_y * Y + normal_z * Z = offset_mm

    The normal need not be unit length on input; the solver normalizes it and the
    offset together. Invalid/non-positive per-zone distances are skipped.
    """

    normal_x: float
    normal_y: float
    normal_z: float
    offset_mm: float
    distances_mm: tuple[float | None, ...]
    label: str = ""

    def __post_init__(self) -> None:
        if len(self.distances_mm) != geometry.ZONE_COUNT:
            raise CalibrationError(
                f"calibration plane must contain {geometry.ZONE_COUNT} zone distances"
            )
        values = (
            float(self.normal_x),
            float(self.normal_y),
            float(self.normal_z),
            float(self.offset_mm),
        )
        if not all(math.isfinite(value) for value in values):
            raise CalibrationError("calibration plane normal/offset must be finite")
        if self.offset_mm <= 0.0:
            raise CalibrationError("calibration plane offset_mm must be greater than zero")
        norm = math.sqrt(
            self.normal_x * self.normal_x
            + self.normal_y * self.normal_y
            + self.normal_z * self.normal_z
        )
        if norm <= 0.0:
            raise CalibrationError("calibration plane normal must be non-zero")
        object.__setattr__(self, "distances_mm", tuple(self.distances_mm))

    def normalised(self) -> tuple[float, float, float, float]:
        norm = math.sqrt(
            self.normal_x * self.normal_x
            + self.normal_y * self.normal_y
            + self.normal_z * self.normal_z
        )
        return (
            self.normal_x / norm,
            self.normal_y / norm,
            self.normal_z / norm,
            self.offset_mm / norm,
        )


@dataclass(frozen=True)
class ZoneCalibrationFit:
    zone_id: int
    x_per_z: float
    y_per_z: float
    observations: int
    rms_plane_residual_mm: float
    max_abs_plane_residual_mm: float


@dataclass(frozen=True)
class GeometryCalibrationResult:
    profile: geometry.TofGeometryProfile
    zone_fits: tuple[ZoneCalibrationFit, ...]


def _valid_distance(value: float | None) -> float | None:
    if value is None:
        return None
    distance = float(value)
    if not math.isfinite(distance) or distance <= 0.0:
        return None
    return distance


def solve_zone_slopes(
    zone_id: int,
    planes: Sequence[CalibrationPlane],
) -> ZoneCalibrationFit:
    """Solve one zone's independent X/Z and Y/Z slopes by linear least squares."""

    geometry.projection_vector(zone_id)  # validates producer zone ID
    if len(planes) < 2:
        raise CalibrationError("at least two plane observations are required")

    s_xx = 0.0
    s_xy = 0.0
    s_yy = 0.0
    s_xb = 0.0
    s_yb = 0.0
    observations: list[tuple[float, float, float, float, float]] = []

    for plane in planes:
        distance = _valid_distance(plane.distances_mm[zone_id])
        if distance is None:
            continue
        nx, ny, nz, offset = plane.normalised()
        rhs = offset / distance - nz
        s_xx += nx * nx
        s_xy += nx * ny
        s_yy += ny * ny
        s_xb += nx * rhs
        s_yb += ny * rhs
        observations.append((nx, ny, nz, offset, distance))

    if len(observations) < 2:
        raise CalibrationError(
            f"zone {zone_id} has fewer than two valid plane observations"
        )

    determinant = s_xx * s_yy - s_xy * s_xy
    conditioning_scale = max(s_xx * s_yy, 1.0e-30)
    if determinant <= 1.0e-10 * conditioning_scale:
        raise CalibrationError(
            f"zone {zone_id} plane set does not span independent X and Y tilts"
        )

    x_per_z = (s_xb * s_yy - s_yb * s_xy) / determinant
    y_per_z = (s_yb * s_xx - s_xb * s_xy) / determinant

    residuals = [
        nx * (x_per_z * distance)
        + ny * (y_per_z * distance)
        + nz * distance
        - offset
        for nx, ny, nz, offset, distance in observations
    ]
    rms = math.sqrt(sum(value * value for value in residuals) / len(residuals))
    max_abs = max(abs(value) for value in residuals)

    return ZoneCalibrationFit(
        zone_id=zone_id,
        x_per_z=x_per_z,
        y_per_z=y_per_z,
        observations=len(observations),
        rms_plane_residual_mm=rms,
        max_abs_plane_residual_mm=max_abs,
    )


def calibrate_geometry_profile(
    planes: Sequence[CalibrationPlane],
    *,
    name: str,
    sensor: str = "VL53L5CX",
) -> GeometryCalibrationResult:
    """Fit all 64 zones independently and return a portable calibrated profile."""

    if len(planes) < 2:
        raise CalibrationError("at least two calibration planes are required")

    zone_fits = tuple(
        solve_zone_slopes(zone_id, planes)
        for zone_id in range(geometry.ZONE_COUNT)
    )
    slopes = tuple((fit.x_per_z, fit.y_per_z) for fit in zone_fits)
    all_rms = [fit.rms_plane_residual_mm for fit in zone_fits]
    all_max = [fit.max_abs_plane_residual_mm for fit in zone_fits]

    profile = geometry.TofGeometryProfile(
        name=name,
        role="calibrated",
        sensor=sensor,
        xy_per_z=slopes,
        metadata={
            "calibration": {
                "method": CALIBRATION_METHOD,
                "plane_count": len(planes),
                "plane_labels": [plane.label for plane in planes],
                "mean_zone_rms_plane_residual_mm": sum(all_rms) / len(all_rms),
                "max_zone_rms_plane_residual_mm": max(all_rms),
                "max_abs_plane_residual_mm": max(all_max),
                "zone_fits": [
                    {
                        "zone_id": fit.zone_id,
                        "observations": fit.observations,
                        "rms_plane_residual_mm": fit.rms_plane_residual_mm,
                        "max_abs_plane_residual_mm": fit.max_abs_plane_residual_mm,
                    }
                    for fit in zone_fits
                ],
            }
        },
    )
    return GeometryCalibrationResult(profile=profile, zone_fits=zone_fits)


def expected_axial_distance_mm(
    profile: geometry.TofGeometryProfile,
    zone_id: int,
    *,
    normal_x: float,
    normal_y: float,
    normal_z: float,
    offset_mm: float,
) -> float:
    """Predict ideal sensor Z for a known plane and geometry profile.

    This is useful for synthetic regression tests and calibration-jig planning.
    """

    plane = CalibrationPlane(
        normal_x=normal_x,
        normal_y=normal_y,
        normal_z=normal_z,
        offset_mm=offset_mm,
        distances_mm=(1.0,) * geometry.ZONE_COUNT,
    )
    nx, ny, nz, offset = plane.normalised()
    vector = profile.projection_vector(zone_id)
    denominator = nx * vector.x_per_z + ny * vector.y_per_z + nz
    if denominator <= 0.0:
        raise CalibrationError(
            f"plane lies behind or parallel to zone {zone_id} projection ray"
        )
    return offset / denominator
