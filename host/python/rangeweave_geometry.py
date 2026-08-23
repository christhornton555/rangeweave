"""VL53L5CX 8x8 zone geometry in the Rangeweave ``tof_optical`` frame.

Protocol/capture data remain in producer-native ZoneID order.  This module is the
first geometry layer above that lossless representation.

Important: VL53L5CX ``distance_mm`` is an axial/perpendicular distance.  It is
therefore the Z coordinate in ``tof_optical``; it must not be multiplied by a
unit ray as though it were slant range.

The built-in ST-derived lookup table is a nominal fallback profile for systems
that do not yet have a per-device optical calibration.  Rangeweave does not
assume that every physical sensor has this exact lattice, or that a calibrated
lattice must be symmetric or bow in any particular direction.
"""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Iterable, Sequence


ZONE_ROWS = 8
ZONE_COLS = 8
ZONE_COUNT = ZONE_ROWS * ZONE_COLS

GEOMETRY_MODEL = "vl53l5cx-st-plane-algo-2022-corrected-yaw"
GEOMETRY_PROFILE_ROLE = "nominal-fallback"


class GeometryError(ValueError):
    """Raised when a zone or distance cannot be projected."""


@dataclass(frozen=True)
class ProjectionVector:
    """Zone direction expressed per unit axial Z: ``(x/z, y/z, 1)``."""

    x_per_z: float
    y_per_z: float
    z_per_z: float = 1.0

    def unit_ray(self) -> tuple[float, float, float]:
        length = math.sqrt(
            self.x_per_z * self.x_per_z
            + self.y_per_z * self.y_per_z
            + self.z_per_z * self.z_per_z
        )
        return (
            self.x_per_z / length,
            self.y_per_z / length,
            self.z_per_z / length,
        )


@dataclass(frozen=True)
class Point3:
    x_mm: float
    y_mm: float
    z_mm: float


# Per-zone (X/Z, Y/Z) slopes in producer-native ZoneID order 0..63.
#
# NOMINAL FALLBACK ONLY: these values are derived from the VL53L5CX pitch/yaw
# lookup table published by ST for its plane/XYZ example.  They provide useful
# geometry before a builder has calibrated their own sensor, but they are not a
# Rangeweave requirement and are not treated as per-device ground truth.  A
# calibrated profile may legitimately be asymmetric, bow inward, bow outward,
# or otherwise differ zone-by-zone from this table.
#
# The source table's duplicated yaw entry at zone 48 is corrected from 203.20
# degrees to 215.40 degrees, restoring the expected symmetry of this *nominal*
# ST profile.  The ST example is expressed while viewing the device from the
# lens side; Rangeweave's frozen tof_optical +X points scene-right when looking
# forward from behind the sensor, so the ST X component is mirrored here.
#
# Keeping the derived slopes rather than ST's angular naming also makes the
# distance contract explicit: distance_mm is Z, and X/Y scale linearly with Z.
_XY_PER_Z: tuple[tuple[float, float], ...] = (
    (0.362623824, 0.362623824), (0.251878622, 0.354427653), (0.148073029, 0.345480261), (0.048473905, 0.339321665), (-0.048473905, 0.339321665), (-0.148073029, 0.345480261), (-0.251878622, 0.354427653), (-0.362623824, 0.362623824),
    (0.354427653, 0.251878622), (0.246102263, 0.246102263), (0.137362605, 0.228971701), (0.043478157, 0.217389451), (-0.043478157, 0.217389451), (-0.137362605, 0.228971701), (-0.246102263, 0.246102263), (-0.354427653, 0.251878622),
    (0.345480261, 0.148073029), (0.228971701, 0.137362605), (0.148366419, 0.148366419), (0.045830581, 0.137371444), (-0.045830581, 0.137371444), (-0.148366419, 0.148366419), (-0.228971701, 0.137362605), (-0.345480261, 0.148073029),
    (0.339321665, 0.048473905), (0.217389451, 0.043478157), (0.137371444, 0.045830581), (0.049445723, 0.049445723), (-0.049445723, 0.049445723), (-0.137371444, 0.045830581), (-0.217389451, 0.043478157), (-0.339321665, 0.048473905),
    (0.339321665, -0.048473905), (0.217389451, -0.043478157), (0.137371444, -0.045830581), (0.049445723, -0.049445723), (-0.049445723, -0.049445723), (-0.137371444, -0.045830581), (-0.217389451, -0.043478157), (-0.339321665, -0.048473905),
    (0.345480261, -0.148073029), (0.228971701, -0.137362605), (0.148366419, -0.148366419), (0.045830581, -0.137371444), (-0.045830581, -0.137371444), (-0.148366419, -0.148366419), (-0.228971701, -0.137362605), (-0.345480261, -0.148073029),
    (0.354427653, -0.251878622), (0.246102263, -0.246102263), (0.137362605, -0.228971701), (0.043478157, -0.217389451), (-0.043478157, -0.217389451), (-0.137362605, -0.228971701), (-0.246102263, -0.246102263), (-0.354427653, -0.251878622),
    (0.362623824, -0.362623824), (0.251878622, -0.354427653), (0.148073029, -0.345480261), (0.048473905, -0.339321665), (-0.048473905, -0.339321665), (-0.148073029, -0.345480261), (-0.251878622, -0.354427653), (-0.362623824, -0.362623824),
)


def _check_zone(zone_id: int) -> int:
    zone_id = int(zone_id)
    if zone_id < 0 or zone_id >= ZONE_COUNT:
        raise GeometryError(f"zone_id must be in 0..{ZONE_COUNT - 1}")
    return zone_id


def producer_row_col(zone_id: int) -> tuple[int, int]:
    zone_id = _check_zone(zone_id)
    return divmod(zone_id, ZONE_COLS)


def physical_row_col(zone_id: int) -> tuple[int, int]:
    """Return the experimentally validated upright physical grid coordinates."""
    row, col = producer_row_col(zone_id)
    return ZONE_ROWS - 1 - row, ZONE_COLS - 1 - col


def projection_vector(zone_id: int) -> ProjectionVector:
    zone_id = _check_zone(zone_id)
    x_per_z, y_per_z = _XY_PER_Z[zone_id]
    return ProjectionVector(x_per_z=x_per_z, y_per_z=y_per_z)


def unit_ray(zone_id: int) -> tuple[float, float, float]:
    """Return a normalized geometric ray; do not multiply distance_mm by it."""
    return projection_vector(zone_id).unit_ray()


def project_axial_distance_mm(zone_id: int, distance_mm: float) -> Point3:
    """Project one valid VL53L5CX axial distance into ``tof_optical`` XYZ."""
    distance = float(distance_mm)
    if not math.isfinite(distance) or distance <= 0.0:
        raise GeometryError("distance_mm must be finite and greater than zero")
    vector = projection_vector(zone_id)
    return Point3(
        x_mm=vector.x_per_z * distance,
        y_mm=vector.y_per_z * distance,
        z_mm=distance,
    )


def project_distances_mm(distances_mm: Sequence[float]) -> tuple[Point3 | None, ...]:
    """Project a producer-native 8x8 distance tuple, preserving zone positions.

    Values <= 0 or non-finite values are returned as ``None``.  This matches the
    current host-analysis interpretation without changing raw protocol semantics.
    """
    if len(distances_mm) != ZONE_COUNT:
        raise GeometryError(f"expected {ZONE_COUNT} distances, got {len(distances_mm)}")

    points: list[Point3 | None] = []
    for zone_id, raw_distance in enumerate(distances_mm):
        distance = float(raw_distance)
        if not math.isfinite(distance) or distance <= 0.0:
            points.append(None)
        else:
            points.append(project_axial_distance_mm(zone_id, distance))
    return tuple(points)
