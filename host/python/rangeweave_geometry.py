"""VL53L5CX 8x8 zone geometry in the Rangeweave ``tof_optical`` frame.

Protocol/capture data remain in producer-native ZoneID order. This module is the
first geometry layer above that lossless representation.

VL53L5CX ``distance_mm`` is an axial/perpendicular distance, so it is the Z
coordinate in ``tof_optical``. The built-in ST-derived lookup table is only a
nominal fallback. Calibrated profiles may differ independently in every zone.
"""

from __future__ import annotations

from dataclasses import dataclass, field
import json
import math
from pathlib import Path
from typing import Any, Mapping, Sequence


ZONE_ROWS = 8
ZONE_COLS = 8
ZONE_COUNT = ZONE_ROWS * ZONE_COLS

PROFILE_SCHEMA = "rangeweave.tof-geometry-profile"
PROFILE_SCHEMA_VERSION = 1
TOF_FRAME = "tof_optical"
DISTANCE_CONTRACT = "axial-z-mm"


class GeometryError(ValueError):
    """Raised when a zone, profile or distance cannot be projected."""


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


@dataclass(frozen=True)
class TofGeometryProfile:
    """Portable per-zone projection slopes for one 8x8 ToF geometry profile."""

    name: str
    role: str
    xy_per_z: tuple[tuple[float, float], ...]
    sensor: str = "VL53L5CX"
    rows: int = ZONE_ROWS
    cols: int = ZONE_COLS
    layout_id: int = 0
    frame: str = TOF_FRAME
    distance_contract: str = DISTANCE_CONTRACT
    metadata: Mapping[str, Any] = field(default_factory=dict)

    def __post_init__(self) -> None:
        if not self.name or not self.role:
            raise GeometryError("geometry profile name and role must be non-empty")
        if self.rows != ZONE_ROWS or self.cols != ZONE_COLS:
            raise GeometryError(
                f"current Rangeweave geometry expects {ZONE_ROWS}x{ZONE_COLS} profiles"
            )
        if self.layout_id != 0:
            raise GeometryError("current Rangeweave geometry expects producer layout_id 0")
        if self.frame != TOF_FRAME:
            raise GeometryError(f"geometry profile frame must be {TOF_FRAME!r}")
        if self.distance_contract != DISTANCE_CONTRACT:
            raise GeometryError(
                f"geometry profile distance_contract must be {DISTANCE_CONTRACT!r}"
            )
        if len(self.xy_per_z) != ZONE_COUNT:
            raise GeometryError(
                f"geometry profile must contain {ZONE_COUNT} zone slopes"
            )
        normalised = []
        for zone_id, pair in enumerate(self.xy_per_z):
            if len(pair) != 2:
                raise GeometryError(f"zone {zone_id} slope must contain X/Z and Y/Z")
            x_per_z = float(pair[0])
            y_per_z = float(pair[1])
            if not math.isfinite(x_per_z) or not math.isfinite(y_per_z):
                raise GeometryError(f"zone {zone_id} slopes must be finite")
            normalised.append((x_per_z, y_per_z))
        object.__setattr__(self, "xy_per_z", tuple(normalised))
        object.__setattr__(self, "metadata", dict(self.metadata))

    def projection_vector(self, zone_id: int) -> ProjectionVector:
        zone_id = _check_zone(zone_id)
        x_per_z, y_per_z = self.xy_per_z[zone_id]
        return ProjectionVector(x_per_z=x_per_z, y_per_z=y_per_z)

    def to_dict(self) -> dict[str, Any]:
        return {
            "schema": PROFILE_SCHEMA,
            "schema_version": PROFILE_SCHEMA_VERSION,
            "name": self.name,
            "role": self.role,
            "sensor": self.sensor,
            "rows": self.rows,
            "cols": self.cols,
            "layout_id": self.layout_id,
            "frame": self.frame,
            "distance_contract": self.distance_contract,
            "zones": [
                {
                    "zone_id": zone_id,
                    "x_per_z": x_per_z,
                    "y_per_z": y_per_z,
                }
                for zone_id, (x_per_z, y_per_z) in enumerate(self.xy_per_z)
            ],
            "metadata": dict(self.metadata),
        }

    @classmethod
    def from_dict(cls, document: Mapping[str, Any]) -> "TofGeometryProfile":
        if document.get("schema") != PROFILE_SCHEMA:
            raise GeometryError(
                f"unexpected geometry profile schema {document.get('schema')!r}"
            )
        if document.get("schema_version") != PROFILE_SCHEMA_VERSION:
            raise GeometryError(
                "unsupported geometry profile schema_version "
                f"{document.get('schema_version')!r}"
            )
        zones = document.get("zones")
        if not isinstance(zones, list) or len(zones) != ZONE_COUNT:
            raise GeometryError(
                f"geometry profile zones must contain {ZONE_COUNT} entries"
            )

        slopes: list[tuple[float, float] | None] = [None] * ZONE_COUNT
        for entry in zones:
            if not isinstance(entry, Mapping):
                raise GeometryError("geometry profile zone entries must be objects")
            try:
                zone_id = int(entry["zone_id"])
                x_per_z = float(entry["x_per_z"])
                y_per_z = float(entry["y_per_z"])
            except (KeyError, TypeError, ValueError) as exc:
                raise GeometryError("invalid geometry profile zone entry") from exc
            _check_zone(zone_id)
            if slopes[zone_id] is not None:
                raise GeometryError(f"duplicate geometry profile zone_id {zone_id}")
            slopes[zone_id] = (x_per_z, y_per_z)

        if any(pair is None for pair in slopes):
            raise GeometryError("geometry profile is missing one or more zone IDs")

        metadata = document.get("metadata", {})
        if not isinstance(metadata, Mapping):
            raise GeometryError("geometry profile metadata must be an object")

        try:
            return cls(
                name=str(document["name"]),
                role=str(document["role"]),
                sensor=str(document.get("sensor", "VL53L5CX")),
                rows=int(document["rows"]),
                cols=int(document["cols"]),
                layout_id=int(document["layout_id"]),
                frame=str(document["frame"]),
                distance_contract=str(document["distance_contract"]),
                xy_per_z=tuple(pair for pair in slopes if pair is not None),
                metadata=dict(metadata),
            )
        except GeometryError:
            raise
        except (KeyError, TypeError, ValueError) as exc:
            raise GeometryError("invalid geometry profile header") from exc


# Per-zone (X/Z, Y/Z) slopes in producer-native ZoneID order 0..63.
# NOMINAL FALLBACK ONLY: calibrated profiles are free to disagree zone-by-zone.
_NOMINAL_ST_XY_PER_Z: tuple[tuple[float, float], ...] = (
    (0.362623824, 0.362623824), (0.251878622, 0.354427653), (0.148073029, 0.345480261), (0.048473905, 0.339321665), (-0.048473905, 0.339321665), (-0.148073029, 0.345480261), (-0.251878622, 0.354427653), (-0.362623824, 0.362623824),
    (0.354427653, 0.251878622), (0.246102263, 0.246102263), (0.137362605, 0.228971701), (0.043478157, 0.217389451), (-0.043478157, 0.217389451), (-0.137362605, 0.228971701), (-0.246102263, 0.246102263), (-0.354427653, 0.251878622),
    (0.345480261, 0.148073029), (0.228971701, 0.137362605), (0.148366419, 0.148366419), (0.045830581, 0.137371444), (-0.045830581, 0.137371444), (-0.148366419, 0.148366419), (-0.228971701, 0.137362605), (-0.345480261, 0.148073029),
    (0.339321665, 0.048473905), (0.217389451, 0.043478157), (0.137371444, 0.045830581), (0.049445723, 0.049445723), (-0.049445723, 0.049445723), (-0.137371444, 0.045830581), (-0.217389451, 0.043478157), (-0.339321665, 0.048473905),
    (0.339321665, -0.048473905), (0.217389451, -0.043478157), (0.137371444, -0.045830581), (0.049445723, -0.049445723), (-0.049445723, -0.049445723), (-0.137371444, -0.045830581), (-0.217389451, -0.043478157), (-0.339321665, -0.048473905),
    (0.345480261, -0.148073029), (0.228971701, -0.137362605), (0.148366419, -0.148366419), (0.045830581, -0.137371444), (-0.045830581, -0.137371444), (-0.148366419, -0.148366419), (-0.228971701, -0.137362605), (-0.345480261, -0.148073029),
    (0.354427653, -0.251878622), (0.246102263, -0.246102263), (0.137362605, -0.228971701), (0.043478157, -0.217389451), (-0.043478157, -0.217389451), (-0.137362605, -0.228971701), (-0.246102263, -0.246102263), (-0.354427653, -0.251878622),
    (0.362623824, -0.362623824), (0.251878622, -0.354427653), (0.148073029, -0.345480261), (0.048473905, -0.339321665), (-0.048473905, -0.339321665), (-0.148073029, -0.345480261), (-0.251878622, -0.354427653), (-0.362623824, -0.362623824),
)

NOMINAL_ST_PROFILE = TofGeometryProfile(
    name="vl53l5cx-st-plane-algo-2022-corrected-yaw",
    role="nominal-fallback",
    xy_per_z=_NOMINAL_ST_XY_PER_Z,
    metadata={
        "source": "ST plane/XYZ example",
        "note": "zone 48 yaw copy error corrected to the symmetry-consistent value",
    },
)

# Compatibility aliases retained for existing host output/tests.
GEOMETRY_MODEL = NOMINAL_ST_PROFILE.name
GEOMETRY_PROFILE_ROLE = NOMINAL_ST_PROFILE.role


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


def projection_vector(
    zone_id: int,
    profile: TofGeometryProfile | None = None,
) -> ProjectionVector:
    return (profile or NOMINAL_ST_PROFILE).projection_vector(zone_id)


def unit_ray(
    zone_id: int,
    profile: TofGeometryProfile | None = None,
) -> tuple[float, float, float]:
    """Return a normalized geometric ray; do not multiply distance_mm by it."""
    return projection_vector(zone_id, profile).unit_ray()


def project_axial_distance_mm(
    zone_id: int,
    distance_mm: float,
    profile: TofGeometryProfile | None = None,
) -> Point3:
    """Project one valid VL53L5CX axial distance into ``tof_optical`` XYZ."""
    distance = float(distance_mm)
    if not math.isfinite(distance) or distance <= 0.0:
        raise GeometryError("distance_mm must be finite and greater than zero")
    vector = projection_vector(zone_id, profile)
    return Point3(
        x_mm=vector.x_per_z * distance,
        y_mm=vector.y_per_z * distance,
        z_mm=distance,
    )


def project_distances_mm(
    distances_mm: Sequence[float],
    profile: TofGeometryProfile | None = None,
) -> tuple[Point3 | None, ...]:
    """Project a producer-native 8x8 distance tuple, preserving zone positions."""
    if len(distances_mm) != ZONE_COUNT:
        raise GeometryError(f"expected {ZONE_COUNT} distances, got {len(distances_mm)}")

    points: list[Point3 | None] = []
    for zone_id, raw_distance in enumerate(distances_mm):
        distance = float(raw_distance)
        if not math.isfinite(distance) or distance <= 0.0:
            points.append(None)
        else:
            points.append(project_axial_distance_mm(zone_id, distance, profile))
    return tuple(points)


def save_geometry_profile(profile: TofGeometryProfile, path: Path | str) -> Path:
    output = Path(path)
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(
        json.dumps(profile.to_dict(), indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )
    return output


def load_geometry_profile(path: Path | str) -> TofGeometryProfile:
    input_path = Path(path)
    try:
        document = json.loads(input_path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as exc:
        raise GeometryError(f"could not read geometry profile {input_path}: {exc}") from exc
    if not isinstance(document, Mapping):
        raise GeometryError("geometry profile root must be an object")
    return TofGeometryProfile.from_dict(document)
