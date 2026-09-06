"""Rotate one Rangeweave ToF grid into ``local_reference``.

This module is the first orientation-aware geometry layer above the existing
``tof_optical`` projection.  It deliberately applies **rotation only**:

    p_tof
      -> R_body_from_tof
      -> R_reference_from_body(t_tof)
      -> p_reference

No sensor-origin translation, trajectory, odometry or SLAM is implied.  The
returned points therefore share the sensing-head origin and are suitable for
rotation-in-place replay/viewing, not yet for globally registered 3D mapping.

The ToF observation time is resolved from protocol ``mcu_ready_us`` plus the
already-selected quick-start/calibrated timing resolution.  The effective
orientation is SLERP-interpolated from the deterministic Phase 3 orientation
run using the same timing semantics as the fixed-wall validator.
"""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Sequence

import rangeweave_extrinsics as ext
import rangeweave_geometry as geometry
import rangeweave_orientation as orientation
import rangeweave_orientation_wall as orientation_wall
import rangeweave_tof_timing as tof_timing


class OrientedGeometryError(ValueError):
    """Raised when a ToF grid cannot be projected safely into local_reference."""


@dataclass(frozen=True)
class ReferenceTofFrame:
    """One producer-native 8x8 ToF grid expressed in ``local_reference``.

    ``points_reference`` preserves producer zone order and invalid-zone ``None``
    slots.  Coordinates are orientation-compensated about the common sensing-head
    origin; they do not yet include any translation of that origin.
    """

    time_s: float
    mcu_ready_us: int
    timing_mode: str
    timing_role: str
    timing_source: str
    timing_profile: str | None
    timing_offset_ms: float
    geometry_profile: str
    geometry_role: str
    quaternion_reference_from_body: orientation.Quaternion
    reference_from_body: ext.Matrix3
    reference_from_tof: ext.Matrix3
    orientation_bracket_gap_s: float
    accel_weight: float
    gravity_innovation_deg: float
    points_reference: tuple[geometry.Point3 | None, ...]

    @property
    def valid_point_count(self) -> int:
        return sum(point is not None for point in self.points_reference)


def _point_from_vector(vector: Sequence[float]) -> geometry.Point3:
    values = tuple(float(value) for value in vector)
    if len(values) != 3 or not all(math.isfinite(value) for value in values):
        raise OrientedGeometryError("transformed point must contain three finite coordinates")
    return geometry.Point3(values[0], values[1], values[2])


def project_tof_grid_to_reference(
    grid: object,
    imu_samples: Sequence[object],
    orientation_run: orientation.OrientationRun,
    rotation_body_from_tof: ext.Matrix3,
    timing_resolution: tof_timing.TimingResolution,
    *,
    geometry_profile: geometry.TofGeometryProfile | None = None,
) -> ReferenceTofFrame:
    """Project one ToF grid into ``local_reference`` using Phase 3 orientation.

    The transform is strictly rotational.  ``distance_mm`` is first projected in
    ``tof_optical`` using the supplied geometry profile (or the nominal fallback),
    then transformed by ``R_body_from_tof`` and the interpolated
    ``R_reference_from_body`` at the timing-resolved ToF observation time.

    ``imu_samples`` is required only to recover the MCU-time origin used by the
    orientation run.  This mirrors the established fixed-wall validation path and
    avoids inventing a second timestamp convention.
    """

    imu_samples = tuple(imu_samples)
    if not imu_samples:
        raise OrientedGeometryError("at least one IMU sample is required to recover the time origin")

    distances = getattr(grid, "distance_mm", None)
    if distances is None:
        raise OrientedGeometryError("ToF grid does not contain distance_mm")
    rows = int(getattr(grid, "rows"))
    cols = int(getattr(grid, "cols"))
    if rows != geometry.ZONE_ROWS or cols != geometry.ZONE_COLS:
        raise OrientedGeometryError(
            f"expected {geometry.ZONE_ROWS}x{geometry.ZONE_COLS} ToF grid, got {rows}x{cols}"
        )
    if len(distances) != geometry.ZONE_COUNT:
        raise OrientedGeometryError(
            f"expected {geometry.ZONE_COUNT} ToF distances, got {len(distances)}"
        )

    ready_us = int(getattr(grid, "mcu_ready_us"))
    offset_ms = float(timing_resolution.effective_offset_ms)
    if not math.isfinite(offset_ms):
        raise OrientedGeometryError("timing resolution offset must be finite")

    first_imu_tick = int(getattr(imu_samples[0], "lsm_tick"))
    origin_mcu_us = float(orientation_run.clock_fit.tick_to_us(first_imu_tick))
    observation_mcu_us = float(ready_us) + offset_ms * 1000.0
    time_s = (observation_mcu_us - origin_mcu_us) / 1_000_000.0

    try:
        interpolated = orientation_wall.interpolate_orientation(
            orientation_run.samples,
            time_s,
        )
        points_tof = geometry.project_distances_mm(
            distances,
            profile=geometry_profile,
        )
    except (orientation_wall.WallOrientationError, geometry.GeometryError) as exc:
        raise OrientedGeometryError(str(exc)) from exc

    reference_from_tof = ext.matrix_multiply(
        interpolated.reference_from_body,
        rotation_body_from_tof,
    )

    points_reference: list[geometry.Point3 | None] = []
    for point in points_tof:
        if point is None:
            points_reference.append(None)
            continue
        transformed = ext.matrix_vector(
            reference_from_tof,
            (point.x_mm, point.y_mm, point.z_mm),
        )
        points_reference.append(_point_from_vector(transformed))

    profile = geometry_profile or geometry.NOMINAL_ST_PROFILE
    return ReferenceTofFrame(
        time_s=time_s,
        mcu_ready_us=ready_us,
        timing_mode=timing_resolution.mode,
        timing_role=timing_resolution.role,
        timing_source=timing_resolution.source,
        timing_profile=timing_resolution.artifact_name,
        timing_offset_ms=offset_ms,
        geometry_profile=profile.name,
        geometry_role=profile.role,
        quaternion_reference_from_body=interpolated.quaternion_reference_from_body,
        reference_from_body=interpolated.reference_from_body,
        reference_from_tof=reference_from_tof,
        orientation_bracket_gap_s=interpolated.bracket_gap_s,
        accel_weight=interpolated.accel_weight,
        gravity_innovation_deg=interpolated.gravity_innovation_deg,
        points_reference=tuple(points_reference),
    )
