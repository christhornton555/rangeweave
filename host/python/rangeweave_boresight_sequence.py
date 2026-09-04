"""Fixed-plane boresight observation sequencing for Rangeweave.

This layer connects two already-validated pieces without changing capture semantics:
- stationary ToF capture reduction + plane fitting in ``tof_optical``; and
- short-baseline IMU relative rotation in ``device_body``.

A baseline stationary ToF capture defines the arbitrary reference body frame. Each
subsequent motion capture yields ``R_previous_from_current`` and each subsequent
stationary ToF capture yields a new plane normal. Relative rotations are composed as

    R_reference_from_current =
        R_reference_from_previous * R_previous_from_current

so the resulting observations can be passed directly to
``solve_fixed_plane_boresight``.
"""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Sequence

import rangeweave_extrinsics as ext
import rangeweave_plane_alignment as plane_alignment
import rangeweave_tof_calibration_capture as tof_capture


# These are boresight-workflow gates, not universal VL53L5CX specifications. They
# are deliberately generous relative to the clean fixed-plane captures observed on
# the reference rig (roughly 1-2 mm RMS and <=2 mm half-drift), while rejecting the
# physically demonstrated moving/slipped and off-target captures.
DEFAULT_MAX_PLANE_RMS_MM = 10.0
DEFAULT_MAX_PLANE_RESIDUAL_MM = 30.0
DEFAULT_MAX_HALF_DRIFT_MM = 10.0


class BoresightSequenceError(ValueError):
    """Raised when a fixed-plane boresight sequence observation is unusable."""


@dataclass(frozen=True)
class BoresightPoseQualityLimits:
    max_plane_rms_mm: float = DEFAULT_MAX_PLANE_RMS_MM
    max_plane_residual_mm: float = DEFAULT_MAX_PLANE_RESIDUAL_MM
    max_half_drift_mm: float = DEFAULT_MAX_HALF_DRIFT_MM

    def __post_init__(self):
        for name, value in (
            ("max_plane_rms_mm", self.max_plane_rms_mm),
            ("max_plane_residual_mm", self.max_plane_residual_mm),
            ("max_half_drift_mm", self.max_half_drift_mm),
        ):
            if float(value) <= 0.0:
                raise ValueError(f"{name} must be positive")


@dataclass(frozen=True)
class StationaryBoresightPose:
    label: str
    capture_path: Path
    reference_from_body: ext.Matrix3
    tof_plane: plane_alignment.PlaneAlignment
    observation: ext.FixedPlaneBoresightObservation
    tof_frame_count: int
    usable_zone_count: int
    median_mad_mm: float | None
    max_half_drift_mm: float | None


def compose_reference_from_body(
    reference_from_previous: ext.Matrix3,
    previous_from_current: ext.Matrix3,
) -> ext.Matrix3:
    """Compose one relative body motion into the baseline/reference frame."""

    return ext.matrix_multiply(reference_from_previous, previous_from_current)


def _finite_values(values: Sequence[float | None]) -> tuple[float, ...]:
    return tuple(float(value) for value in values if value is not None)


def stationary_pose_quality_errors(
    plane_fit: plane_alignment.PlaneAlignment,
    max_half_drift_mm: float | None,
    *,
    limits: BoresightPoseQualityLimits | None = None,
) -> tuple[str, ...]:
    """Return boresight-specific physical quality failures for a stationary pose.

    Structural stream/capture validity is checked separately by the generic ToF
    capture reducer. These checks answer a different question: was the sensor held
    steadily on one sufficiently planar target for this capture to be admitted to a
    fixed-plane boresight solve?
    """

    limits = limits or BoresightPoseQualityLimits()
    errors = []
    if float(plane_fit.rms_residual_mm) > float(limits.max_plane_rms_mm):
        errors.append(
            "plane RMS {:.2f} mm exceeds {:.2f} mm".format(
                plane_fit.rms_residual_mm, limits.max_plane_rms_mm
            )
        )
    if float(plane_fit.max_abs_residual_mm) > float(limits.max_plane_residual_mm):
        errors.append(
            "plane max residual {:.2f} mm exceeds {:.2f} mm".format(
                plane_fit.max_abs_residual_mm, limits.max_plane_residual_mm
            )
        )
    if max_half_drift_mm is None:
        errors.append("no finite ToF half-capture drift measurement is available")
    elif float(max_half_drift_mm) > float(limits.max_half_drift_mm):
        errors.append(
            "max half-drift {:.2f} mm exceeds {:.2f} mm".format(
                max_half_drift_mm, limits.max_half_drift_mm
            )
        )
    return tuple(errors)


def stationary_pose_from_capture(
    path: Path | str,
    reference_from_body: ext.Matrix3,
    *,
    label: str = "",
    quality_limits: BoresightPoseQualityLimits | None = None,
) -> StationaryBoresightPose:
    """Reduce and quality-gate one stationary ToF boresight capture."""

    reduced = tof_capture.analyse_calibration_capture(path)
    if not reduced.structurally_valid:
        raise BoresightSequenceError(
            "stationary ToF capture is structurally invalid: "
            + "; ".join(reduced.structural_errors)
        )

    try:
        fit = plane_alignment.fit_plane_from_distances(reduced.distances_mm)
    except (plane_alignment.PlaneAlignmentError, TypeError, ValueError) as exc:
        raise BoresightSequenceError(f"could not fit stationary ToF plane: {exc}") from exc

    mads = _finite_values(reduced.mad_mm)
    drifts = _finite_values(reduced.half_drift_mm)
    max_drift = max(drifts) if drifts else None
    quality_errors = stationary_pose_quality_errors(
        fit,
        max_drift,
        limits=quality_limits,
    )
    if quality_errors:
        raise BoresightSequenceError(
            "stationary ToF pose failed boresight quality: " + "; ".join(quality_errors)
        )

    observation = ext.FixedPlaneBoresightObservation(
        tof_plane_normal=(fit.normal_x, fit.normal_y, fit.normal_z),
        reference_from_body=reference_from_body,
    )

    median_mad = None
    if mads:
        ordered = sorted(mads)
        middle = len(ordered) // 2
        median_mad = (
            ordered[middle]
            if len(ordered) % 2
            else (ordered[middle - 1] + ordered[middle]) / 2.0
        )

    return StationaryBoresightPose(
        label=str(label),
        capture_path=Path(path),
        reference_from_body=reference_from_body,
        tof_plane=fit,
        observation=observation,
        tof_frame_count=reduced.tof_frame_count,
        usable_zone_count=reduced.usable_zone_count,
        median_mad_mm=median_mad,
        max_half_drift_mm=max_drift,
    )


def solve_if_ready(
    poses: Sequence[StationaryBoresightPose],
    *,
    max_abs_angle_deg: float = 20.0,
) -> ext.BoresightFit | None:
    """Run the fixed-plane solver once the minimum four stationary poses exist."""

    poses = tuple(poses)
    if len(poses) < 4:
        return None
    return ext.solve_fixed_plane_boresight(
        [pose.observation for pose in poses],
        max_abs_angle_deg=max_abs_angle_deg,
    )
