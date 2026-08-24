"""Reduce one stationary Rangeweave ToF capture to a robust calibration observation.

This module stays above the canonical capture/depth layer and below the known-plane
geometry solver. It does not alter raw capture semantics. Per-zone medians are used
as the calibration distances; median absolute deviation (MAD) and temporal half-drift
are retained as quality diagnostics.
"""

from __future__ import annotations

from dataclasses import dataclass
import math
from pathlib import Path
import statistics
from typing import Mapping, Sequence

import rangeweave_depth as depth
import rangeweave_geometry as geometry
import rangeweave_tof_calibration as calibration
import rangeweave_tof_calibration_workflow as workflow


DEFAULT_MIN_FRAMES = 30
DEFAULT_MIN_VALID_FRACTION = 0.90


class CalibrationCaptureError(ValueError):
    """Raised when calibration-capture reduction parameters are invalid."""


@dataclass(frozen=True)
class CalibrationCaptureObservation:
    """Robust 64-zone observation and quality evidence for one stationary capture."""

    input_path: Path
    packets_sha256: str
    tof_frame_count: int
    distance_frame_count: int
    min_valid_fraction: float
    distances_mm: tuple[float | None, ...]
    raw_median_mm: tuple[float | None, ...]
    mad_mm: tuple[float | None, ...]
    half_drift_mm: tuple[float | None, ...]
    valid_count: tuple[int, ...]
    valid_fraction: tuple[float, ...]
    health_deltas: Mapping[str, int]
    metadata_errors: tuple[str, ...]
    structural_errors: tuple[str, ...]
    warnings: tuple[str, ...]
    observed_ready_rate_hz: float | None

    @property
    def usable_zone_count(self) -> int:
        return sum(value is not None for value in self.distances_mm)

    @property
    def structurally_valid(self) -> bool:
        return not self.structural_errors


def _valid_distance(value: object) -> float | None:
    try:
        distance = float(value)
    except (TypeError, ValueError):
        return None
    if not math.isfinite(distance) or distance <= depth.DISTANCE_INVALID_MM:
        return None
    return distance


def _median(values: Sequence[float]) -> float | None:
    if not values:
        return None
    return float(statistics.median(values))


def _mad(values: Sequence[float], median_value: float | None) -> float | None:
    if not values or median_value is None:
        return None
    return float(statistics.median(abs(value - median_value) for value in values))


def reduce_capture_analysis(
    analysis: depth.DepthCaptureAnalysis,
    *,
    min_frames: int = DEFAULT_MIN_FRAMES,
    min_valid_fraction: float = DEFAULT_MIN_VALID_FRACTION,
) -> CalibrationCaptureObservation:
    """Reduce one decoded stationary capture to robust producer-native zone medians.

    A zone is passed to the geometry solver only when at least
    ``min_valid_fraction`` of distance-bearing frames contain a valid positive
    distance for that zone. Low-coverage zones become ``None`` rather than being
    filled or extrapolated.

    MAD and first-half/second-half median drift are diagnostics only in this first
    workflow version. No empirical stability threshold is imposed until physical
    calibration-board captures establish an evidence-based value.
    """

    if int(min_frames) <= 0:
        raise CalibrationCaptureError("min_frames must be greater than zero")
    min_frames = int(min_frames)
    min_valid_fraction = float(min_valid_fraction)
    if not 0.0 < min_valid_fraction <= 1.0:
        raise CalibrationCaptureError("min_valid_fraction must be in (0, 1]")

    structural_errors: list[str] = []
    warnings: list[str] = []

    if analysis.rows != geometry.ZONE_ROWS or analysis.cols != geometry.ZONE_COLS:
        structural_errors.append(
            f"expected {geometry.ZONE_ROWS}x{geometry.ZONE_COLS} ToF grid, "
            f"got {analysis.rows}x{analysis.cols}"
        )
    if analysis.layout_id != 0:
        structural_errors.append(
            f"expected producer layout_id 0, got {analysis.layout_id}"
        )

    tof_frames = tuple(analysis.tof_frames)
    distance_frames = tuple(frame for frame in tof_frames if frame.distance_mm is not None)
    if len(distance_frames) < min_frames:
        structural_errors.append(
            f"need at least {min_frames} distance-bearing ToF frames, "
            f"got {len(distance_frames)}"
        )
    if len(distance_frames) != len(tof_frames):
        warnings.append(
            f"{len(tof_frames) - len(distance_frames)} ToF frame(s) had no distance field"
        )

    decoder_bad = int(getattr(analysis.decoder, "frames_bad", 0))
    if decoder_bad:
        structural_errors.append(f"decoder reported {decoder_bad} bad frame(s)")

    semantic_errors = int(getattr(analysis.stream_stats, "semantic_errors", 0))
    if semantic_errors:
        structural_errors.append(
            f"stream reported {semantic_errors} semantic error(s)"
        )

    sequence_gaps = int(getattr(analysis.stream_stats, "sequence_gaps", 0))
    if sequence_gaps:
        structural_errors.append(
            f"stream reported {sequence_gaps} missing sequence number(s)"
        )

    metadata_errors = tuple(analysis.metadata_errors)
    if metadata_errors:
        structural_errors.extend(
            f"metadata parity: {message}" for message in metadata_errors
        )

    health_deltas = dict(analysis.stream_stats.health_deltas())
    for name, delta in sorted(health_deltas.items()):
        if int(delta) != 0:
            structural_errors.append(f"health counter {name} increased by {delta}")
    if not health_deltas:
        warnings.append("capture has no STATUS interval from which to verify health deltas")

    zone_samples: list[list[float]] = [[] for _ in range(geometry.ZONE_COUNT)]
    first_half_samples: list[list[float]] = [[] for _ in range(geometry.ZONE_COUNT)]
    second_half_samples: list[list[float]] = [[] for _ in range(geometry.ZONE_COUNT)]

    split_index = len(distance_frames) // 2
    for frame_index, frame in enumerate(distance_frames):
        values = frame.distance_mm
        if values is None:
            continue
        if len(values) != geometry.ZONE_COUNT:
            structural_errors.append(
                f"distance frame {frame_index} has {len(values)} zones, "
                f"expected {geometry.ZONE_COUNT}"
            )
            continue
        target = first_half_samples if frame_index < split_index else second_half_samples
        for zone_id, raw_value in enumerate(values):
            value = _valid_distance(raw_value)
            if value is None:
                continue
            zone_samples[zone_id].append(value)
            target[zone_id].append(value)

    raw_medians: list[float | None] = []
    calibration_distances: list[float | None] = []
    mads: list[float | None] = []
    half_drifts: list[float | None] = []
    valid_counts: list[int] = []
    valid_fractions: list[float] = []

    denominator = len(distance_frames)
    for zone_id in range(geometry.ZONE_COUNT):
        samples = zone_samples[zone_id]
        median_value = _median(samples)
        count = len(samples)
        fraction = count / denominator if denominator else 0.0
        first_median = _median(first_half_samples[zone_id])
        second_median = _median(second_half_samples[zone_id])
        drift = (
            abs(second_median - first_median)
            if first_median is not None and second_median is not None
            else None
        )

        raw_medians.append(median_value)
        calibration_distances.append(
            median_value if fraction >= min_valid_fraction else None
        )
        mads.append(_mad(samples, median_value))
        half_drifts.append(drift)
        valid_counts.append(count)
        valid_fractions.append(fraction)

    unusable = sum(value is None for value in calibration_distances)
    if unusable:
        warnings.append(
            f"{unusable} zone(s) below {100.0 * min_valid_fraction:.1f}% valid coverage; "
            "their calibration observation is None"
        )

    return CalibrationCaptureObservation(
        input_path=Path(analysis.input_path),
        packets_sha256=str(analysis.packets_sha256),
        tof_frame_count=len(tof_frames),
        distance_frame_count=len(distance_frames),
        min_valid_fraction=min_valid_fraction,
        distances_mm=tuple(calibration_distances),
        raw_median_mm=tuple(raw_medians),
        mad_mm=tuple(mads),
        half_drift_mm=tuple(half_drifts),
        valid_count=tuple(valid_counts),
        valid_fraction=tuple(valid_fractions),
        health_deltas=health_deltas,
        metadata_errors=metadata_errors,
        structural_errors=tuple(structural_errors),
        warnings=tuple(warnings),
        observed_ready_rate_hz=analysis.observed_ready_rate_hz,
    )


def analyse_calibration_capture(
    path: Path | str,
    *,
    min_frames: int = DEFAULT_MIN_FRAMES,
    min_valid_fraction: float = DEFAULT_MIN_VALID_FRACTION,
) -> CalibrationCaptureObservation:
    """Decode a canonical capture and reduce it to one calibration observation."""

    analysis = depth.analyse_capture(path)
    return reduce_capture_analysis(
        analysis,
        min_frames=min_frames,
        min_valid_fraction=min_valid_fraction,
    )


def calibration_plane_from_observation(
    pose: workflow.KnownPlanePose,
    observation: CalibrationCaptureObservation,
    *,
    label: str = "",
) -> calibration.CalibrationPlane:
    """Attach robust capture medians to a measured known-plane pose."""

    if not observation.structurally_valid:
        raise CalibrationCaptureError(
            "cannot use a structurally invalid calibration capture: "
            + "; ".join(observation.structural_errors)
        )
    return workflow.calibration_plane_from_pose(
        pose,
        observation.distances_mm,
        label=label,
    )
