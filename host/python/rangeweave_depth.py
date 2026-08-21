"""Replay-first raw depth analysis for Rangeweave captures.

This module deliberately stays standard-library only. It consumes exact Rangeweave
wire bytes through the existing StreamDecoder and computes raw 2D zone statistics.
It does not assign optical axes or project zones into 3D.
"""

from __future__ import annotations

from dataclasses import dataclass
import hashlib
import math
from pathlib import Path
from typing import Optional

import rangeweave_capture as cap
import rangeweave_protocol as rw


DISTANCE_INVALID_MM = 0
PLANE_MIN_VALID_FRACTION = 0.50


class DepthAnalysisError(ValueError):
    """Raised when a capture cannot be interpreted as one consistent raw depth grid."""


@dataclass(frozen=True)
class ZoneStatistics:
    """Per-zone population statistics.

    ``count`` contains accepted/valid samples. ``invalid_count`` contains samples
    rejected by the analysis validity mask. For distance in protocol v0.1 captures,
    a value of 0 mm is treated as an invalid/sentinel measurement.
    """

    count: tuple[int, ...]
    invalid_count: tuple[int, ...]
    mean: tuple[float, ...]
    stddev: tuple[float, ...]
    minimum: tuple[float, ...]
    maximum: tuple[float, ...]

    @property
    def total_count(self) -> tuple[int, ...]:
        return tuple(
            valid + invalid
            for valid, invalid in zip(self.count, self.invalid_count)
        )

    @property
    def invalid_percent(self) -> tuple[float, ...]:
        values = []
        for valid, invalid in zip(self.count, self.invalid_count):
            total = valid + invalid
            values.append(100.0 * invalid / total if total else float("nan"))
        return tuple(values)


@dataclass(frozen=True)
class PlaneFit:
    """Least-squares plane fitted to per-zone mean distance."""

    intercept_mm: float
    row_slope_mm: float
    column_slope_mm: float
    residual_mm: tuple[float, ...]
    rms_residual_mm: float
    max_abs_residual_mm: float
    zones_used: int
    min_valid_fraction: float


@dataclass(frozen=True)
class DepthCaptureAnalysis:
    input_path: Path
    packets_path: Path
    rows: int
    cols: int
    layout_id: int
    field_masks: tuple[int, ...]
    tof_frames: tuple[rw.TofGrid, ...]
    distance: ZoneStatistics
    reflectance: Optional[ZoneStatistics]
    mean_distance_plane: Optional[PlaneFit]
    ready_span_us: int
    observed_ready_rate_hz: Optional[float]
    read_duration_us: ZoneStatistics
    decoder: rw.StreamDecoder
    stream_stats: cap.StreamStats
    packets_bytes: int
    packets_sha256: str
    metadata_errors: tuple[str, ...]

    @property
    def zones(self) -> int:
        return self.rows * self.cols

    def frame(self, index: int = -1) -> rw.TofGrid:
        try:
            return self.tof_frames[index]
        except IndexError as exc:
            raise DepthAnalysisError(
                "ToF frame index {} outside capture range {}..{}".format(
                    index, -len(self.tof_frames), len(self.tof_frames) - 1
                )
            ) from exc


class _OnlineVectorStats:
    """Per-element Welford statistics for fixed-width vectors."""

    def __init__(self, size: int) -> None:
        self.size = size
        self.count = [0] * size
        self.invalid_count = [0] * size
        self.mean = [0.0] * size
        self.m2 = [0.0] * size
        self.minimum = [math.inf] * size
        self.maximum = [-math.inf] * size

    def update(self, values, valid_mask=None) -> None:
        if len(values) != self.size:
            raise DepthAnalysisError(
                "zone-vector length changed from {} to {}".format(self.size, len(values))
            )
        if valid_mask is not None and len(valid_mask) != self.size:
            raise DepthAnalysisError(
                "validity-mask length changed from {} to {}".format(
                    self.size, len(valid_mask)
                )
            )

        for index, raw_value in enumerate(values):
            if valid_mask is not None and not valid_mask[index]:
                self.invalid_count[index] += 1
                continue

            value = float(raw_value)
            count = self.count[index] + 1
            delta = value - self.mean[index]
            mean = self.mean[index] + delta / count
            delta2 = value - mean
            self.count[index] = count
            self.mean[index] = mean
            self.m2[index] += delta * delta2
            if value < self.minimum[index]:
                self.minimum[index] = value
            if value > self.maximum[index]:
                self.maximum[index] = value

    def finish(self) -> ZoneStatistics:
        means = []
        stddevs = []
        minimums = []
        maximums = []
        for index, count in enumerate(self.count):
            if count == 0:
                means.append(float("nan"))
                stddevs.append(float("nan"))
                minimums.append(float("nan"))
                maximums.append(float("nan"))
            else:
                means.append(self.mean[index])
                # Population standard deviation: the capture itself is the population
                # being described, not an estimator of an unknown larger sample set.
                stddevs.append(math.sqrt(self.m2[index] / count))
                minimums.append(self.minimum[index])
                maximums.append(self.maximum[index])
        return ZoneStatistics(
            tuple(self.count),
            tuple(self.invalid_count),
            tuple(means),
            tuple(stddevs),
            tuple(minimums),
            tuple(maximums),
        )


def resolve_packets_path(path: Path | str) -> tuple[Path, Optional[Path]]:
    """Return (packets.bin path, session directory if the input is a session)."""
    input_path = Path(path)
    if input_path.is_dir():
        packets_path = input_path / cap.PACKETS_FILENAME
        if not packets_path.is_file():
            raise DepthAnalysisError("capture directory has no packets.bin")
        return packets_path, input_path
    if not input_path.is_file():
        raise DepthAnalysisError("capture path does not exist: {}".format(input_path))
    return input_path, None


def _solve_3x3(matrix, vector) -> Optional[tuple[float, float, float]]:
    """Solve one 3x3 linear system with partial-pivot Gaussian elimination."""
    augmented = [
        [float(matrix[row][col]) for col in range(3)] + [float(vector[row])]
        for row in range(3)
    ]

    for pivot_col in range(3):
        pivot_row = max(
            range(pivot_col, 3),
            key=lambda row: abs(augmented[row][pivot_col]),
        )
        if abs(augmented[pivot_row][pivot_col]) < 1e-12:
            return None

        if pivot_row != pivot_col:
            augmented[pivot_col], augmented[pivot_row] = (
                augmented[pivot_row],
                augmented[pivot_col],
            )

        pivot = augmented[pivot_col][pivot_col]
        for col in range(pivot_col, 4):
            augmented[pivot_col][col] /= pivot

        for row in range(3):
            if row == pivot_col:
                continue
            factor = augmented[row][pivot_col]
            if factor == 0.0:
                continue
            for col in range(pivot_col, 4):
                augmented[row][col] -= factor * augmented[pivot_col][col]

    return (
        augmented[0][3],
        augmented[1][3],
        augmented[2][3],
    )


def fit_mean_distance_plane(
    distance: ZoneStatistics,
    rows: int,
    cols: int,
    *,
    min_valid_fraction: float = PLANE_MIN_VALID_FRACTION,
) -> Optional[PlaneFit]:
    """Fit ``distance_mm = a + b*row + c*column`` to per-zone means.

    This is a spatial diagnostic, not a claim that the observed scene is planar.
    Zones with insufficient valid distance coverage are excluded from the fit.
    """
    if len(distance.mean) != rows * cols:
        raise DepthAnalysisError("distance statistic count does not match rows*cols")
    if not 0.0 <= min_valid_fraction <= 1.0:
        raise ValueError("min_valid_fraction must be between 0 and 1")

    usable = []
    for index, mean in enumerate(distance.mean):
        valid = distance.count[index]
        invalid = distance.invalid_count[index]
        total = valid + invalid
        valid_fraction = valid / total if total else 0.0
        if (
            valid > 0
            and math.isfinite(mean)
            and valid_fraction >= min_valid_fraction
        ):
            row, col = divmod(index, cols)
            usable.append((float(row), float(col), float(mean)))

    if len(usable) < 3:
        return None

    n = float(len(usable))
    sum_r = sum(row for row, _, _ in usable)
    sum_c = sum(col for _, col, _ in usable)
    sum_rr = sum(row * row for row, _, _ in usable)
    sum_cc = sum(col * col for _, col, _ in usable)
    sum_rc = sum(row * col for row, col, _ in usable)
    sum_z = sum(z for _, _, z in usable)
    sum_rz = sum(row * z for row, _, z in usable)
    sum_cz = sum(col * z for _, col, z in usable)

    coefficients = _solve_3x3(
        (
            (n, sum_r, sum_c),
            (sum_r, sum_rr, sum_rc),
            (sum_c, sum_rc, sum_cc),
        ),
        (sum_z, sum_rz, sum_cz),
    )
    if coefficients is None:
        return None

    intercept, row_slope, column_slope = coefficients
    residuals = [float("nan")] * (rows * cols)
    squared_sum = 0.0
    max_abs = 0.0

    for row, col, observed in usable:
        predicted = intercept + row_slope * row + column_slope * col
        residual = observed - predicted
        index = int(row) * cols + int(col)
        residuals[index] = residual
        squared_sum += residual * residual
        max_abs = max(max_abs, abs(residual))

    return PlaneFit(
        intercept_mm=intercept,
        row_slope_mm=row_slope,
        column_slope_mm=column_slope,
        residual_mm=tuple(residuals),
        rms_residual_mm=math.sqrt(squared_sum / len(usable)),
        max_abs_residual_mm=max_abs,
        zones_used=len(usable),
        min_valid_fraction=min_valid_fraction,
    )


def analyse_capture(path: Path | str, *, chunk_size: int = 4096) -> DepthCaptureAnalysis:
    """Decode one capture and compute raw per-zone distance/reflectance statistics."""
    input_path = Path(path)
    packets_path, session_dir = resolve_packets_path(input_path)

    decoder = rw.StreamDecoder()
    stream_stats = cap.StreamStats()
    digest = hashlib.sha256()
    packets_bytes = 0

    tof_frames: list[rw.TofGrid] = []
    rows = None
    cols = None
    layout_id = None
    field_masks: set[int] = set()
    distance_stats = None
    reflectance_stats = None
    read_duration_stats = _OnlineVectorStats(1)

    with packets_path.open("rb") as source:
        while True:
            chunk = source.read(chunk_size)
            if not chunk:
                break
            packets_bytes += len(chunk)
            digest.update(chunk)
            for frame in decoder.feed(chunk):
                stream_stats.consume(frame)
                if frame.record_type != rw.RECORD_TOF_GRID:
                    continue
                try:
                    record = rw.decode_record(frame)
                except rw.ProtocolError:
                    # StreamStats already counted the semantic failure.
                    continue

                if rows is None:
                    rows = record.rows
                    cols = record.cols
                    layout_id = record.layout_id
                    zones = rows * cols
                    distance_stats = _OnlineVectorStats(zones)
                    reflectance_stats = _OnlineVectorStats(zones)
                elif (
                    record.rows != rows
                    or record.cols != cols
                    or record.layout_id != layout_id
                ):
                    raise DepthAnalysisError(
                        "ToF grid geometry/layout changed within capture: "
                        "{}x{} layout {} -> {}x{} layout {}".format(
                            rows,
                            cols,
                            layout_id,
                            record.rows,
                            record.cols,
                            record.layout_id,
                        )
                    )

                field_masks.add(record.field_mask)
                tof_frames.append(record)
                read_duration_stats.update(
                    [record.mcu_read_complete_us - record.mcu_ready_us]
                )

                valid_distance = None
                if record.distance_mm is not None:
                    valid_distance = tuple(
                        int(value) > DISTANCE_INVALID_MM
                        for value in record.distance_mm
                    )
                    distance_stats.update(record.distance_mm, valid_distance)

                if record.reflectance_percent is not None:
                    # When a matching distance array exists, exclude reflectance
                    # samples for invalid distance returns as well.
                    reflectance_stats.update(
                        record.reflectance_percent,
                        valid_distance,
                    )

    if not tof_frames or rows is None or cols is None or layout_id is None:
        raise DepthAnalysisError("capture contains no decodable TOF_GRID records")
    if distance_stats is None or not any(distance_stats.count):
        raise DepthAnalysisError("capture contains no valid TOF_GRID distance samples")

    ready_span_us = tof_frames[-1].mcu_ready_us - tof_frames[0].mcu_ready_us
    observed_ready_rate_hz = None
    if len(tof_frames) >= 2 and ready_span_us > 0:
        observed_ready_rate_hz = (
            (len(tof_frames) - 1) * 1_000_000.0 / ready_span_us
        )

    distance_result = distance_stats.finish()
    reflectance_result = None
    if reflectance_stats is not None and any(reflectance_stats.count):
        reflectance_result = reflectance_stats.finish()

    packets_sha256 = digest.hexdigest()
    metadata_errors: list[str] = []
    if session_dir is not None:
        try:
            metadata = cap.load_metadata(session_dir)
        except (OSError, ValueError) as exc:
            metadata_errors.append("could not read metadata.json: {}".format(exc))
        else:
            metadata_errors.extend(
                cap.metadata_parity_errors(
                    metadata,
                    decoder=decoder,
                    stats=stream_stats,
                    packets_bytes=packets_bytes,
                    packets_sha256=packets_sha256,
                )
            )

    return DepthCaptureAnalysis(
        input_path=input_path,
        packets_path=packets_path,
        rows=rows,
        cols=cols,
        layout_id=layout_id,
        field_masks=tuple(sorted(field_masks)),
        tof_frames=tuple(tof_frames),
        distance=distance_result,
        reflectance=reflectance_result,
        mean_distance_plane=fit_mean_distance_plane(
            distance_result,
            rows,
            cols,
        ),
        ready_span_us=ready_span_us,
        observed_ready_rate_hz=observed_ready_rate_hz,
        read_duration_us=read_duration_stats.finish(),
        decoder=decoder,
        stream_stats=stream_stats,
        packets_bytes=packets_bytes,
        packets_sha256=packets_sha256,
        metadata_errors=tuple(metadata_errors),
    )


def as_rows(values, rows: int, cols: int) -> list[list[float]]:
    """Reshape producer-native flat zone values without changing their ordering."""
    if len(values) != rows * cols:
        raise DepthAnalysisError("grid value count does not match rows*cols")
    return [list(values[start:start + cols]) for start in range(0, len(values), cols)]
