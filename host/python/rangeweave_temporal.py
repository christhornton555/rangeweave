"""Timestamp helpers for Rangeweave temporal depth visualization.

This module is standard-library only. It contains no plotting or video dependencies so
playback/export timing semantics can be regression-tested independently of Matplotlib.
"""

from __future__ import annotations

from bisect import bisect_right
import math
import statistics


class TemporalViewError(ValueError):
    """Raised when ToF timestamps cannot define a sensible playback timeline."""


def relative_ready_times_s(frames) -> tuple[float, ...]:
    """Return ToF MCU-ready timestamps relative to the first frame, in seconds."""
    if not frames:
        raise TemporalViewError("capture contains no ToF frames")

    origin = int(frames[0].mcu_ready_us)
    previous = origin
    result = []

    for index, frame in enumerate(frames):
        current = int(frame.mcu_ready_us)
        if current < previous:
            raise TemporalViewError(
                "ToF mcu_ready_us moved backwards at frame {}: {} -> {}".format(
                    index, previous, current
                )
            )
        result.append((current - origin) / 1_000_000.0)
        previous = current

    return tuple(result)


def median_period_s(frames) -> float | None:
    """Return the median positive ready-observation interval, in seconds."""
    times = relative_ready_times_s(frames)
    intervals = [
        later - earlier
        for earlier, later in zip(times, times[1:])
        if later > earlier
    ]
    if not intervals:
        return None
    return float(statistics.median(intervals))


def suggested_export_fps(frames) -> float:
    """Choose a practical constant video FPS from the recorded ToF cadence."""
    period = median_period_s(frames)
    if period is None or period <= 0.0:
        return 15.0

    rate = 1.0 / period
    nearest_integer = round(rate)
    if nearest_integer > 0 and abs(rate - nearest_integer) <= 0.10:
        return float(nearest_integer)
    return round(rate, 3)


def cfr_source_indices(frames, fps: float) -> tuple[int, ...]:
    """Map a timestamped ToF sequence onto a constant-frame-rate video timeline.

    Each output time uses the most recent source frame at or before that instant.
    Therefore a real acquisition gap becomes a held frame in the export instead of
    silently compressing time.
    """
    if not math.isfinite(fps) or fps <= 0.0:
        raise TemporalViewError("export fps must be greater than zero")

    times = relative_ready_times_s(frames)
    if len(times) == 1:
        return (0,)

    period = median_period_s(frames)
    if period is None:
        period = 1.0 / fps

    duration = times[-1] + period
    output_count = max(1, int(math.ceil(duration * fps)))

    indices = []
    for output_index in range(output_count):
        output_time = output_index / fps
        source_index = bisect_right(times, output_time) - 1
        indices.append(max(0, min(source_index, len(times) - 1)))

    return tuple(indices)
