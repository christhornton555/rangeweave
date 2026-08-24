"""Inspect one stationary Rangeweave capture as a ToF calibration observation."""

from __future__ import annotations

import argparse
import math
from pathlib import Path
import statistics

import rangeweave_depth as depth
import rangeweave_tof_calibration_capture as capture_cal


def _finite(values):
    return [float(value) for value in values if value is not None and math.isfinite(float(value))]


def _median(values):
    finite = _finite(values)
    return statistics.median(finite) if finite else None


def _maximum(values):
    finite = _finite(values)
    return max(finite) if finite else None


def _format_metric(value, suffix=" mm"):
    return "n/a" if value is None else f"{value:.2f}{suffix}"


def _format_grid(values, *, decimals=1):
    lines = []
    for row in range(8):
        cells = []
        for value in values[row * 8:(row + 1) * 8]:
            if value is None or not math.isfinite(float(value)):
                cells.append("    --")
            else:
                cells.append(f"{float(value):6.{decimals}f}")
        lines.append(" ".join(cells))
    return "\n".join(lines)


def print_summary(observation, *, show_grids=False):
    valid_fraction_percent = [100.0 * value for value in observation.valid_fraction]
    health_nonzero = {
        name: delta
        for name, delta in observation.health_deltas.items()
        if int(delta) != 0
    }

    print("Rangeweave ToF calibration-capture inspection")
    print(f"  capture:             {observation.input_path}")
    print(f"  packets sha256:      {observation.packets_sha256}")
    print(f"  ToF frames:          {observation.tof_frame_count}")
    print(f"  distance frames:     {observation.distance_frame_count}")
    if observation.observed_ready_rate_hz is not None:
        print(f"  observed ToF rate:   {observation.observed_ready_rate_hz:.3f} Hz")
    else:
        print("  observed ToF rate:   n/a")
    print(
        f"  usable zones:        {observation.usable_zone_count} / 64 "
        f"(>= {100.0 * observation.min_valid_fraction:.1f}% valid)"
    )
    print(
        "  valid coverage:      min {:.1f}%, median {:.1f}%".format(
            min(valid_fraction_percent) if valid_fraction_percent else 0.0,
            statistics.median(valid_fraction_percent) if valid_fraction_percent else 0.0,
        )
    )
    print(
        "  zone MAD:            median {}, max {}".format(
            _format_metric(_median(observation.mad_mm)),
            _format_metric(_maximum(observation.mad_mm)),
        )
    )
    print(
        "  half-capture drift:  median {}, max {}".format(
            _format_metric(_median(observation.half_drift_mm)),
            _format_metric(_maximum(observation.half_drift_mm)),
        )
    )
    print(
        "  stream/metadata:     {}".format(
            "PASS" if observation.structurally_valid else "FAIL"
        )
    )
    print(
        "  health deltas:       {}".format(
            "all zero"
            if observation.health_deltas and not health_nonzero
            else (str(health_nonzero) if health_nonzero else "not available")
        )
    )

    for warning in observation.warnings:
        print(f"  WARNING:             {warning}")
    for error in observation.structural_errors:
        print(f"  ERROR:               {error}")

    if show_grids:
        print("\nProducer-native robust median distance (mm)")
        print(_format_grid(observation.distances_mm, decimals=1))
        print("\nProducer-native MAD (mm)")
        print(_format_grid(observation.mad_mm, decimals=1))
        print("\nProducer-native half-capture median drift (mm)")
        print(_format_grid(observation.half_drift_mm, decimals=1))


def main() -> int:
    parser = argparse.ArgumentParser(
        description=(
            "Reduce one stationary Rangeweave ToF capture to robust per-zone "
            "calibration distances and quality diagnostics"
        )
    )
    parser.add_argument("capture", help="capture directory or packets.bin")
    parser.add_argument(
        "--min-frames",
        type=int,
        default=capture_cal.DEFAULT_MIN_FRAMES,
        help=(
            "minimum distance-bearing ToF frames required for structural PASS "
            f"(default: {capture_cal.DEFAULT_MIN_FRAMES})"
        ),
    )
    parser.add_argument(
        "--min-valid-fraction",
        type=float,
        default=capture_cal.DEFAULT_MIN_VALID_FRACTION,
        help=(
            "minimum valid-return fraction for a zone to contribute a calibration "
            f"distance (default: {capture_cal.DEFAULT_MIN_VALID_FRACTION:.2f})"
        ),
    )
    parser.add_argument(
        "--show-grids",
        action="store_true",
        help="also print producer-native 8x8 median/MAD/drift grids",
    )
    args = parser.parse_args()

    try:
        observation = capture_cal.analyse_calibration_capture(
            Path(args.capture),
            min_frames=args.min_frames,
            min_valid_fraction=args.min_valid_fraction,
        )
    except (OSError, depth.DepthAnalysisError, capture_cal.CalibrationCaptureError) as exc:
        parser.error(str(exc))

    print_summary(observation, show_grids=args.show_grids)
    return 0 if observation.structurally_valid else 2


if __name__ == "__main__":
    raise SystemExit(main())
