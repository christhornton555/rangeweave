"""Inspect start/end stationary gyro medians in one Rangeweave capture.

This is a diagnostic for hold-move-hold captures. It deliberately does not estimate
orientation or alter the established imu_sensor -> device_body axis mapping.
"""

from __future__ import annotations

import argparse
from pathlib import Path
import statistics

import inspect_imu_axes as imu_axes
import rangeweave_protocol as rw


AXES = ("x", "y", "z")
DEFAULT_EDGE_FRACTION = 0.20


class ImuBiasEdgeError(ValueError):
    pass


def _median(values):
    return statistics.median(float(value) for value in values)


def analyse_bias_edges(samples, *, edge_fraction=DEFAULT_EDGE_FRACTION):
    samples = tuple(samples)
    if len(samples) < 20:
        raise ImuBiasEdgeError("at least 20 IMU samples are required")
    if not (0.05 <= float(edge_fraction) <= 0.40):
        raise ImuBiasEdgeError("edge_fraction must be between 0.05 and 0.40")

    edge_count = max(5, int(round(len(samples) * float(edge_fraction))))
    if edge_count * 2 >= len(samples):
        edge_count = max(5, len(samples) // 4)

    initial = samples[:edge_count]
    final = samples[-edge_count:]
    initial_gyro = tuple(
        _median(getattr(sample, f"gyro_{axis}") for sample in initial)
        for axis in AXES
    )
    final_gyro = tuple(
        _median(getattr(sample, f"gyro_{axis}") for sample in final)
        for axis in AXES
    )
    delta = tuple(end - start for start, end in zip(initial_gyro, final_gyro))

    return {
        "sample_count": len(samples),
        "edge_count": edge_count,
        "initial_gyro_raw": initial_gyro,
        "final_gyro_raw": final_gyro,
        "delta_gyro_raw": delta,
    }


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Compare initial/final stationary gyro medians in a hold-move-hold capture"
    )
    parser.add_argument("capture", help="capture directory or packets.bin")
    parser.add_argument(
        "--edge-fraction",
        type=float,
        default=DEFAULT_EDGE_FRACTION,
        help="fraction at each end treated as stationary (default: 0.20)",
    )
    args = parser.parse_args()

    try:
        packets_path, samples, decoder, stats = imu_axes.decode_capture(Path(args.capture))
        result = analyse_bias_edges(samples, edge_fraction=args.edge_fraction)
    except (OSError, imu_axes.ImuAxisInspectionError, ImuBiasEdgeError) as exc:
        parser.error(str(exc))

    ctrl2 = imu_axes._config_byte(stats.last_info, rw.INFO_LSM_CTRL2_G)
    scale = imu_axes.gyro_dps_per_lsb(ctrl2)
    health_nonzero = {
        name: delta for name, delta in stats.health_deltas().items() if int(delta) != 0
    }

    print("Rangeweave IMU stationary-edge bias inspection")
    print(f"  capture:          {packets_path}")
    print(f"  IMU samples:      {result['sample_count']}")
    print(f"  stationary edge:  {result['edge_count']} samples at each end")
    print(f"  decoder bad:      {decoder.frames_bad}")
    print(f"  sequence gaps:    {stats.sequence_gaps}")
    print(
        "  health deltas:    {}".format(
            "all zero"
            if stats.health_deltas() and not health_nonzero
            else (str(health_nonzero) if health_nonzero else "not available")
        )
    )

    print("\nInitial stationary gyro median")
    print("  raw:              " + imu_axes._format_vector(result["initial_gyro_raw"]))
    if scale is not None:
        print(
            "  scaled:           "
            + imu_axes._format_vector(result["initial_gyro_raw"], scale, "deg/s")
        )

    print("Final stationary gyro median")
    print("  raw:              " + imu_axes._format_vector(result["final_gyro_raw"]))
    if scale is not None:
        print(
            "  scaled:           "
            + imu_axes._format_vector(result["final_gyro_raw"], scale, "deg/s")
        )

    print("Start -> end stationary gyro shift")
    print("  raw:              " + imu_axes._format_vector(result["delta_gyro_raw"]))
    if scale is not None:
        print(
            "  scaled:           "
            + imu_axes._format_vector(result["delta_gyro_raw"], scale, "deg/s")
        )

    print(
        "\nInterpretation: this reports endpoint zero-rate behaviour only; it does not "
        "choose a bias model or estimate orientation."
    )

    return 1 if decoder.frames_bad or stats.sequence_gaps or health_nonzero else 0


if __name__ == "__main__":
    raise SystemExit(main())
