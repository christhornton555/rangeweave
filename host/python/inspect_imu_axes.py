"""Inspect raw LSM6DSOX axes in one canonical Rangeweave capture.

This is a physical frame-validation aid. It deliberately does not estimate attitude
or apply an IMU->device_body mapping; that mapping is what the experiment is meant
to establish.
"""

from __future__ import annotations

import argparse
import math
from pathlib import Path
import statistics

import rangeweave_capture as cap
import rangeweave_protocol as rw


AXES = ("x", "y", "z")
DEFAULT_EDGE_FRACTION = 0.20


class ImuAxisInspectionError(ValueError):
    """Raised when a capture cannot support the requested IMU-axis inspection."""


def _resolve_packets_path(path: Path | str) -> Path:
    input_path = Path(path)
    if input_path.is_dir():
        input_path = input_path / cap.PACKETS_FILENAME
    if not input_path.is_file():
        raise ImuAxisInspectionError(f"packets file not found: {input_path}")
    return input_path


def _median(values):
    return statistics.median(float(value) for value in values)


def analyse_imu_samples(samples, *, edge_fraction=DEFAULT_EDGE_FRACTION):
    """Summarise one hold-move-hold capture without assuming physical axis mapping."""

    samples = tuple(samples)
    if len(samples) < 20:
        raise ImuAxisInspectionError("at least 20 IMU samples are required")
    if not (0.05 <= float(edge_fraction) <= 0.40):
        raise ImuAxisInspectionError("edge_fraction must be between 0.05 and 0.40")

    edge_count = max(5, int(round(len(samples) * float(edge_fraction))))
    if edge_count * 2 >= len(samples):
        edge_count = max(5, len(samples) // 4)

    initial = samples[:edge_count]
    final = samples[-edge_count:]
    stationary = initial + final

    initial_accel = tuple(
        _median(getattr(sample, f"accel_{axis}") for sample in initial)
        for axis in AXES
    )
    final_accel = tuple(
        _median(getattr(sample, f"accel_{axis}") for sample in final)
        for axis in AXES
    )
    gyro_bias = tuple(
        _median(getattr(sample, f"gyro_{axis}") for sample in stationary)
        for axis in AXES
    )

    signed_activity = []
    absolute_activity = []
    peak_activity = []
    for axis_index, axis in enumerate(AXES):
        corrected = [
            float(getattr(sample, f"gyro_{axis}")) - gyro_bias[axis_index]
            for sample in samples
        ]
        signed_activity.append(sum(corrected))
        absolute_activity.append(sum(abs(value) for value in corrected))
        peak_activity.append(max(abs(value) for value in corrected))

    ordering = sorted(
        range(3),
        key=lambda index: absolute_activity[index],
        reverse=True,
    )
    dominant_index = ordering[0]
    second = absolute_activity[ordering[1]]
    dominance_ratio = (
        math.inf
        if second <= 0.0 and absolute_activity[dominant_index] > 0.0
        else (
            absolute_activity[dominant_index] / second
            if second > 0.0
            else 1.0
        )
    )

    return {
        "sample_count": len(samples),
        "edge_count": edge_count,
        "initial_accel_raw": tuple(initial_accel),
        "final_accel_raw": tuple(final_accel),
        "gyro_bias_raw": tuple(gyro_bias),
        "signed_activity_raw": tuple(signed_activity),
        "absolute_activity_raw": tuple(absolute_activity),
        "peak_activity_raw": tuple(peak_activity),
        "dominant_axis": AXES[dominant_index],
        "dominant_sign": 1 if signed_activity[dominant_index] >= 0.0 else -1,
        "dominance_ratio": dominance_ratio,
    }


def accel_g_per_lsb(ctrl1_xl):
    """Return g/LSB for the LSM6DSOX accelerometer full-scale bits."""

    if ctrl1_xl is None:
        return None
    fs = (int(ctrl1_xl) >> 2) & 0x03
    mg_per_lsb = {
        0b00: 0.061,
        0b01: 0.488,
        0b10: 0.122,
        0b11: 0.244,
    }[fs]
    return mg_per_lsb / 1000.0


def gyro_dps_per_lsb(ctrl2_g):
    """Return deg/s/LSB for the LSM6DSOX gyro full-scale bits."""

    if ctrl2_g is None:
        return None
    value = int(ctrl2_g)
    fs_125 = bool(value & 0x02)
    fs = (value >> 2) & 0x03
    if fs_125 and fs == 0:
        mdps_per_lsb = 4.375
    else:
        mdps_per_lsb = {
            0b00: 8.75,
            0b01: 17.50,
            0b10: 35.0,
            0b11: 70.0,
        }[fs]
    return mdps_per_lsb / 1000.0


def decode_capture(path: Path | str):
    packets_path = _resolve_packets_path(path)
    decoder = rw.StreamDecoder()
    stats = cap.StreamStats()
    samples = []

    with packets_path.open("rb") as handle:
        while True:
            chunk = handle.read(4096)
            if not chunk:
                break
            for frame in decoder.feed(chunk):
                stats.consume(frame)
                if frame.record_type != rw.RECORD_IMU_BATCH:
                    continue
                try:
                    record = rw.decode_record(frame)
                except rw.ProtocolError:
                    continue
                samples.extend(record.samples)

    return packets_path, tuple(samples), decoder, stats


def _config_byte(info, tag):
    if info is None:
        return None
    value = info.first_value(tag)
    if not value or len(value) != 1:
        return None
    return value[0]


def _format_vector(values, scale=None, unit="raw"):
    if scale is None:
        return "X {:+8.1f}  Y {:+8.1f}  Z {:+8.1f} {}".format(*values, unit)
    scaled = tuple(value * scale for value in values)
    return "X {:+8.4f}  Y {:+8.4f}  Z {:+8.4f} {}".format(*scaled, unit)


def main() -> int:
    parser = argparse.ArgumentParser(
        description=(
            "Inspect raw LSM6DSOX accel/gyro axes in a hold-move-hold Rangeweave capture"
        )
    )
    parser.add_argument("capture", help="capture directory or packets.bin")
    parser.add_argument(
        "--edge-fraction",
        type=float,
        default=DEFAULT_EDGE_FRACTION,
        help="fraction at each end treated as stationary for gyro bias (default: 0.20)",
    )
    args = parser.parse_args()

    try:
        packets_path, samples, decoder, stats = decode_capture(Path(args.capture))
        result = analyse_imu_samples(samples, edge_fraction=args.edge_fraction)
    except (OSError, ImuAxisInspectionError) as exc:
        parser.error(str(exc))

    ctrl1 = _config_byte(stats.last_info, rw.INFO_LSM_CTRL1_XL)
    ctrl2 = _config_byte(stats.last_info, rw.INFO_LSM_CTRL2_G)
    accel_scale = accel_g_per_lsb(ctrl1)
    gyro_scale = gyro_dps_per_lsb(ctrl2)
    health_nonzero = {
        name: delta for name, delta in stats.health_deltas().items() if int(delta) != 0
    }

    print("Rangeweave IMU axis inspection")
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
    if ctrl1 is not None or ctrl2 is not None:
        print(
            "  LSM config:       CTRL1_XL={} CTRL2_G={}".format(
                "n/a" if ctrl1 is None else f"0x{ctrl1:02X}",
                "n/a" if ctrl2 is None else f"0x{ctrl2:02X}",
            )
        )

    print()
    print("Initial stationary accelerometer median")
    print("  raw:              " + _format_vector(result["initial_accel_raw"]))
    if accel_scale is not None:
        print("  scaled:           " + _format_vector(
            result["initial_accel_raw"], accel_scale, "g"
        ))

    print("Final stationary accelerometer median")
    print("  raw:              " + _format_vector(result["final_accel_raw"]))
    if accel_scale is not None:
        print("  scaled:           " + _format_vector(
            result["final_accel_raw"], accel_scale, "g"
        ))

    print()
    print("Stationary gyro bias")
    print("  raw:              " + _format_vector(result["gyro_bias_raw"]))
    if gyro_scale is not None:
        print("  scaled:           " + _format_vector(
            result["gyro_bias_raw"], gyro_scale, "deg/s"
        ))

    print()
    print("Bias-removed gyro activity over whole capture")
    for index, axis in enumerate(AXES):
        print(
            "  {}: signed {:+12.1f}  absolute {:12.1f}  peak {:8.1f} raw".format(
                axis.upper(),
                result["signed_activity_raw"][index],
                result["absolute_activity_raw"][index],
                result["peak_activity_raw"][index],
            )
        )

    sign = "+" if result["dominant_sign"] > 0 else "-"
    ratio = result["dominance_ratio"]
    ratio_text = "inf" if math.isinf(ratio) else f"{ratio:.2f}x"
    print()
    print(f"  dominant gyro:    {sign}{result['dominant_axis'].upper()}")
    print(f"  dominance ratio:  {ratio_text} versus next axis")
    if ratio < 3.0:
        print("  WARNING:           motion is not strongly isolated to one sensor axis")

    print()
    print(
        "Interpretation: dominant gyro axis/sign is in raw imu_sensor coordinates; "
        "do not label it device_body until the physical motion is specified."
    )

    return 1 if decoder.frames_bad or stats.sequence_gaps or health_nonzero else 0


if __name__ == "__main__":
    raise SystemExit(main())
