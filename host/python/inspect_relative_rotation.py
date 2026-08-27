"""Inspect short-baseline relative device_body rotation in a Rangeweave capture."""

from __future__ import annotations

import argparse
from pathlib import Path

import rangeweave_capture as cap
import rangeweave_imu_relative as imu
import rangeweave_protocol as rw


def _resolve_packets_path(path: Path) -> Path:
    if path.is_dir():
        path = path / cap.PACKETS_FILENAME
    if not path.is_file():
        raise imu.RelativeRotationError(f"packets file not found: {path}")
    return path


def _decode(path: Path):
    packets_path = _resolve_packets_path(path)
    decoder = rw.StreamDecoder()
    stats = cap.StreamStats()
    samples = []
    clock_syncs = []

    with packets_path.open("rb") as handle:
        while True:
            chunk = handle.read(4096)
            if not chunk:
                break
            for frame in decoder.feed(chunk):
                stats.consume(frame)
                try:
                    record = rw.decode_record(frame)
                except rw.ProtocolError:
                    continue
                if frame.record_type == rw.RECORD_IMU_BATCH:
                    samples.extend(record.samples)
                elif frame.record_type == rw.RECORD_CLOCK_SYNC:
                    clock_syncs.append(record)

    return packets_path, tuple(samples), tuple(clock_syncs), decoder, stats


def _config_byte(info, tag):
    if info is None:
        return None
    value = info.first_value(tag)
    if not value or len(value) != 1:
        return None
    return value[0]


def _fmt_vec(values, unit, digits=4):
    fmt = "{:" + "+." + str(digits) + "f}"
    return "X {}  Y {}  Z {} {}".format(
        fmt.format(values[0]),
        fmt.format(values[1]),
        fmt.format(values[2]),
        unit,
    )


def _print_window(label, window):
    print(label)
    print(
        "  interval:          {:.3f} .. {:.3f} s ({:.3f} s)".format(
            window.start_time_s,
            window.end_time_s,
            window.end_time_s - window.start_time_s,
        )
    )
    print("  gyro robust spread:{:8.3f} deg/s".format(window.gyro_spread_dps))
    print("  accel median dev:  {:8.4f} g".format(window.accel_median_deviation_g))
    print("  accel norm:        {:8.4f} g".format(window.accel_median_norm_g))


def main() -> int:
    parser = argparse.ArgumentParser(
        description=(
            "Estimate relative device_body rotation between stationary poses "
            "in a hold-move-hold Rangeweave capture"
        )
    )
    parser.add_argument("capture", help="capture directory or packets.bin")
    parser.add_argument(
        "--stationary-window-seconds",
        type=float,
        default=0.60,
        help=(
            "duration of each data-selected stationary window in seconds "
            "(default: 0.60; estimator allows 0.25..2.0)"
        ),
    )
    args = parser.parse_args()

    try:
        packets_path, samples, clock_syncs, decoder, stats = _decode(Path(args.capture))
        ctrl1 = _config_byte(stats.last_info, rw.INFO_LSM_CTRL1_XL)
        ctrl2 = _config_byte(stats.last_info, rw.INFO_LSM_CTRL2_G)
        if ctrl1 is None or ctrl2 is None:
            raise imu.RelativeRotationError(
                "capture is missing LSM CTRL1_XL/CTRL2_G STREAM_INFO metadata"
            )
        result = imu.estimate_relative_body_rotation(
            samples,
            clock_syncs,
            ctrl1_xl=ctrl1,
            ctrl2_g=ctrl2,
            stationary_window_seconds=args.stationary_window_seconds,
        )
    except (OSError, imu.RelativeRotationError) as exc:
        parser.error(str(exc))

    health_nonzero = {
        name: delta for name, delta in stats.health_deltas().items() if int(delta) != 0
    }

    print("Rangeweave relative body-rotation inspection")
    print(f"  capture:          {packets_path}")
    print(f"  IMU samples:      {result.sample_count}")
    print(f"  CLOCK_SYNC:       {result.clock_fit.observation_count}")
    print(f"  decoder bad:      {decoder.frames_bad}")
    print(f"  sequence gaps:    {stats.sequence_gaps}")
    print(
        "  health deltas:    {}".format(
            "all zero"
            if stats.health_deltas() and not health_nonzero
            else (str(health_nonzero) if health_nonzero else "not available")
        )
    )
    print(f"  IMU mapping:      {result.imu_mapping_role}")
    print("                    body X=-imu X, body Y=-imu Z, body Z=-imu Y")
    print(f"  stationary span:  {args.stationary_window_seconds:.3f} s requested")

    print()
    print("LSM clock fit from CLOCK_SYNC")
    print(f"  us per tick:      {result.clock_fit.us_per_tick:.9f}")
    print(f"  fit RMS:          {result.clock_fit.rms_residual_us:.3f} us")
    print(f"  max residual:     {result.clock_fit.max_abs_residual_us:.3f} us")
    print(f"  max half-bracket: {result.clock_fit.max_half_bracket_us:.3f} us")

    print()
    _print_window("Initial stationary window", result.initial_stationary)
    print("  gyro bias:        " + _fmt_vec(result.initial_bias_body_dps, "deg/s"))
    print(
        "  accel median:     "
        + _fmt_vec(result.initial_stationary.accel_median_body_g, "g")
    )

    print()
    _print_window("Final stationary window", result.final_stationary)
    print("  gyro bias:        " + _fmt_vec(result.final_bias_body_dps, "deg/s"))
    print(
        "  accel median:     "
        + _fmt_vec(result.final_stationary.accel_median_body_g, "g")
    )

    print()
    print("Relative body rotation")
    print("  transform:        reference_from_body")
    print(f"  integrated span:  {result.integrated_duration_s:.3f} s")
    print(f"  rotation angle:   {result.rotation_angle_deg:.3f} deg")
    print(
        "  fixed XYZ:        Rx {:+.3f}  Ry {:+.3f}  Rz {:+.3f} deg".format(
            *result.rotation_xyz_deg
        )
    )
    print(
        "  axis-angle axis:  X {:+.4f}  Y {:+.4f}  Z {:+.4f}".format(
            *result.rotation_axis_reference
        )
    )
    print("  matrix:")
    for row in result.reference_from_body:
        print("                   [{:+.7f} {:+.7f} {:+.7f}]".format(*row))

    print()
    print("Independent stationary-accelerometer check")
    print(f"  gravity direction change: {result.gravity_direction_change_deg:.3f} deg")
    print(f"  gyro/gravity closure:     {result.gravity_closure_error_deg:.3f} deg")
    if result.gravity_closure_error_deg > 5.0:
        print("  WARNING: gravity closure is poor; do not use this rotation for boresight")
    elif result.gravity_closure_error_deg > 2.0:
        print("  note: gravity closure is marginal; inspect the physical movement/holds")

    print()
    print(
        "Interpretation: this is a short-baseline relative rotation only. "
        "It does not define world attitude or use magnetometer heading."
    )

    return 1 if decoder.frames_bad or stats.sequence_gaps or health_nonzero else 0


if __name__ == "__main__":
    raise SystemExit(main())
