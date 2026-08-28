"""Inspect a stateless fixed-plane ToF/body boresight capture sequence."""

from __future__ import annotations

import argparse
from pathlib import Path

import rangeweave_boresight_sequence as sequence
import rangeweave_capture as cap
import rangeweave_extrinsics as ext
import rangeweave_imu_quality as imu_quality
import rangeweave_imu_relative as imu
import rangeweave_protocol as rw


def _resolve_packets_path(path: Path) -> Path:
    if path.is_dir():
        path = path / cap.PACKETS_FILENAME
    if not path.is_file():
        raise sequence.BoresightSequenceError(f"packets file not found: {path}")
    return path


def _decode_motion_capture(path: Path):
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

    health_nonzero = {
        name: delta for name, delta in stats.health_deltas().items() if int(delta) != 0
    }
    if decoder.frames_bad or stats.sequence_gaps or health_nonzero:
        raise sequence.BoresightSequenceError(
            "motion capture has stream/health errors: "
            f"bad={decoder.frames_bad}, gaps={stats.sequence_gaps}, health={health_nonzero}"
        )

    return packets_path, tuple(samples), tuple(clock_syncs), stats


def _config_byte(info, tag):
    if info is None:
        return None
    value = info.first_value(tag)
    if not value or len(value) != 1:
        return None
    return value[0]


def _motion_estimate(path: Path, stationary_window_seconds: float):
    packets_path, samples, clock_syncs, stats = _decode_motion_capture(path)
    ctrl1 = _config_byte(stats.last_info, rw.INFO_LSM_CTRL1_XL)
    ctrl2 = _config_byte(stats.last_info, rw.INFO_LSM_CTRL2_G)
    if ctrl1 is None or ctrl2 is None:
        raise sequence.BoresightSequenceError(
            "motion capture is missing LSM CTRL1_XL/CTRL2_G STREAM_INFO metadata"
        )
    try:
        range_usage = imu_quality.analyse_gyro_range(samples, ctrl2_g=ctrl2)
        if range_usage.rejected:
            raise sequence.BoresightSequenceError(
                "gyro range exceeded safe boresight limit for {}: {} reached {:.1%} "
                "of configured +/-{:.0f} deg/s; move more slowly or use a wider gyro range".format(
                    packets_path,
                    range_usage.max_axis,
                    range_usage.max_fraction,
                    range_usage.full_scale_dps,
                )
            )
        result = imu.estimate_relative_body_rotation(
            samples,
            clock_syncs,
            ctrl1_xl=ctrl1,
            ctrl2_g=ctrl2,
            stationary_window_seconds=stationary_window_seconds,
        )
    except (imu.RelativeRotationError, imu_quality.ImuQualityError) as exc:
        raise sequence.BoresightSequenceError(
            f"relative rotation failed for {packets_path}: {exc}"
        ) from exc
    return packets_path, result, range_usage


def _fmt_matrix(matrix):
    return " / ".join(
        "[{:+.5f} {:+.5f} {:+.5f}]".format(*row)
        for row in matrix
    )


def _print_pose(index, pose):
    print(f"Pose {index}: {pose.label}")
    print(f"  ToF capture:      {pose.capture_path}")
    print(
        "  ToF plane:        Rx {:+.3f}  Ry {:+.3f} deg; RMS {:.3f} mm; max {:.3f} mm".format(
            pose.tof_plane.rotation_x_deg,
            pose.tof_plane.rotation_y_deg,
            pose.tof_plane.rms_residual_mm,
            pose.tof_plane.max_abs_residual_mm,
        )
    )
    print(
        f"  ToF quality:      {pose.usable_zone_count}/64 zones, "
        f"{pose.tof_frame_count} frames, median MAD "
        + ("n/a" if pose.median_mad_mm is None else f"{pose.median_mad_mm:.2f} mm")
        + ", max half-drift "
        + ("n/a" if pose.max_half_drift_mm is None else f"{pose.max_half_drift_mm:.2f} mm")
    )
    print(f"  reference_from_body: {_fmt_matrix(pose.reference_from_body)}")


def main() -> int:
    parser = argparse.ArgumentParser(
        description=(
            "Build fixed-plane boresight observations from a baseline stationary ToF "
            "capture plus repeated MOTION_CAPTURE POSE_CAPTURE pairs"
        )
    )
    parser.add_argument("--baseline", required=True, help="stationary baseline capture")
    parser.add_argument(
        "--step",
        action="append",
        nargs=2,
        metavar=("MOTION_CAPTURE", "POSE_CAPTURE"),
        default=[],
        help="one motion capture followed by the stationary ToF capture at its final pose",
    )
    parser.add_argument(
        "--stationary-window-seconds",
        type=float,
        default=0.60,
        help="stationary-window duration for all motion estimates (default: 0.60)",
    )
    parser.add_argument(
        "--min-rotation-deg",
        type=float,
        default=5.0,
        help="reject a motion smaller than this angle (default: 5.0 deg)",
    )
    parser.add_argument(
        "--max-gravity-closure-deg",
        type=float,
        default=2.0,
        help="reject a motion whose independent gravity closure exceeds this value (default: 2.0)",
    )
    args = parser.parse_args()

    if float(args.min_rotation_deg) <= 0.0:
        parser.error("--min-rotation-deg must be positive")
    if float(args.max_gravity_closure_deg) <= 0.0:
        parser.error("--max-gravity-closure-deg must be positive")

    try:
        reference_from_body = ext.identity_matrix()
        poses = [
            sequence.stationary_pose_from_capture(
                Path(args.baseline), reference_from_body, label="baseline"
            )
        ]

        motion_results = []
        for index, (motion_arg, pose_arg) in enumerate(args.step, start=1):
            motion_path, motion, range_usage = _motion_estimate(
                Path(motion_arg), args.stationary_window_seconds
            )
            if motion.rotation_angle_deg < float(args.min_rotation_deg):
                raise sequence.BoresightSequenceError(
                    f"step {index} rotation {motion.rotation_angle_deg:.3f} deg is below "
                    f"minimum {float(args.min_rotation_deg):.3f} deg"
                )
            if motion.gravity_closure_error_deg > float(args.max_gravity_closure_deg):
                raise sequence.BoresightSequenceError(
                    f"step {index} gravity closure {motion.gravity_closure_error_deg:.3f} deg "
                    f"exceeds {float(args.max_gravity_closure_deg):.3f} deg"
                )
            reference_from_body = sequence.compose_reference_from_body(
                reference_from_body, motion.reference_from_body
            )
            pose = sequence.stationary_pose_from_capture(
                Path(pose_arg), reference_from_body, label=f"step-{index}"
            )
            poses.append(pose)
            motion_results.append((motion_path, motion, range_usage))

    except (OSError, sequence.BoresightSequenceError) as exc:
        parser.error(str(exc))

    print("Rangeweave fixed-plane boresight sequence")
    print(f"  observations:     {len(poses)}")
    print(f"  motion steps:     {len(motion_results)}")
    print(f"  stationary span:  {args.stationary_window_seconds:.3f} s")
    print(f"  minimum motion:   {float(args.min_rotation_deg):.3f} deg")
    print(f"  gravity gate:     {float(args.max_gravity_closure_deg):.3f} deg")
    print(
        "  gyro range gate:  warn {:.0%}, reject {:.0%} of configured full scale".format(
            imu_quality.DEFAULT_GYRO_WARNING_FRACTION,
            imu_quality.DEFAULT_GYRO_REJECT_FRACTION,
        )
    )
    print()

    _print_pose(0, poses[0])
    for index, ((motion_path, motion, range_usage), pose) in enumerate(
        zip(motion_results, poses[1:]), start=1
    ):
        print()
        print(f"Motion {index}")
        print(f"  capture:          {motion_path}")
        print(
            "  relative XYZ:     Rx {:+.3f}  Ry {:+.3f}  Rz {:+.3f} deg".format(
                *motion.rotation_xyz_deg
            )
        )
        print(f"  rotation angle:   {motion.rotation_angle_deg:.3f} deg")
        print(f"  gravity closure:  {motion.gravity_closure_error_deg:.3f} deg")
        print(
            "  gyro range:       +/-{:.0f} deg/s; peak {:.1f} deg/s on {}; {:.1%} used{}".format(
                range_usage.full_scale_dps,
                range_usage.max_peak_dps,
                range_usage.max_axis,
                range_usage.max_fraction,
                " WARNING" if range_usage.warning else "",
            )
        )
        print(
            "  stationary gyro:  {:.3f} / {:.3f} deg/s robust spread".format(
                motion.initial_stationary.gyro_spread_dps,
                motion.final_stationary.gyro_spread_dps,
            )
        )
        print()
        _print_pose(index, pose)

    print()
    if len(poses) < 4:
        print(
            f"Boresight solver: not run; need at least {4 - len(poses)} more "
            "stationary pose(s)."
        )
        return 0

    try:
        fit = sequence.solve_if_ready(poses)
    except ext.ExtrinsicError as exc:
        print(f"Boresight solver: observations collected, but solver rejected set: {exc}")
        return 0

    if fit is None:
        print("Boresight solver: not run")
        return 0

    print("Boresight solver")
    print(
        "  R_body_from_tof:  Rx {:+.3f}  Ry {:+.3f}  Rz {:+.3f} deg".format(
            fit.rotation_x_deg, fit.rotation_y_deg, fit.rotation_z_deg
        )
    )
    print(f"  normal RMS:       {fit.rms_normal_error_deg:.3f} deg")
    print(f"  normal max:       {fit.max_normal_error_deg:.3f} deg")
    print(
        "  observability:    X {:.3e}  Y {:.3e}  Z {:.3e}".format(
            *fit.observability_cost_increase_at_1deg
        )
    )
    print(f"  matrix:            {_fmt_matrix(fit.extrinsic.rotation_body_from_tof)}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
