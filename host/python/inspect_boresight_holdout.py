"""Evaluate one guided fixed-wall boresight session with P4 ToF held out."""

from __future__ import annotations

import argparse
from pathlib import Path

import inspect_boresight_sequence as sequence_inspector
import rangeweave_boresight_holdout as holdout
import rangeweave_boresight_sequence as sequence
import rangeweave_extrinsics as ext


DEFAULT_MIN_ROTATION_DEG = 5.0
DEFAULT_MAX_GRAVITY_CLOSURE_DEG = 2.0


def _capture_for(root: Path, session: str, stage: str) -> Path:
    suffix = f"_{session}-{stage}"
    matches = [path for path in root.glob("capture_*") if path.is_dir() and path.name.endswith(suffix)]
    if len(matches) != 1:
        raise sequence.BoresightSequenceError(
            f"expected exactly one capture ending in {suffix!r}; found {len(matches)}"
        )
    return matches[0]


def _fmt_vec(vector) -> str:
    return "X {:+.5f}  Y {:+.5f}  Z {:+.5f}".format(*vector)


def _fmt_fit(label: str, fit: ext.BoresightFit) -> None:
    print(label)
    print(
        "  R_body_from_tof:  Rx {:+.3f}  Ry {:+.3f}  Rz {:+.3f} deg".format(
            fit.rotation_x_deg,
            fit.rotation_y_deg,
            fit.rotation_z_deg,
        )
    )
    print(f"  normal RMS:       {fit.rms_normal_error_deg:.3f} deg")
    print(f"  normal max:       {fit.max_normal_error_deg:.3f} deg")
    print(
        "  observability:    X {:.3e}  Y {:.3e}  Z {:.3e}".format(
            *fit.observability_cost_increase_at_1deg
        )
    )


def main() -> int:
    parser = argparse.ArgumentParser(
        description=(
            "Fit P0-P3 from a guided boresight session, use M4 IMU orientation to "
            "predict P4 ToF, then reveal P4 and compare against the five-pose refit"
        )
    )
    parser.add_argument("session", help="guided session prefix, e.g. boresight-guided-20260904_010944")
    parser.add_argument("--root", default="captures", help="capture directory root")
    parser.add_argument(
        "--stationary-window-seconds", type=float, default=0.60,
        help="stationary-window duration for motion estimates (default: 0.60)",
    )
    parser.add_argument(
        "--min-rotation-deg", type=float, default=DEFAULT_MIN_ROTATION_DEG,
        help="reject a motion below this angle (default: 5 deg)",
    )
    parser.add_argument(
        "--max-gravity-closure-deg", type=float, default=DEFAULT_MAX_GRAVITY_CLOSURE_DEG,
        help="reject a motion above this gravity closure (default: 2 deg)",
    )
    parser.add_argument(
        "--max-plane-rms-mm", type=float, default=sequence.DEFAULT_MAX_PLANE_RMS_MM,
    )
    parser.add_argument(
        "--max-plane-residual-mm", type=float, default=sequence.DEFAULT_MAX_PLANE_RESIDUAL_MM,
    )
    parser.add_argument(
        "--max-half-drift-mm", type=float, default=sequence.DEFAULT_MAX_HALF_DRIFT_MM,
    )
    args = parser.parse_args()

    if args.stationary_window_seconds <= 0.0:
        parser.error("--stationary-window-seconds must be positive")
    if args.min_rotation_deg <= 0.0:
        parser.error("--min-rotation-deg must be positive")
    if args.max_gravity_closure_deg <= 0.0:
        parser.error("--max-gravity-closure-deg must be positive")

    try:
        limits = sequence.BoresightPoseQualityLimits(
            max_plane_rms_mm=args.max_plane_rms_mm,
            max_plane_residual_mm=args.max_plane_residual_mm,
            max_half_drift_mm=args.max_half_drift_mm,
        )
        root = Path(args.root)
        paths = {"p0": _capture_for(root, args.session, "p0")}
        for index in range(1, 5):
            paths[f"m{index}"] = _capture_for(root, args.session, f"m{index}")
            paths[f"p{index}"] = _capture_for(root, args.session, f"p{index}")

        reference_from_body = ext.identity_matrix()
        training_poses = [
            sequence.stationary_pose_from_capture(
                paths["p0"],
                reference_from_body,
                label="P0",
                quality_limits=limits,
            )
        ]
        motions = []

        for index in range(1, 5):
            motion_path, motion, range_usage = sequence_inspector._motion_estimate(
                paths[f"m{index}"],
                args.stationary_window_seconds,
            )
            if motion.rotation_angle_deg < float(args.min_rotation_deg):
                raise sequence.BoresightSequenceError(
                    f"M{index} rotation {motion.rotation_angle_deg:.3f} deg is below "
                    f"minimum {float(args.min_rotation_deg):.3f} deg"
                )
            if motion.gravity_closure_error_deg > float(args.max_gravity_closure_deg):
                raise sequence.BoresightSequenceError(
                    f"M{index} gravity closure {motion.gravity_closure_error_deg:.3f} deg "
                    f"exceeds {float(args.max_gravity_closure_deg):.3f} deg"
                )
            reference_from_body = sequence.compose_reference_from_body(
                reference_from_body,
                motion.reference_from_body,
            )
            motions.append((motion_path, motion, range_usage))

            if index <= 3:
                training_poses.append(
                    sequence.stationary_pose_from_capture(
                        paths[f"p{index}"],
                        reference_from_body,
                        label=f"P{index}",
                        quality_limits=limits,
                    )
                )

        # P4 ToF is deliberately reduced only after the P0-P3 training sequence and
        # M4 body orientation have been established.  Its ToF normal is not used by
        # the training fit inside evaluate_holdout().
        holdout_pose = sequence.stationary_pose_from_capture(
            paths["p4"],
            reference_from_body,
            label="P4-held-out",
            quality_limits=limits,
        )
        evaluation = holdout.evaluate_holdout(
            [pose.observation for pose in training_poses],
            holdout_pose.observation,
        )

    except (OSError, ValueError, ext.ExtrinsicError, sequence.BoresightSequenceError) as exc:
        parser.error(str(exc))

    print("Rangeweave boresight held-out validation")
    print(f"  session:           {args.session}")
    print("  training ToF:      P0, P1, P2, P3")
    print("  held-out ToF:      P4")
    print("  P4 body pose:      P0->M1->M2->M3->M4 IMU composition")
    print()

    print("Physical quality summary")
    for index, (_path, motion, range_usage) in enumerate(motions, start=1):
        print(
            "  M{}: angle {:6.3f} deg; gravity closure {:5.3f} deg; gyro {:4.1%} used{}".format(
                index,
                motion.rotation_angle_deg,
                motion.gravity_closure_error_deg,
                range_usage.max_fraction,
                " WARNING" if range_usage.warning else "",
            )
        )
    for index, pose in enumerate(training_poses + [holdout_pose]):
        print(
            "  P{}: plane RMS {:5.3f} mm; max {:5.3f} mm; half-drift {}".format(
                index,
                pose.tof_plane.rms_residual_mm,
                pose.tof_plane.max_abs_residual_mm,
                "n/a" if pose.max_half_drift_mm is None else f"{pose.max_half_drift_mm:.2f} mm",
            )
        )
    print()

    _fmt_fit("Training fit (P0-P3 only)", evaluation.training_fit)
    print()
    print("Held-out P4 prediction")
    print("  predicted n_tof:  " + _fmt_vec(evaluation.predicted_holdout_tof_normal))
    print("  observed n_tof:   " + _fmt_vec(evaluation.observed_holdout_tof_normal))
    print(f"  angular error:    {evaluation.holdout_normal_error_deg:.3f} deg")
    print("  note: P4 ToF was excluded from the training fit; M4 IMU orientation is an input.")
    print()

    _fmt_fit("All-pose refit (P0-P4)", evaluation.all_pose_fit)
    print()
    print("Fit stability after revealing P4")
    print(
        "  Euler delta:      dRx {:+.3f}  dRy {:+.3f}  dRz {:+.3f} deg".format(
            *evaluation.fit_parameter_delta_deg
        )
    )
    print(f"  rotation change:  {evaluation.fit_rotation_change_deg:.3f} deg")
    print()
    print(
        "Interpretation: this command intentionally reports held-out error and fit "
        "stability without imposing a new acceptance threshold."
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
