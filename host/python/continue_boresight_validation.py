"""Capture one additional held-out boresight validation pose from an existing P4.

This command is intentionally narrow: it resumes a completed five-pose guided
session while the rig is still physically at P4, captures M5/P5 with the same
hold/move/settle timing, then fits P0-P4 and evaluates P5 as held-out ToF.
"""

from __future__ import annotations

import argparse
from datetime import datetime, timezone
from pathlib import Path

import guided_boresight as guided
import inspect_boresight_holdout as prior_holdout
import inspect_boresight_sequence as sequence_inspector
import rangeweave_boresight_holdout as holdout
import rangeweave_boresight_sequence as sequence
import rangeweave_extrinsics as ext


VALIDATION_INSTRUCTION = (
    "From P4, yaw left to a clearly different orientation (roughly 20-30 deg if "
    "comfortable), while changing pitch modestly and reducing or changing the "
    "existing roll. Keep the full ToF field on the same wall."
)

DEFAULT_MIN_ROTATION_DEG = 5.0
DEFAULT_MAX_GRAVITY_CLOSURE_DEG = 2.0


def _validation_prefix(session: str) -> str:
    stamp = datetime.now(timezone.utc).strftime("%Y%m%d_%H%M%S")
    return f"{session}-validation-{stamp}"


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


def _validate_motion(index: int, motion, range_usage, *, min_rotation_deg: float, max_gravity_closure_deg: float) -> None:
    if motion.rotation_angle_deg < min_rotation_deg:
        raise sequence.BoresightSequenceError(
            f"M{index} rotation {motion.rotation_angle_deg:.3f} deg is below minimum "
            f"{min_rotation_deg:.3f} deg"
        )
    if motion.gravity_closure_error_deg > max_gravity_closure_deg:
        raise sequence.BoresightSequenceError(
            f"M{index} gravity closure {motion.gravity_closure_error_deg:.3f} deg exceeds "
            f"{max_gravity_closure_deg:.3f} deg"
        )
    if range_usage.rejected:
        raise sequence.BoresightSequenceError(
            f"M{index} gyro range exceeded the safe boresight limit"
        )


def _load_existing_session(
    root: Path,
    session_name: str,
    *,
    stationary_window_seconds: float,
    min_rotation_deg: float,
    max_gravity_closure_deg: float,
    quality_limits: sequence.BoresightPoseQualityLimits,
):
    paths = {"p0": prior_holdout._capture_for(root, session_name, "p0")}
    for index in range(1, 5):
        paths[f"m{index}"] = prior_holdout._capture_for(root, session_name, f"m{index}")
        paths[f"p{index}"] = prior_holdout._capture_for(root, session_name, f"p{index}")

    reference_from_body = ext.identity_matrix()
    poses = [
        sequence.stationary_pose_from_capture(
            paths["p0"], reference_from_body, label="P0", quality_limits=quality_limits
        )
    ]
    motions = []
    for index in range(1, 5):
        motion_path, motion, range_usage = sequence_inspector._motion_estimate(
            paths[f"m{index}"], stationary_window_seconds
        )
        _validate_motion(
            index,
            motion,
            range_usage,
            min_rotation_deg=min_rotation_deg,
            max_gravity_closure_deg=max_gravity_closure_deg,
        )
        reference_from_body = sequence.compose_reference_from_body(
            reference_from_body, motion.reference_from_body
        )
        poses.append(
            sequence.stationary_pose_from_capture(
                paths[f"p{index}"],
                reference_from_body,
                label=f"P{index}",
                quality_limits=quality_limits,
            )
        )
        motions.append((motion_path, motion, range_usage))
    return paths, poses, motions, reference_from_body


def main() -> int:
    parser = argparse.ArgumentParser(
        description=(
            "Continue a completed guided P0-P4 boresight session from its unchanged "
            "physical P4 pose, capture M5/P5, and evaluate P5 as held-out ToF"
        )
    )
    parser.add_argument("port", help="serial port, e.g. COM5 or /dev/ttyACM0")
    parser.add_argument("session", help="existing guided session prefix")
    parser.add_argument("--root", default="captures", help="capture directory root")
    parser.add_argument("--stationary-window-seconds", type=float, default=0.60)
    parser.add_argument("--min-rotation-deg", type=float, default=DEFAULT_MIN_ROTATION_DEG)
    parser.add_argument(
        "--max-gravity-closure-deg", type=float, default=DEFAULT_MAX_GRAVITY_CLOSURE_DEG
    )
    parser.add_argument("--max-plane-rms-mm", type=float, default=sequence.DEFAULT_MAX_PLANE_RMS_MM)
    parser.add_argument(
        "--max-plane-residual-mm", type=float, default=sequence.DEFAULT_MAX_PLANE_RESIDUAL_MM
    )
    parser.add_argument("--max-half-drift-mm", type=float, default=sequence.DEFAULT_MAX_HALF_DRIFT_MM)
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
        root.mkdir(parents=True, exist_ok=True)
        _paths, training_poses, prior_motions, reference_from_p4 = _load_existing_session(
            root,
            args.session,
            stationary_window_seconds=args.stationary_window_seconds,
            min_rotation_deg=float(args.min_rotation_deg),
            max_gravity_closure_deg=float(args.max_gravity_closure_deg),
            quality_limits=limits,
        )
    except (OSError, ValueError, ext.ExtrinsicError, sequence.BoresightSequenceError) as exc:
        parser.error(str(exc))

    print("Rangeweave P5 held-out validation continuation")
    print(f"  existing session:  {args.session}")
    print("  starting pose:      recorded P4 (rig must still be physically unchanged)")
    print("  training ToF:       P0, P1, P2, P3, P4")
    print("  held-out ToF:       new P5")
    print()
    print("M5 instruction:")
    print("  " + VALIDATION_INSTRUCTION)
    print()
    print(
        "Timing remains: 3 s discarded warm-up + 5 s recorded initial hold + "
        "10 s movement + 12 s hands-off final hold."
    )
    print()
    print("IMPORTANT: do not continue if the rig has moved since the recorded P4 pose.")
    input("Press Enter only if the rig is still at P4 and the same wall is unchanged... ")

    prefix = _validation_prefix(args.session)
    try:
        motion5 = guided._run_motion_capture(
            port=args.port,
            root=root,
            name=prefix + "-m5",
            instruction=VALIDATION_INSTRUCTION,
            seconds=guided.DEFAULT_MOTION_CAPTURE_SECONDS,
            warmup_seconds=guided.DEFAULT_WARMUP_SECONDS,
            recorded_initial_hold_seconds=guided.DEFAULT_RECORDED_INITIAL_HOLD_SECONDS,
            move_seconds=guided.DEFAULT_MOVE_SECONDS,
            notes="held-out boresight validation M5 from unchanged P4; " + VALIDATION_INSTRUCTION,
        )
        pose5_path = guided._run_stationary_capture(
            port=args.port,
            root=root,
            name=prefix + "-p5",
            seconds=guided.DEFAULT_STATIONARY_CAPTURE_SECONDS,
            warmup_seconds=guided.DEFAULT_WARMUP_SECONDS,
            notes="held-out boresight validation P5; excluded from P0-P4 training fit",
        )

        _motion_path, motion5_estimate, range5 = sequence_inspector._motion_estimate(
            motion5, args.stationary_window_seconds
        )
        _validate_motion(
            5,
            motion5_estimate,
            range5,
            min_rotation_deg=float(args.min_rotation_deg),
            max_gravity_closure_deg=float(args.max_gravity_closure_deg),
        )
        reference_from_p5 = sequence.compose_reference_from_body(
            reference_from_p4, motion5_estimate.reference_from_body
        )
        pose5 = sequence.stationary_pose_from_capture(
            pose5_path,
            reference_from_p5,
            label="P5-held-out",
            quality_limits=limits,
        )
        evaluation = holdout.evaluate_holdout(
            [pose.observation for pose in training_poses], pose5.observation
        )
    except (OSError, RuntimeError, ValueError, ext.ExtrinsicError, sequence.BoresightSequenceError) as exc:
        guided._banner("P5 VALIDATION STOPPED")
        print(str(exc))
        print("No captures are deleted. Keep the rig/wall fixed for diagnosis if practical.")
        return 2

    guided._banner("P5 HELD-OUT VALIDATION COMPLETE")
    print(f"Validation prefix: {prefix}")
    print(f"M5:                {motion5}")
    print(f"P5:                {pose5_path}")
    print()
    print("Physical quality")
    print(
        "  M5: angle {:.3f} deg; gravity closure {:.3f} deg; gyro {:.1%} used{}".format(
            motion5_estimate.rotation_angle_deg,
            motion5_estimate.gravity_closure_error_deg,
            range5.max_fraction,
            " WARNING" if range5.warning else "",
        )
    )
    print(
        "  P5: plane RMS {:.3f} mm; max {:.3f} mm; half-drift {}".format(
            pose5.tof_plane.rms_residual_mm,
            pose5.tof_plane.max_abs_residual_mm,
            "n/a" if pose5.max_half_drift_mm is None else f"{pose5.max_half_drift_mm:.2f} mm",
        )
    )
    print()

    _fmt_fit("Training fit (P0-P4 only)", evaluation.training_fit)
    print()
    print("Held-out P5 prediction")
    print("  predicted n_tof:  " + _fmt_vec(evaluation.predicted_holdout_tof_normal))
    print("  observed n_tof:   " + _fmt_vec(evaluation.observed_holdout_tof_normal))
    print(f"  angular error:    {evaluation.holdout_normal_error_deg:.3f} deg")
    print("  note: P5 ToF was excluded from the training fit; M5 IMU orientation is an input.")
    print()

    _fmt_fit("All-pose refit (P0-P5)", evaluation.all_pose_fit)
    print()
    print("Fit stability after revealing P5")
    print(
        "  Euler delta:      dRx {:+.3f}  dRy {:+.3f}  dRz {:+.3f} deg".format(
            *evaluation.fit_parameter_delta_deg
        )
    )
    print(f"  rotation change:  {evaluation.fit_rotation_change_deg:.3f} deg")
    print()
    print(
        "Interpretation: P5 is an independent ToF validation pose for the P0-P4 fit. "
        "No new acceptance threshold is imposed by this command."
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
