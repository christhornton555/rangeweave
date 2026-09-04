"""Generate a versioned per-device ToF/body rotation artifact from a validated P0-P5 session."""

from __future__ import annotations

import argparse
from datetime import datetime, timezone
import json
from pathlib import Path
from typing import Iterable

import continue_boresight_validation as continuation
import inspect_boresight_holdout as prior_holdout
import inspect_boresight_sequence as sequence_inspector
import rangeweave_boresight_holdout as holdout
import rangeweave_boresight_sequence as sequence
import rangeweave_capture as capture
import rangeweave_extrinsics as ext
import rangeweave_geometry as geometry


DEFAULT_STATIONARY_WINDOW_SECONDS = 0.60
DEFAULT_MIN_ROTATION_DEG = 5.0
DEFAULT_MAX_GRAVITY_CLOSURE_DEG = 2.0


def _capture_for_optional_validation(root: Path, session: str, stage: str) -> tuple[Path, str]:
    """Resolve M5/P5 from either a single full session or one continuation prefix."""

    direct_suffix = f"_{session}-{stage}"
    direct = [p for p in root.glob("capture_*") if p.is_dir() and p.name.endswith(direct_suffix)]
    if len(direct) == 1:
        return direct[0], session
    if len(direct) > 1:
        raise sequence.BoresightSequenceError(
            f"expected at most one capture ending in {direct_suffix!r}; found {len(direct)}"
        )

    marker = f"_{session}-validation-"
    suffix = f"-{stage}"
    matches = [
        p for p in root.glob("capture_*")
        if p.is_dir() and marker in p.name and p.name.endswith(suffix)
    ]
    if len(matches) != 1:
        raise sequence.BoresightSequenceError(
            f"expected exactly one {stage.upper()} capture for {session!r}; found {len(matches)}"
        )
    name = matches[0].name
    validation_prefix = name[name.index(session): -len(suffix)]
    return matches[0], validation_prefix


def _capture_provenance(path: Path) -> dict:
    metadata = capture.load_metadata(path)
    packets = metadata.get("packets", {})
    stream_info = metadata.get("stream", {}).get("stream_info")
    return {
        "capture_directory": path.name,
        "started_at_utc": metadata.get("started_at_utc"),
        "packets_sha256": packets.get("sha256"),
        "packets_bytes": packets.get("bytes"),
        "stream_info": stream_info,
    }


def _artifact_document(
    *,
    session: str,
    validation_prefix: str,
    all_pose_fit: ext.BoresightFit,
    evaluation: holdout.BoresightHoldoutEvaluation,
    capture_paths: dict[str, Path],
    imu_mapping_role: str,
    gyro_full_scales_dps: Iterable[float],
) -> dict:
    calibrated = ext.TofBodyRotation(
        name=f"tof-body-boresight-{session}",
        role="calibrated",
        rotation_body_from_tof=all_pose_fit.extrinsic.rotation_body_from_tof,
        metadata={
            "calibration": {
                "method": ext.BORESIGHT_METHOD,
                "observation_count": 6,
                "poses": ["P0", "P1", "P2", "P3", "P4", "P5"],
                "rotation_parameter_convention": "active fixed-axis X then Y then Z; R = Rz*Ry*Rx",
                "rotation_xyz_deg": [
                    all_pose_fit.rotation_x_deg,
                    all_pose_fit.rotation_y_deg,
                    all_pose_fit.rotation_z_deg,
                ],
                "rms_normal_error_deg": all_pose_fit.rms_normal_error_deg,
                "max_normal_error_deg": all_pose_fit.max_normal_error_deg,
                "observability_cost_increase_at_1deg": list(
                    all_pose_fit.observability_cost_increase_at_1deg
                ),
            },
            "validation": {
                "method": "held-out-final-tof-pose-v1",
                "training_poses": ["P0", "P1", "P2", "P3", "P4"],
                "held_out_pose": "P5",
                "held_out_normal_error_deg": evaluation.holdout_normal_error_deg,
                "training_fit_rotation_xyz_deg": [
                    evaluation.training_fit.rotation_x_deg,
                    evaluation.training_fit.rotation_y_deg,
                    evaluation.training_fit.rotation_z_deg,
                ],
                "all_pose_fit_rotation_xyz_deg": [
                    evaluation.all_pose_fit.rotation_x_deg,
                    evaluation.all_pose_fit.rotation_y_deg,
                    evaluation.all_pose_fit.rotation_z_deg,
                ],
                "fit_parameter_delta_deg": list(evaluation.fit_parameter_delta_deg),
                "fit_rotation_change_deg": evaluation.fit_rotation_change_deg,
            },
            "provenance": {
                "guided_session": session,
                "validation_prefix": validation_prefix,
                "imu_mapping_role": imu_mapping_role,
                "gyro_full_scales_dps": sorted(set(float(v) for v in gyro_full_scales_dps)),
                "geometry_profile": {
                    "schema": geometry.PROFILE_SCHEMA,
                    "schema_version": geometry.PROFILE_SCHEMA_VERSION,
                    "name": geometry.NOMINAL_ST_PROFILE.name,
                    "role": geometry.NOMINAL_ST_PROFILE.role,
                    "frame": geometry.NOMINAL_ST_PROFILE.frame,
                    "distance_contract": geometry.NOMINAL_ST_PROFILE.distance_contract,
                },
                "quality_gates": {
                    "minimum_relative_rotation_deg": DEFAULT_MIN_ROTATION_DEG,
                    "maximum_gravity_closure_deg": DEFAULT_MAX_GRAVITY_CLOSURE_DEG,
                    "maximum_plane_rms_mm": sequence.DEFAULT_MAX_PLANE_RMS_MM,
                    "maximum_plane_residual_mm": sequence.DEFAULT_MAX_PLANE_RESIDUAL_MM,
                    "maximum_half_drift_mm": sequence.DEFAULT_MAX_HALF_DRIFT_MM,
                },
                "captures": {
                    key: _capture_provenance(path)
                    for key, path in sorted(capture_paths.items())
                },
            },
            "generated": {
                "tool": "host/python/generate_boresight_artifact.py",
                "created_at_utc": datetime.now(timezone.utc).isoformat(timespec="seconds").replace("+00:00", "Z"),
            },
        },
    )
    return calibrated.to_dict()


def main() -> int:
    parser = argparse.ArgumentParser(
        description=(
            "Re-evaluate a validated P0-P5 fixed-wall boresight sequence and write a "
            "versioned rangeweave.tof-body-rotation artifact with capture provenance"
        )
    )
    parser.add_argument("session", help="guided session prefix")
    parser.add_argument("--root", default="captures", help="capture directory root")
    parser.add_argument(
        "--output",
        help=(
            "artifact path; default calibration/tof-body-rotation-<session>.json"
        ),
    )
    args = parser.parse_args()

    root = Path(args.root)
    output = Path(args.output) if args.output else Path("calibration") / f"tof-body-rotation-{args.session}.json"

    try:
        limits = sequence.BoresightPoseQualityLimits()
        paths, poses, motions, reference_from_p4 = continuation._load_existing_session(
            root,
            args.session,
            stationary_window_seconds=DEFAULT_STATIONARY_WINDOW_SECONDS,
            min_rotation_deg=DEFAULT_MIN_ROTATION_DEG,
            max_gravity_closure_deg=DEFAULT_MAX_GRAVITY_CLOSURE_DEG,
            quality_limits=limits,
        )

        m5_path, validation_prefix_m = _capture_for_optional_validation(root, args.session, "m5")
        p5_path, validation_prefix_p = _capture_for_optional_validation(root, args.session, "p5")
        if validation_prefix_m != validation_prefix_p:
            raise sequence.BoresightSequenceError(
                "M5 and P5 do not belong to the same validation continuation"
            )

        _m5_packets, motion5, range5 = sequence_inspector._motion_estimate(
            m5_path, DEFAULT_STATIONARY_WINDOW_SECONDS
        )
        continuation._validate_motion(
            5,
            motion5,
            range5,
            min_rotation_deg=DEFAULT_MIN_ROTATION_DEG,
            max_gravity_closure_deg=DEFAULT_MAX_GRAVITY_CLOSURE_DEG,
        )
        reference_from_p5 = sequence.compose_reference_from_body(
            reference_from_p4, motion5.reference_from_body
        )
        pose5 = sequence.stationary_pose_from_capture(
            p5_path,
            reference_from_p5,
            label="P5-held-out",
            quality_limits=limits,
        )
        evaluation = holdout.evaluate_holdout(
            [pose.observation for pose in poses], pose5.observation
        )

        capture_paths = dict(paths)
        capture_paths["m5"] = m5_path
        capture_paths["p5"] = p5_path
        gyro_ranges = [item[2].full_scale_dps for item in motions] + [range5.full_scale_dps]
        document = _artifact_document(
            session=args.session,
            validation_prefix=validation_prefix_m,
            all_pose_fit=evaluation.all_pose_fit,
            evaluation=evaluation,
            capture_paths=capture_paths,
            imu_mapping_role=motion5.imu_mapping_role,
            gyro_full_scales_dps=gyro_ranges,
        )

        output.parent.mkdir(parents=True, exist_ok=True)
        output.write_text(json.dumps(document, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    except (OSError, ValueError, KeyError, json.JSONDecodeError, ext.ExtrinsicError, sequence.BoresightSequenceError) as exc:
        parser.error(str(exc))

    print("Rangeweave ToF/body rotation artifact generated")
    print(f"  output:             {output}")
    print(f"  schema:             {document['schema']} v{document['schema_version']}")
    calibration = document["metadata"]["calibration"]
    validation = document["metadata"]["validation"]
    print(
        "  R_body_from_tof:    Rx {:+.3f}  Ry {:+.3f}  Rz {:+.3f} deg".format(
            *calibration["rotation_xyz_deg"]
        )
    )
    print(f"  normal RMS:         {calibration['rms_normal_error_deg']:.3f} deg")
    print(f"  held-out P5 error:  {validation['held_out_normal_error_deg']:.3f} deg")
    print(f"  fit change at P5:   {validation['fit_rotation_change_deg']:.3f} deg")
    print("  geometry role:      " + document["metadata"]["provenance"]["geometry_profile"]["role"])
    print("  capture provenance: P0-P5 and M1-M5 hashes embedded")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
