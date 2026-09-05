"""Replay one ToF frame through the orientation-aware geometry path."""

from __future__ import annotations

import argparse
from pathlib import Path

import inspect_orientation_wall as inspect_wall
import rangeweave_geometry as geometry
import rangeweave_imu_quality as imu_quality
import rangeweave_orientation as orientation
import rangeweave_oriented_geometry as oriented
import rangeweave_protocol as rw
import rangeweave_tof_timing as tof_timing


def _selected_grid(tof_grids, requested_index):
    count = len(tof_grids)
    if count == 0:
        raise oriented.OrientedGeometryError("capture contains no ToF grids")
    if requested_index < -count or requested_index >= count:
        raise oriented.OrientedGeometryError(
            f"ToF frame index {requested_index} outside capture range {-count}..{count - 1}"
        )
    index = requested_index % count
    return index, tof_grids[index]


def _extent(points, attribute):
    values = [float(getattr(point, attribute)) for point in points if point is not None]
    if not values:
        raise oriented.OrientedGeometryError("selected frame contains no valid projected points")
    return min(values), max(values)


def _format_matrix(matrix):
    return "\n".join(
        "                   [{:+.8f} {:+.8f} {:+.8f}]".format(*row)
        for row in matrix
    )


def main() -> int:
    parser = argparse.ArgumentParser(
        description=(
            "Replay one Rangeweave ToF grid through calibrated/quick-start timing, "
            "ToF/body boresight and persistent orientation into local_reference"
        )
    )
    parser.add_argument("capture", help="capture directory or packets.bin")
    parser.add_argument("--boresight-artifact", required=True)
    parser.add_argument(
        "--frame",
        type=int,
        default=-1,
        help="ToF frame index; negative indices count from the end (default: -1)",
    )
    parser.add_argument(
        "--geometry-profile",
        default=None,
        metavar="PATH",
        help="optional calibrated rangeweave.tof-geometry-profile JSON",
    )
    parser.add_argument(
        "--timing-mode",
        choices=tof_timing.MODES,
        default=tof_timing.MODE_QUICK_START,
    )
    parser.add_argument("--timing-artifact")
    parser.add_argument("--timing-assembly-id")
    parser.add_argument("--tof-time-offset-ms", type=float, default=None)
    args = parser.parse_args()

    try:
        (
            packets_path,
            imu_samples,
            clock_syncs,
            tof_grids,
            decoder,
            stats,
        ) = inspect_wall._decode(Path(args.capture))
        ctrl1 = inspect_wall._config_byte(stats.last_info, rw.INFO_LSM_CTRL1_XL)
        ctrl2 = inspect_wall._config_byte(stats.last_info, rw.INFO_LSM_CTRL2_G)
        if ctrl1 is None or ctrl2 is None:
            raise oriented.OrientedGeometryError(
                "capture is missing LSM CTRL1_XL/CTRL2_G STREAM_INFO metadata"
            )

        range_usage = imu_quality.analyse_gyro_range(imu_samples, ctrl2_g=ctrl2)
        orientation_run = orientation.estimate_orientation(
            imu_samples,
            clock_syncs,
            ctrl1_xl=ctrl1,
            ctrl2_g=ctrl2,
        )
        boresight = inspect_wall._load_boresight(Path(args.boresight_artifact))
        profile = (
            geometry.NOMINAL_ST_PROFILE
            if args.geometry_profile is None
            else geometry.load_geometry_profile(Path(args.geometry_profile))
        )
        timing_artifact = (
            None
            if args.timing_artifact is None
            else tof_timing.load_artifact(Path(args.timing_artifact))
        )
        timing = tof_timing.resolve_tof_timing(
            stats.last_info,
            tuple(stats.protocol_versions),
            mode=args.timing_mode,
            assembly_id=args.timing_assembly_id,
            artifact=timing_artifact,
            explicit_offset_ms=args.tof_time_offset_ms,
        )
        frame_index, grid = _selected_grid(tof_grids, args.frame)
        reference_frame = oriented.project_tof_grid_to_reference(
            grid,
            imu_samples,
            orientation_run,
            boresight.rotation_body_from_tof,
            timing,
            geometry_profile=profile,
        )
        distances = getattr(grid, "distance_mm")
        points_tof = geometry.project_distances_mm(distances, profile)
    except (
        OSError,
        ValueError,
        orientation.OrientationError,
        oriented.OrientedGeometryError,
        geometry.GeometryError,
        imu_quality.ImuQualityError,
        tof_timing.TofTimingError,
    ) as exc:
        parser.error(str(exc))

    health_nonzero = {
        name: delta for name, delta in stats.health_deltas().items() if int(delta) != 0
    }
    x_tof = _extent(points_tof, "x_mm")
    y_tof = _extent(points_tof, "y_mm")
    z_tof = _extent(points_tof, "z_mm")
    x_ref = _extent(reference_frame.points_reference, "x_mm")
    y_ref = _extent(reference_frame.points_reference, "y_mm")
    z_ref = _extent(reference_frame.points_reference, "z_mm")

    print("Rangeweave orientation-aware ToF geometry replay")
    print(f"  capture:          {packets_path}")
    print(f"  selected frame:   {frame_index} / {len(tof_grids) - 1}")
    print(f"  IMU samples:      {len(imu_samples)}")
    print(f"  ToF grids:        {len(tof_grids)}")
    print(f"  decoder bad:      {decoder.frames_bad}")
    print(f"  semantic errors:  {stats.semantic_errors}")
    print(f"  sequence gaps:    {stats.sequence_gaps}")
    print(
        "  health deltas:    {}".format(
            "all zero"
            if stats.health_deltas() and not health_nonzero
            else (str(health_nonzero) if health_nonzero else "not available")
        )
    )
    print()
    print("Calibration / timing")
    print(f"  boresight role:   {boresight.role}")
    print(f"  geometry:         {profile.name} [{profile.role}]")
    print(f"  timing mode:      {timing.mode}")
    print(f"  timing role:      {timing.role}")
    print(f"  timing source:    {timing.source}")
    print(f"  timing profile:   {timing.artifact_name or '(none)'}")
    print(f"  effective offset: {timing.effective_offset_ms:+.3f} ms")
    print()
    print("Selected observation")
    print(f"  mcu_ready_us:     {reference_frame.mcu_ready_us}")
    print(f"  orientation time: {reference_frame.time_s:.6f} s")
    print(f"  IMU bracket gap:  {reference_frame.orientation_bracket_gap_s * 1000.0:.3f} ms")
    print(f"  accel weight:     {reference_frame.accel_weight:.3f}")
    print(f"  gravity innov.:   {reference_frame.gravity_innovation_deg:.3f} deg")
    print(f"  valid points:     {reference_frame.valid_point_count} / {geometry.ZONE_COUNT}")
    print()
    print("tof_optical extents")
    print(f"  X:                {x_tof[0]:.1f} .. {x_tof[1]:.1f} mm")
    print(f"  Y:                {y_tof[0]:.1f} .. {y_tof[1]:.1f} mm")
    print(f"  Z:                {z_tof[0]:.1f} .. {z_tof[1]:.1f} mm")
    print()
    print("local_reference extents (rotation only; no translation)")
    print(f"  X:                {x_ref[0]:.1f} .. {x_ref[1]:.1f} mm")
    print(f"  Y:                {y_ref[0]:.1f} .. {y_ref[1]:.1f} mm")
    print(f"  Z:                {z_ref[0]:.1f} .. {z_ref[1]:.1f} mm")
    print("  R_reference_from_tof:")
    print(_format_matrix(reference_frame.reference_from_tof))
    print()
    print(
        "Interpretation: these points are rotated into local_reference about the "
        "sensing-head origin. They are not yet translated or globally registered."
    )

    failed = bool(
        decoder.frames_bad
        or stats.semantic_errors
        or stats.sequence_gaps
        or health_nonzero
        or range_usage.rejected
    )
    return 1 if failed else 0


if __name__ == "__main__":
    raise SystemExit(main())
