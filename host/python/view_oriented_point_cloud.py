"""Replay Rangeweave ToF geometry in sensor and local-reference frames.

This viewer is deliberately rotation-only.  ``local_reference`` mode applies the
calibrated/quick-start ToF timing resolution, ToF/body boresight and Phase 3
orientation estimate, but no sensing-head translation.  A static wall should
therefore become much more stable in *orientation* while small position shifts
from the physical rig/pivot may remain visible.
"""

from __future__ import annotations

import argparse
from dataclasses import dataclass
from pathlib import Path

import inspect_orientation_wall as inspect_wall
import rangeweave_geometry as geometry
import rangeweave_imu_quality as imu_quality
import rangeweave_orientation as orientation
import rangeweave_oriented_geometry as oriented
import rangeweave_protocol as rw
import rangeweave_tof_timing as tof_timing


@dataclass(frozen=True)
class ReplayFrame:
    source_index: int
    mcu_ready_us: int
    time_s: float
    points_tof: tuple[geometry.Point3 | None, ...]
    points_reference: tuple[geometry.Point3 | None, ...]


@dataclass(frozen=True)
class ReplayData:
    packets_path: Path
    frames: tuple[ReplayFrame, ...]
    total_tof_grids: int
    skipped_outside_orientation: int
    timing: tof_timing.TimingResolution
    geometry_profile: geometry.TofGeometryProfile
    boresight_role: str
    stream_clean: bool


def _valid_xyz(points):
    valid = [point for point in points if point is not None]
    if not valid:
        raise oriented.OrientedGeometryError("frame contains no valid projected points")
    return (
        [point.x_mm for point in valid],
        [point.y_mm for point in valid],
        [point.z_mm for point in valid],
    )


def _axis_limits(frames, attribute, padding_fraction=0.05):
    values = []
    for points in frames:
        for point in points:
            if point is not None:
                values.append(float(getattr(point, attribute)))
    if not values:
        raise oriented.OrientedGeometryError("replay contains no valid projected points")
    low = min(values)
    high = max(values)
    span = max(high - low, 1.0)
    pad = span * float(padding_fraction)
    return low - pad, high + pad


def build_replay_data(
    capture: Path,
    *,
    boresight_artifact: Path,
    timing_mode: str,
    timing_artifact: Path | None,
    timing_assembly_id: str | None,
    tof_time_offset_ms: float | None,
    geometry_profile_path: Path | None,
) -> ReplayData:
    (
        packets_path,
        imu_samples,
        clock_syncs,
        tof_grids,
        decoder,
        stats,
    ) = inspect_wall._decode(capture)

    ctrl1 = inspect_wall._config_byte(stats.last_info, rw.INFO_LSM_CTRL1_XL)
    ctrl2 = inspect_wall._config_byte(stats.last_info, rw.INFO_LSM_CTRL2_G)
    if ctrl1 is None or ctrl2 is None:
        raise oriented.OrientedGeometryError(
            "capture is missing LSM CTRL1_XL/CTRL2_G STREAM_INFO metadata"
        )

    range_usage = imu_quality.analyse_gyro_range(imu_samples, ctrl2_g=ctrl2)
    if range_usage.rejected:
        raise oriented.OrientedGeometryError("configured gyro range is rejected by utilisation gate")

    orientation_run = orientation.estimate_orientation(
        imu_samples,
        clock_syncs,
        ctrl1_xl=ctrl1,
        ctrl2_g=ctrl2,
    )
    boresight = inspect_wall._load_boresight(boresight_artifact)
    profile = (
        geometry.NOMINAL_ST_PROFILE
        if geometry_profile_path is None
        else geometry.load_geometry_profile(geometry_profile_path)
    )
    timing_artifact_object = (
        None if timing_artifact is None else tof_timing.load_artifact(timing_artifact)
    )
    timing = tof_timing.resolve_tof_timing(
        stats.last_info,
        tuple(stats.protocol_versions),
        mode=timing_mode,
        assembly_id=timing_assembly_id,
        artifact=timing_artifact_object,
        explicit_offset_ms=tof_time_offset_ms,
    )

    frames = []
    outside = 0
    for index, grid in enumerate(tof_grids):
        distances = getattr(grid, "distance_mm", None)
        if distances is None:
            continue
        try:
            reference_frame = oriented.project_tof_grid_to_reference(
                grid,
                imu_samples,
                orientation_run,
                boresight.rotation_body_from_tof,
                timing,
                geometry_profile=profile,
            )
        except oriented.OrientedGeometryError as exc:
            if "outside the orientation run" in str(exc):
                outside += 1
                continue
            raise
        points_tof = geometry.project_distances_mm(distances, profile)
        frames.append(
            ReplayFrame(
                source_index=index,
                mcu_ready_us=reference_frame.mcu_ready_us,
                time_s=reference_frame.time_s,
                points_tof=points_tof,
                points_reference=reference_frame.points_reference,
            )
        )

    if not frames:
        raise oriented.OrientedGeometryError("no ToF frames remain for replay")

    health_deltas = stats.health_deltas()
    stream_clean = bool(
        decoder.frames_bad == 0
        and stats.semantic_errors == 0
        and stats.sequence_gaps == 0
        and health_deltas
        and all(int(value) == 0 for value in health_deltas.values())
    )

    return ReplayData(
        packets_path=packets_path,
        frames=tuple(frames),
        total_tof_grids=len(tof_grids),
        skipped_outside_orientation=outside,
        timing=timing,
        geometry_profile=profile,
        boresight_role=boresight.role,
        stream_clean=stream_clean,
    )


def import_matplotlib():
    try:
        import matplotlib.pyplot as plt
        from matplotlib.animation import FuncAnimation
    except ModuleNotFoundError as exc:
        raise RuntimeError(
            "Matplotlib is required for graphical replay: py -m pip install matplotlib"
        ) from exc
    return plt, FuncAnimation


def _configure_axis(ax, *, title, frame_name, limits):
    ax.set_title(title)
    ax.set_xlabel(f"{frame_name} X (mm)")
    ax.set_ylabel(f"{frame_name} Y (mm)")
    ax.set_zlabel(f"{frame_name} Z (mm)")
    ax.set_xlim(*limits[0])
    ax.set_ylim(*limits[1])
    ax.set_zlim(*limits[2])
    spans = [max(high - low, 1.0) for low, high in limits]
    ax.set_box_aspect(spans)
    ax.view_init(elev=22, azim=-55)


def build_figure(replay: ReplayData, *, mode: str, fps: float):
    plt, FuncAnimation = import_matplotlib()
    if mode not in ("compare", "sensor", "reference"):
        raise ValueError("mode must be compare, sensor or reference")
    if fps <= 0.0:
        raise ValueError("fps must be greater than zero")

    tof_sequences = [frame.points_tof for frame in replay.frames]
    ref_sequences = [frame.points_reference for frame in replay.frames]
    tof_limits = tuple(_axis_limits(tof_sequences, axis) for axis in ("x_mm", "y_mm", "z_mm"))
    ref_limits = tuple(_axis_limits(ref_sequences, axis) for axis in ("x_mm", "y_mm", "z_mm"))

    if mode == "compare":
        fig = plt.figure(figsize=(15, 7))
        axes = [
            fig.add_subplot(121, projection="3d"),
            fig.add_subplot(122, projection="3d"),
        ]
        _configure_axis(
            axes[0],
            title="Sensor view — tof_optical",
            frame_name="tof_optical",
            limits=tof_limits,
        )
        _configure_axis(
            axes[1],
            title="Orientation compensated — local_reference",
            frame_name="local_reference",
            limits=ref_limits,
        )
        scatters = [axes[0].scatter([], [], [], s=28), axes[1].scatter([], [], [], s=28)]
    else:
        fig = plt.figure(figsize=(9, 7))
        axis = fig.add_subplot(111, projection="3d")
        if mode == "sensor":
            _configure_axis(
                axis,
                title="Sensor view — tof_optical",
                frame_name="tof_optical",
                limits=tof_limits,
            )
        else:
            _configure_axis(
                axis,
                title="Orientation compensated — local_reference",
                frame_name="local_reference",
                limits=ref_limits,
            )
        axes = [axis]
        scatters = [axis.scatter([], [], [], s=32)]

    status = fig.text(0.5, 0.02, "", ha="center", va="bottom")

    def update(animation_index):
        frame = replay.frames[animation_index]
        point_sets = (
            [frame.points_tof, frame.points_reference]
            if mode == "compare"
            else [frame.points_tof if mode == "sensor" else frame.points_reference]
        )
        for scatter, points in zip(scatters, point_sets):
            xs, ys, zs = _valid_xyz(points)
            scatter._offsets3d = (xs, ys, zs)
        status.set_text(
            "source frame {}/{}   replay {}/{}   t={:.3f}s   timing {} {:+.1f} ms   "
            "rotation only — no translation".format(
                frame.source_index,
                replay.total_tof_grids - 1,
                animation_index + 1,
                len(replay.frames),
                frame.time_s,
                replay.timing.role,
                replay.timing.effective_offset_ms,
            )
        )
        return tuple(scatters) + (status,)

    interval_ms = 1000.0 / float(fps)
    animation = FuncAnimation(
        fig,
        update,
        frames=len(replay.frames),
        interval=interval_ms,
        blit=False,
        repeat=True,
    )
    update(0)
    fig.suptitle(
        "Rangeweave orientation-aware ToF replay\n"
        f"{replay.geometry_profile.name} [{replay.geometry_profile.role}] — "
        f"boresight {replay.boresight_role}, timing {replay.timing.role}"
    )
    fig.tight_layout(rect=(0.0, 0.06, 1.0, 0.93))
    return plt, fig, animation


def print_summary(replay: ReplayData, *, fps: float, mode: str) -> None:
    print("Rangeweave orientation-aware point-cloud replay")
    print(f"  capture:          {replay.packets_path}")
    print(f"  mode:             {mode}")
    print(f"  replay frames:    {len(replay.frames)} / {replay.total_tof_grids}")
    print(f"  outside attitude: {replay.skipped_outside_orientation}")
    print(f"  stream health:    {'PASS' if replay.stream_clean else 'NOT CLEAN'}")
    print(f"  boresight role:   {replay.boresight_role}")
    print(
        f"  geometry:         {replay.geometry_profile.name} "
        f"[{replay.geometry_profile.role}]"
    )
    print(f"  timing mode:      {replay.timing.mode}")
    print(f"  timing role:      {replay.timing.role}")
    print(f"  timing source:    {replay.timing.source}")
    print(f"  timing profile:   {replay.timing.artifact_name or '(none)'}")
    print(f"  effective offset: {replay.timing.effective_offset_ms:+.3f} ms")
    print(f"  playback rate:    {fps:.3f} fps")
    print(
        "  scope:            orientation compensation only; sensing-head translation "
        "is not estimated or applied"
    )


def main() -> int:
    parser = argparse.ArgumentParser(
        description=(
            "Replay a Rangeweave capture in tof_optical and/or orientation-compensated "
            "local_reference coordinates"
        )
    )
    parser.add_argument("capture", help="capture directory or packets.bin")
    parser.add_argument("--boresight-artifact", required=True)
    parser.add_argument(
        "--mode",
        choices=("compare", "sensor", "reference"),
        default="compare",
        help="viewer layout (default: compare)",
    )
    parser.add_argument("--fps", type=float, default=15.0, help="playback rate (default: 15)")
    parser.add_argument("--geometry-profile", default=None, metavar="PATH")
    parser.add_argument(
        "--timing-mode",
        choices=tof_timing.MODES,
        default=tof_timing.MODE_QUICK_START,
    )
    parser.add_argument("--timing-artifact")
    parser.add_argument("--timing-assembly-id")
    parser.add_argument("--tof-time-offset-ms", type=float, default=None)
    parser.add_argument("--no-show", action="store_true")
    args = parser.parse_args()

    if args.fps <= 0.0:
        parser.error("--fps must be greater than zero")

    try:
        replay = build_replay_data(
            Path(args.capture),
            boresight_artifact=Path(args.boresight_artifact),
            timing_mode=args.timing_mode,
            timing_artifact=(Path(args.timing_artifact) if args.timing_artifact else None),
            timing_assembly_id=args.timing_assembly_id,
            tof_time_offset_ms=args.tof_time_offset_ms,
            geometry_profile_path=(Path(args.geometry_profile) if args.geometry_profile else None),
        )
        print_summary(replay, fps=args.fps, mode=args.mode)
        plt, fig, animation = build_figure(replay, mode=args.mode, fps=args.fps)
        # Keep a live reference for Matplotlib's animation lifetime.
        fig._rangeweave_animation = animation
    except (
        OSError,
        RuntimeError,
        ValueError,
        orientation.OrientationError,
        oriented.OrientedGeometryError,
        geometry.GeometryError,
        imu_quality.ImuQualityError,
        tof_timing.TofTimingError,
    ) as exc:
        parser.error(str(exc))

    if args.no_show:
        plt.close(fig)
    else:
        plt.show()
    return 0 if replay.stream_clean else 1


if __name__ == "__main__":
    raise SystemExit(main())
