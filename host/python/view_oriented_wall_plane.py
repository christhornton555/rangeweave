"""Visualize a flat wall as a rolling best-fit plane in ``local_reference``.

This diagnostic is for captures known to contain one approximately planar target.
It complements ``view_oriented_point_cloud.py`` by fitting an orthogonal plane to
each orientation-compensated ToF frame, averaging accepted plane normals over a
short trailing window, and showing the resulting residual orientation over time.

Only orientation is evaluated.  Each displayed plane is placed through the current
point-cloud centroid, so sensing-head translation does not masquerade as a plane-
normal error.  Rotation about the wall normal is fundamentally unobservable from a
featureless plane and is therefore not measured here.
"""

from __future__ import annotations

import argparse
from dataclasses import dataclass
import math
from pathlib import Path
import statistics

import rangeweave_geometry as geometry
import rangeweave_imu_quality as imu_quality
import rangeweave_orientation as orientation
import rangeweave_oriented_geometry as oriented
import rangeweave_reference_plane as reference_plane
import rangeweave_tof_timing as tof_timing
import view_oriented_point_cloud as replay_view


DEFAULT_PLANE_WINDOW = 5
DEFAULT_MIN_POINTS = 48
DEFAULT_MAX_RMS_MM = 10.0
DEFAULT_MAX_RESIDUAL_MM = 30.0


@dataclass(frozen=True)
class PlaneDiagnostic:
    fit: reference_plane.PlaneFit | None
    rolling_normal: reference_plane.Vector3 | None
    angular_error_deg: float | None
    delta_rx_deg: float | None
    delta_ry_deg: float | None


def _normal_rx_ry(normal):
    nx, ny, nz = normal
    rx = math.degrees(math.asin(max(-1.0, min(1.0, -ny))))
    ry = math.degrees(math.atan2(nx, nz))
    return rx, ry


def _percentile(values, probability):
    ordered = sorted(float(value) for value in values)
    if not ordered:
        raise reference_plane.ReferencePlaneError("cannot calculate percentile of empty data")
    if len(ordered) == 1:
        return ordered[0]
    position = float(probability) * (len(ordered) - 1)
    lower = int(math.floor(position))
    upper = int(math.ceil(position))
    if lower == upper:
        return ordered[lower]
    fraction = position - lower
    return ordered[lower] * (1.0 - fraction) + ordered[upper] * fraction


def _accepted_fit(points):
    try:
        fit = reference_plane.fit_plane(points, min_points=DEFAULT_MIN_POINTS)
    except reference_plane.ReferencePlaneError:
        return None
    if (
        fit.rms_residual_mm > DEFAULT_MAX_RMS_MM
        or fit.max_abs_residual_mm > DEFAULT_MAX_RESIDUAL_MM
    ):
        return None
    return fit


def analyse_planes(replay: replay_view.ReplayData, *, window: int):
    if int(window) < 1:
        raise reference_plane.ReferencePlaneError("plane window must be at least one frame")
    fits = tuple(_accepted_fit(frame.points_reference) for frame in replay.frames)
    accepted = [fit for fit in fits if fit is not None]
    if len(accepted) < 10:
        raise reference_plane.ReferencePlaneError(
            f"only {len(accepted)} accepted plane fits remain; at least 10 are required"
        )

    baseline = reference_plane.mean_direction(fit.normal for fit in accepted)
    baseline_rx, baseline_ry = _normal_rx_ry(baseline)
    diagnostics = []
    for index, fit in enumerate(fits):
        start = max(0, index - int(window) + 1)
        window_fits = [item for item in fits[start : index + 1] if item is not None]
        if not window_fits:
            diagnostics.append(PlaneDiagnostic(fit, None, None, None, None))
            continue
        rolling = reference_plane.mean_direction(item.normal for item in window_fits)
        rx, ry = _normal_rx_ry(rolling)
        diagnostics.append(
            PlaneDiagnostic(
                fit=fit,
                rolling_normal=rolling,
                angular_error_deg=reference_plane.angle_deg(rolling, baseline),
                delta_rx_deg=rx - baseline_rx,
                delta_ry_deg=ry - baseline_ry,
            )
        )
    return fits, tuple(diagnostics), baseline


def _valid_points(points):
    return [point for point in points if point is not None]


def _centroid(points):
    valid = _valid_points(points)
    if not valid:
        raise reference_plane.ReferencePlaneError("frame contains no valid points")
    count = float(len(valid))
    return (
        sum(point.x_mm for point in valid) / count,
        sum(point.y_mm for point in valid) / count,
        sum(point.z_mm for point in valid) / count,
    )


def _plane_corners(points, normal):
    valid = _valid_points(points)
    centroid = _centroid(points)
    xs = [point.x_mm for point in valid]
    ys = [point.y_mm for point in valid]
    x_low, x_high = min(xs), max(xs)
    y_low, y_high = min(ys), max(ys)
    x_pad = max((x_high - x_low) * 0.08, 5.0)
    y_pad = max((y_high - y_low) * 0.08, 5.0)
    x_low -= x_pad
    x_high += x_pad
    y_low -= y_pad
    y_high += y_pad
    nx, ny, nz = normal
    if abs(nz) < 0.2:
        return None, centroid

    def z_at(x, y):
        return centroid[2] - (nx * (x - centroid[0]) + ny * (y - centroid[1])) / nz

    corners = [
        (x_low, y_low, z_at(x_low, y_low)),
        (x_high, y_low, z_at(x_high, y_low)),
        (x_high, y_high, z_at(x_high, y_high)),
        (x_low, y_high, z_at(x_low, y_high)),
    ]
    return corners, centroid


def import_matplotlib():
    try:
        import matplotlib.pyplot as plt
        from matplotlib.animation import FuncAnimation
        from mpl_toolkits.mplot3d.art3d import Poly3DCollection
    except ModuleNotFoundError as exc:
        raise RuntimeError(
            "Matplotlib is required for wall-plane viewing: py -m pip install matplotlib"
        ) from exc
    return plt, FuncAnimation, Poly3DCollection


def build_figure(replay, diagnostics, baseline, *, fps, window):
    plt, FuncAnimation, Poly3DCollection = import_matplotlib()
    fig = plt.figure(figsize=(15, 7))
    ax_cloud = fig.add_subplot(121, projection="3d")
    ax_error = fig.add_subplot(122)

    ref_sequences = [frame.points_reference for frame in replay.frames]
    limits = tuple(
        replay_view._axis_limits(ref_sequences, axis)
        for axis in ("x_mm", "y_mm", "z_mm")
    )
    replay_view._configure_axis(
        ax_cloud,
        title=f"local_reference wall + rolling {window}-frame plane",
        frame_name="local_reference",
        limits=limits,
    )
    scatter = ax_cloud.scatter([], [], [], s=28)

    times = [frame.time_s for frame in replay.frames]
    rx_values = [item.delta_rx_deg if item.delta_rx_deg is not None else math.nan for item in diagnostics]
    ry_values = [item.delta_ry_deg if item.delta_ry_deg is not None else math.nan for item in diagnostics]
    angle_values = [
        item.angular_error_deg if item.angular_error_deg is not None else math.nan
        for item in diagnostics
    ]
    ax_error.plot(times, rx_values, label="ΔRx normal tilt")
    ax_error.plot(times, ry_values, label="ΔRy normal tilt")
    ax_error.plot(times, angle_values, label="total normal error")
    ax_error.axhline(0.0, linewidth=0.8)
    cursor = ax_error.axvline(times[0], linewidth=1.0)
    ax_error.set_xlabel("orientation time (s)")
    ax_error.set_ylabel("angle from capture mean (deg)")
    ax_error.set_title("Rolling plane-normal stability")
    ax_error.grid(True, alpha=0.25)
    ax_error.legend()

    baseline_rx, baseline_ry = _normal_rx_ry(baseline)
    status = fig.text(0.5, 0.02, "", ha="center", va="bottom")
    plane_artist = [None]
    normal_artist = [None]

    def update(index):
        frame = replay.frames[index]
        diagnostic = diagnostics[index]
        xs, ys, zs = replay_view._valid_xyz(frame.points_reference)
        scatter._offsets3d = (xs, ys, zs)
        cursor.set_xdata([frame.time_s, frame.time_s])

        if plane_artist[0] is not None:
            plane_artist[0].remove()
            plane_artist[0] = None
        if normal_artist[0] is not None:
            normal_artist[0].remove()
            normal_artist[0] = None

        if diagnostic.rolling_normal is not None:
            corners, centroid = _plane_corners(frame.points_reference, diagnostic.rolling_normal)
            if corners is not None:
                plane = Poly3DCollection([corners], alpha=0.20)
                ax_cloud.add_collection3d(plane)
                plane_artist[0] = plane
                scale = 120.0
                nx, ny, nz = diagnostic.rolling_normal
                normal_artist[0] = ax_cloud.quiver(
                    centroid[0], centroid[1], centroid[2],
                    nx * scale, ny * scale, nz * scale,
                    normalize=False,
                )

        if diagnostic.fit is None:
            fit_text = "instant plane REJECTED by wall-quality screen"
        else:
            fit_text = (
                f"instant plane RMS {diagnostic.fit.rms_residual_mm:.2f} mm, "
                f"max {diagnostic.fit.max_abs_residual_mm:.2f} mm"
            )
        if diagnostic.angular_error_deg is None:
            angle_text = "rolling normal unavailable"
        else:
            angle_text = (
                f"rolling normal error {diagnostic.angular_error_deg:.3f}°  "
                f"ΔRx {diagnostic.delta_rx_deg:+.3f}°  ΔRy {diagnostic.delta_ry_deg:+.3f}°"
            )
        status.set_text(
            f"frame {frame.source_index}/{replay.total_tof_grids - 1}  t={frame.time_s:.3f}s  "
            f"{fit_text}  |  {angle_text}"
        )
        artists = [scatter, cursor, status]
        if plane_artist[0] is not None:
            artists.append(plane_artist[0])
        if normal_artist[0] is not None:
            artists.append(normal_artist[0])
        return tuple(artists)

    animation = FuncAnimation(
        fig,
        update,
        frames=len(replay.frames),
        interval=1000.0 / float(fps),
        blit=False,
        repeat=True,
    )
    update(0)
    fig.suptitle(
        "Rangeweave local-reference flat-wall diagnostic\n"
        f"baseline normal Rx {baseline_rx:+.3f}°, Ry {baseline_ry:+.3f}° — "
        "plane normal constrains two rotational axes only"
    )
    fig.tight_layout(rect=(0.0, 0.07, 1.0, 0.93))
    return plt, fig, animation


def print_summary(replay, diagnostics, baseline, *, window, fps):
    accepted = [item.fit for item in diagnostics if item.fit is not None]
    angular = [
        item.angular_error_deg
        for item in diagnostics
        if item.angular_error_deg is not None
    ]
    rx = [item.delta_rx_deg for item in diagnostics if item.delta_rx_deg is not None]
    ry = [item.delta_ry_deg for item in diagnostics if item.delta_ry_deg is not None]
    baseline_rx, baseline_ry = _normal_rx_ry(baseline)
    rms_angle = math.sqrt(sum(value * value for value in angular) / len(angular))

    print("Rangeweave local-reference wall-plane diagnostic")
    print(f"  capture:          {replay.packets_path}")
    print(f"  stream health:    {'PASS' if replay.stream_clean else 'NOT CLEAN'}")
    print(f"  replay frames:    {len(replay.frames)} / {replay.total_tof_grids}")
    print(f"  accepted planes:  {len(accepted)} / {len(replay.frames)}")
    print(f"  rolling window:   {int(window)} frames")
    print(f"  playback rate:    {float(fps):.3f} fps")
    print(f"  baseline normal:  X {baseline[0]:+.6f}  Y {baseline[1]:+.6f}  Z {baseline[2]:+.6f}")
    print(f"  baseline tilt:    Rx {baseline_rx:+.3f} deg  Ry {baseline_ry:+.3f} deg")
    print("  rolling normal error vs capture mean:")
    print(f"    median:          {statistics.median(angular):.3f} deg")
    print(f"    RMS:             {rms_angle:.3f} deg")
    print(f"    p95:             {_percentile(angular, 0.95):.3f} deg")
    print(f"    max:             {max(angular):.3f} deg")
    print(f"    |Delta Rx| p95:  {_percentile([abs(value) for value in rx], 0.95):.3f} deg")
    print(f"    |Delta Ry| p95:  {_percentile([abs(value) for value in ry], 0.95):.3f} deg")
    print(
        "  scope:            orientation-only flat-wall diagnostic; plane offset/position "
        "is not interpreted because sensing-head translation is not estimated"
    )
    print(
        "  observability:    a featureless plane constrains its normal (two rotational "
        "degrees of freedom), not rotation about that normal"
    )


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Replay a known flat wall with a rolling best-fit local-reference plane"
    )
    parser.add_argument("capture", help="capture directory or packets.bin")
    parser.add_argument("--boresight-artifact", required=True)
    parser.add_argument("--fps", type=float, default=15.0)
    parser.add_argument("--plane-window", type=int, default=DEFAULT_PLANE_WINDOW)
    parser.add_argument("--geometry-profile", default=None, metavar="PATH")
    parser.add_argument(
        "--timing-mode", choices=tof_timing.MODES, default=tof_timing.MODE_QUICK_START
    )
    parser.add_argument("--timing-artifact")
    parser.add_argument("--timing-assembly-id")
    parser.add_argument("--tof-time-offset-ms", type=float, default=None)
    parser.add_argument("--no-show", action="store_true")
    args = parser.parse_args()

    if args.fps <= 0.0:
        parser.error("--fps must be greater than zero")
    if args.plane_window < 1:
        parser.error("--plane-window must be at least 1")

    try:
        replay = replay_view.build_replay_data(
            Path(args.capture),
            boresight_artifact=Path(args.boresight_artifact),
            timing_mode=args.timing_mode,
            timing_artifact=(Path(args.timing_artifact) if args.timing_artifact else None),
            timing_assembly_id=args.timing_assembly_id,
            tof_time_offset_ms=args.tof_time_offset_ms,
            geometry_profile_path=(Path(args.geometry_profile) if args.geometry_profile else None),
        )
        _, diagnostics, baseline = analyse_planes(replay, window=args.plane_window)
        print_summary(replay, diagnostics, baseline, window=args.plane_window, fps=args.fps)
        plt, fig, animation = build_figure(
            replay,
            diagnostics,
            baseline,
            fps=args.fps,
            window=args.plane_window,
        )
        fig._rangeweave_animation = animation
    except (
        OSError,
        RuntimeError,
        ValueError,
        orientation.OrientationError,
        oriented.OrientedGeometryError,
        geometry.GeometryError,
        reference_plane.ReferencePlaneError,
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
