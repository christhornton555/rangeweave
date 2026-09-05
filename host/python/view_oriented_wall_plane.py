"""Visualize a flat wall as a centred best-fit plane in ``local_reference``.

This diagnostic is for captures known to contain one approximately planar target.
It complements ``view_oriented_point_cloud.py`` by fitting an orthogonal plane to
each orientation-compensated ToF frame, averaging accepted plane normals over a
short centred window, and showing the resulting residual orientation over time.

Because this is offline replay, the smoothing is deliberately non-causal: a
centred window avoids the phase lag of the earlier trailing average.  The
instantaneous plane-normal error is retained underneath the smoothed trace so the
actual frame-by-frame behaviour remains visible.

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
    instant_angular_error_deg: float | None
    instant_delta_rx_deg: float | None
    instant_delta_ry_deg: float | None
    smoothed_normal: reference_plane.Vector3 | None
    smoothed_angular_error_deg: float | None
    smoothed_delta_rx_deg: float | None
    smoothed_delta_ry_deg: float | None


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


def _centred_window_fits(fits, index: int, window: int):
    """Return accepted fits in one full centred odd-width window.

    Edge frames without a complete centred window return an empty tuple.  Rejected
    plane fits inside the window are omitted, but at least a majority of the
    requested window must remain before a smoothed normal is reported.
    """

    width = int(window)
    if width < 1:
        raise reference_plane.ReferencePlaneError("plane window must be at least one frame")
    if width % 2 == 0:
        raise reference_plane.ReferencePlaneError(
            "centred plane window must be odd (for example 1, 3, 5 or 7)"
        )
    half = width // 2
    start = int(index) - half
    end = int(index) + half + 1
    if start < 0 or end > len(fits):
        return ()
    accepted = tuple(item for item in fits[start:end] if item is not None)
    minimum = half + 1
    return accepted if len(accepted) >= minimum else ()


def analyse_planes(replay: replay_view.ReplayData, *, window: int):
    width = int(window)
    if width < 1:
        raise reference_plane.ReferencePlaneError("plane window must be at least one frame")
    if width % 2 == 0:
        raise reference_plane.ReferencePlaneError(
            "centred plane window must be odd (for example 1, 3, 5 or 7)"
        )

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
        if fit is None:
            instant_angle = None
            instant_rx = None
            instant_ry = None
        else:
            fit_rx, fit_ry = _normal_rx_ry(fit.normal)
            instant_angle = reference_plane.angle_deg(fit.normal, baseline)
            instant_rx = fit_rx - baseline_rx
            instant_ry = fit_ry - baseline_ry

        window_fits = _centred_window_fits(fits, index, width)
        if not window_fits:
            diagnostics.append(
                PlaneDiagnostic(
                    fit=fit,
                    instant_angular_error_deg=instant_angle,
                    instant_delta_rx_deg=instant_rx,
                    instant_delta_ry_deg=instant_ry,
                    smoothed_normal=None,
                    smoothed_angular_error_deg=None,
                    smoothed_delta_rx_deg=None,
                    smoothed_delta_ry_deg=None,
                )
            )
            continue

        smoothed = reference_plane.mean_direction(item.normal for item in window_fits)
        smooth_rx, smooth_ry = _normal_rx_ry(smoothed)
        diagnostics.append(
            PlaneDiagnostic(
                fit=fit,
                instant_angular_error_deg=instant_angle,
                instant_delta_rx_deg=instant_rx,
                instant_delta_ry_deg=instant_ry,
                smoothed_normal=smoothed,
                smoothed_angular_error_deg=reference_plane.angle_deg(smoothed, baseline),
                smoothed_delta_rx_deg=smooth_rx - baseline_rx,
                smoothed_delta_ry_deg=smooth_ry - baseline_ry,
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
        title=f"local_reference wall + centred {window}-frame plane",
        frame_name="local_reference",
        limits=limits,
    )
    scatter = ax_cloud.scatter([], [], [], s=28)

    times = [frame.time_s for frame in replay.frames]
    instant_angle_values = [
        item.instant_angular_error_deg
        if item.instant_angular_error_deg is not None
        else math.nan
        for item in diagnostics
    ]
    rx_values = [
        item.smoothed_delta_rx_deg if item.smoothed_delta_rx_deg is not None else math.nan
        for item in diagnostics
    ]
    ry_values = [
        item.smoothed_delta_ry_deg if item.smoothed_delta_ry_deg is not None else math.nan
        for item in diagnostics
    ]
    angle_values = [
        item.smoothed_angular_error_deg
        if item.smoothed_angular_error_deg is not None
        else math.nan
        for item in diagnostics
    ]
    ax_error.plot(
        times,
        instant_angle_values,
        label="instant total normal error",
        linewidth=1.0,
        alpha=0.25,
    )
    ax_error.plot(times, rx_values, label="centred ΔRx normal tilt")
    ax_error.plot(times, ry_values, label="centred ΔRy normal tilt")
    ax_error.plot(times, angle_values, label="centred total normal error")
    ax_error.axhline(0.0, linewidth=0.8)
    cursor = ax_error.axvline(times[0], linewidth=1.0)
    ax_error.set_xlabel("orientation time (s)")
    ax_error.set_ylabel("angle from capture mean (deg)")
    ax_error.set_title("Centred plane-normal stability")
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

        if diagnostic.smoothed_normal is not None:
            corners, centroid = _plane_corners(frame.points_reference, diagnostic.smoothed_normal)
            if corners is not None:
                plane = Poly3DCollection([corners], alpha=0.20)
                ax_cloud.add_collection3d(plane)
                plane_artist[0] = plane
                scale = 120.0
                nx, ny, nz = diagnostic.smoothed_normal
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
                f"max {diagnostic.fit.max_abs_residual_mm:.2f} mm, "
                f"normal error {diagnostic.instant_angular_error_deg:.3f}°"
            )
        if diagnostic.smoothed_angular_error_deg is None:
            angle_text = "centred normal unavailable"
        else:
            angle_text = (
                f"centred normal error {diagnostic.smoothed_angular_error_deg:.3f}°  "
                f"ΔRx {diagnostic.smoothed_delta_rx_deg:+.3f}°  "
                f"ΔRy {diagnostic.smoothed_delta_ry_deg:+.3f}°"
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
        f"centred {window}-frame smoothing; plane normal constrains two rotational axes"
    )
    fig.tight_layout(rect=(0.0, 0.07, 1.0, 0.93))
    return plt, fig, animation


def _error_stats(values):
    values = [float(value) for value in values if value is not None]
    if not values:
        raise reference_plane.ReferencePlaneError("no plane-normal errors are available")
    return {
        "count": len(values),
        "median": statistics.median(values),
        "rms": math.sqrt(sum(value * value for value in values) / len(values)),
        "p95": _percentile(values, 0.95),
        "max": max(values),
    }


def print_summary(replay, diagnostics, baseline, *, window, fps):
    accepted = [item.fit for item in diagnostics if item.fit is not None]
    instant = [item.instant_angular_error_deg for item in diagnostics]
    smoothed = [item.smoothed_angular_error_deg for item in diagnostics]
    rx = [item.smoothed_delta_rx_deg for item in diagnostics if item.smoothed_delta_rx_deg is not None]
    ry = [item.smoothed_delta_ry_deg for item in diagnostics if item.smoothed_delta_ry_deg is not None]
    instant_stats = _error_stats(instant)
    smooth_stats = _error_stats(smoothed)
    baseline_rx, baseline_ry = _normal_rx_ry(baseline)

    print("Rangeweave local-reference wall-plane diagnostic")
    print(f"  capture:          {replay.packets_path}")
    print(f"  stream health:    {'PASS' if replay.stream_clean else 'NOT CLEAN'}")
    print(f"  replay frames:    {len(replay.frames)} / {replay.total_tof_grids}")
    print(f"  accepted planes:  {len(accepted)} / {len(replay.frames)}")
    print(f"  centred window:   {int(window)} frames")
    print(f"  playback rate:    {float(fps):.3f} fps")
    print(f"  baseline normal:  X {baseline[0]:+.6f}  Y {baseline[1]:+.6f}  Z {baseline[2]:+.6f}")
    print(f"  baseline tilt:    Rx {baseline_rx:+.3f} deg  Ry {baseline_ry:+.3f} deg")
    print("  instantaneous normal error vs capture mean:")
    print(f"    samples:         {instant_stats['count']}")
    print(f"    median:          {instant_stats['median']:.3f} deg")
    print(f"    RMS:             {instant_stats['rms']:.3f} deg")
    print(f"    p95:             {instant_stats['p95']:.3f} deg")
    print(f"    max:             {instant_stats['max']:.3f} deg")
    print(f"  centred {int(window)}-frame normal error vs capture mean:")
    print(f"    samples:         {smooth_stats['count']}")
    print(f"    median:          {smooth_stats['median']:.3f} deg")
    print(f"    RMS:             {smooth_stats['rms']:.3f} deg")
    print(f"    p95:             {smooth_stats['p95']:.3f} deg")
    print(f"    max:             {smooth_stats['max']:.3f} deg")
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
        description="Replay a known flat wall with a centred best-fit local-reference plane"
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
    if args.plane_window % 2 == 0:
        parser.error("--plane-window must be odd for centred smoothing")

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
