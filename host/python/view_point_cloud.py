"""Project one VL53L5CX frame into Rangeweave tof_optical and optionally plot it."""

from __future__ import annotations

import argparse
from pathlib import Path

import rangeweave_depth as depth
import rangeweave_geometry as geometry


DEFAULT_MAX_LINK_DZ_MM = 150.0


def selected_frame(analysis, requested_index):
    frame_count = len(analysis.tof_frames)
    if requested_index < -frame_count or requested_index >= frame_count:
        raise depth.DepthAnalysisError(
            f"ToF frame index {requested_index} outside capture range "
            f"{-frame_count}..{frame_count - 1}"
        )
    index = requested_index % frame_count
    return index, analysis.tof_frames[index]


def project_frame(frame):
    if frame.distance_mm is None:
        raise geometry.GeometryError("selected ToF frame has no distance field")
    return geometry.project_distances_mm(frame.distance_mm)


def valid_points(points):
    return [(zone_id, point) for zone_id, point in enumerate(points) if point is not None]


def connected_runs(points, max_link_dz_mm):
    """Split one grid row/column at invalid points and large depth discontinuities."""
    threshold = float(max_link_dz_mm)
    if threshold <= 0.0:
        raise ValueError("max_link_dz_mm must be greater than zero")

    runs = []
    run = []
    previous = None
    for point in list(points) + [None]:
        if point is None:
            if len(run) >= 2:
                runs.append(run)
            run = []
            previous = None
            continue

        if previous is not None and abs(point.z_mm - previous.z_mm) > threshold:
            if len(run) >= 2:
                runs.append(run)
            run = [point]
        else:
            run.append(point)
        previous = point

    return runs


def print_summary(analysis, frame_index, points, max_link_dz_mm) -> None:
    valid = valid_points(points)
    if not valid:
        raise geometry.GeometryError("selected frame contains no valid projected distances")

    xs = [point.x_mm for _, point in valid]
    ys = [point.y_mm for _, point in valid]
    zs = [point.z_mm for _, point in valid]

    print("Rangeweave 64-zone point projection")
    print(f"  capture:        {analysis.input_path}")
    print(f"  frame:          {frame_index} / {len(analysis.tof_frames) - 1}")
    print(f"  geometry model: {geometry.GEOMETRY_MODEL}")
    print(f"  profile role:   {geometry.GEOMETRY_PROFILE_ROLE}")
    print(f"  valid points:   {len(valid)} / {geometry.ZONE_COUNT}")
    print("  frame axes:     tof_optical +X right, +Y down, +Z forward")
    print("  distance rule:  VL53L5CX distance_mm is axial Z, not slant range")
    print(f"  mesh link dZ:   <= {float(max_link_dz_mm):.1f} mm")
    print(f"  X extent:       {min(xs):.1f} .. {max(xs):.1f} mm")
    print(f"  Y extent:       {min(ys):.1f} .. {max(ys):.1f} mm")
    print(f"  Z extent:       {min(zs):.1f} .. {max(zs):.1f} mm")


def import_matplotlib():
    try:
        import matplotlib.pyplot as plt
    except ModuleNotFoundError as exc:
        raise RuntimeError(
            "Matplotlib is required for graphical point-cloud viewing: "
            "py -m pip install matplotlib"
        ) from exc
    return plt


def _physical_grid_points(points):
    grid = [[None for _ in range(geometry.ZONE_COLS)] for _ in range(geometry.ZONE_ROWS)]
    for zone_id, point in enumerate(points):
        physical_row, physical_col = geometry.physical_row_col(zone_id)
        grid[physical_row][physical_col] = point
    return grid


def _grid_runs(grid, max_link_dz_mm):
    for row in grid:
        yield from connected_runs(row, max_link_dz_mm)

    for col in range(geometry.ZONE_COLS):
        column = [grid[row][col] for row in range(geometry.ZONE_ROWS)]
        yield from connected_runs(column, max_link_dz_mm)


def _plot_run_3d(ax, run):
    ax.plot(
        [point.x_mm for point in run],
        [point.y_mm for point in run],
        [point.z_mm for point in run],
        linewidth=0.8,
        alpha=0.5,
    )


def _plot_run_front(ax, run):
    ax.plot(
        [point.x_mm for point in run],
        [point.y_mm for point in run],
        linewidth=0.8,
        alpha=0.35,
    )


def build_figure(analysis, frame_index, points, max_link_dz_mm=DEFAULT_MAX_LINK_DZ_MM):
    plt = import_matplotlib()
    fig = plt.figure(figsize=(15, 7))
    ax_3d = fig.add_subplot(121, projection="3d")
    ax_front = fig.add_subplot(122)

    valid = valid_points(points)
    xs = [point.x_mm for _, point in valid]
    ys = [point.y_mm for _, point in valid]
    zs = [point.z_mm for _, point in valid]
    grid = _physical_grid_points(points)
    runs = list(_grid_runs(grid, max_link_dz_mm))

    ax_3d.scatter(xs, ys, zs, c=zs, s=30)
    for run in runs:
        _plot_run_3d(ax_3d, run)

    ax_3d.set_xlabel("tof_optical X (mm) — scene right")
    ax_3d.set_ylabel("tof_optical Y (mm) — down")
    ax_3d.set_zlabel("tof_optical Z (mm) — forward")
    ax_3d.set_title(
        f"3D projection — frame {frame_index}\n"
        f"mesh breaks when |dZ| > {float(max_link_dz_mm):g} mm"
    )

    x_span = max(xs) - min(xs) if len(xs) > 1 else 1.0
    y_span = max(ys) - min(ys) if len(ys) > 1 else 1.0
    z_span = max(zs) - min(zs) if len(zs) > 1 else 1.0
    ax_3d.set_box_aspect((max(x_span, 1.0), max(y_span, 1.0), max(z_span, 1.0)))
    ax_3d.view_init(elev=22, azim=-55)

    depth_scatter = ax_front.scatter(xs, ys, c=zs, s=55)
    for run in runs:
        _plot_run_front(ax_front, run)
    ax_front.set_xlabel("tof_optical X (mm) — scene right")
    ax_front.set_ylabel("tof_optical Y (mm) — down")
    ax_front.set_title("Front-on geometry — colour = axial Z")
    ax_front.set_aspect("equal", adjustable="box")
    ax_front.invert_yaxis()
    ax_front.grid(True, alpha=0.25)
    fig.colorbar(depth_scatter, ax=ax_front, label="tof_optical Z (mm) — forward")

    fig.suptitle(
        f"Rangeweave 64-zone projection — {analysis.input_path.name}",
        fontsize=12,
    )
    fig.tight_layout()
    return plt, fig


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Project one Rangeweave VL53L5CX frame into tof_optical XYZ"
    )
    parser.add_argument("capture", help="capture directory or packets.bin")
    parser.add_argument(
        "--frame",
        type=int,
        default=-1,
        help="ToF frame index; negative indices count from end (default: -1)",
    )
    parser.add_argument(
        "--summary-only",
        action="store_true",
        help="print XYZ extents without importing Matplotlib",
    )
    parser.add_argument(
        "--max-link-dz-mm",
        type=float,
        default=DEFAULT_MAX_LINK_DZ_MM,
        help=(
            "do not draw a mesh edge across a larger neighbouring-Z jump "
            f"(default: {DEFAULT_MAX_LINK_DZ_MM:g} mm)"
        ),
    )
    parser.add_argument("--save", default=None, help="save point-cloud figure to this path")
    parser.add_argument("--no-show", action="store_true", help="do not open the figure")
    args = parser.parse_args()

    if args.summary_only and args.save:
        parser.error("--save cannot be used with --summary-only")
    if args.max_link_dz_mm <= 0.0:
        parser.error("--max-link-dz-mm must be greater than zero")

    try:
        analysis = depth.analyse_capture(Path(args.capture))
        frame_index, frame = selected_frame(analysis, args.frame)
        points = project_frame(frame)
        print_summary(analysis, frame_index, points, args.max_link_dz_mm)
    except (OSError, depth.DepthAnalysisError, geometry.GeometryError) as exc:
        parser.error(str(exc))

    if args.summary_only:
        return 0

    try:
        plt, fig = build_figure(
            analysis,
            frame_index,
            points,
            max_link_dz_mm=args.max_link_dz_mm,
        )
    except (RuntimeError, ValueError) as exc:
        parser.error(str(exc))

    if args.save:
        output = Path(args.save)
        fig.savefig(output, dpi=160)
        print(f"  saved figure:   {output}")

    if args.no_show:
        plt.close(fig)
    else:
        plt.show()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
