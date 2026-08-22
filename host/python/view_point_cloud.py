"""Project one VL53L5CX frame into Rangeweave tof_optical and optionally plot it."""

from __future__ import annotations

import argparse
from pathlib import Path

import rangeweave_depth as depth
import rangeweave_geometry as geometry


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


def print_summary(analysis, frame_index, points) -> None:
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
    print(f"  valid points:   {len(valid)} / {geometry.ZONE_COUNT}")
    print("  frame axes:     tof_optical +X right, +Y down, +Z forward")
    print("  distance rule:  VL53L5CX distance_mm is axial Z, not slant range")
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


def _plot_connected_run(ax, run):
    if len(run) < 2:
        return
    ax.plot(
        [point.x_mm for point in run],
        [point.y_mm for point in run],
        [point.z_mm for point in run],
        linewidth=0.8,
        alpha=0.5,
    )


def _plot_grid_lines(ax, grid) -> None:
    for row in grid:
        run = []
        for point in row + [None]:
            if point is None:
                _plot_connected_run(ax, run)
                run = []
            else:
                run.append(point)

    for col in range(geometry.ZONE_COLS):
        run = []
        for row in range(geometry.ZONE_ROWS + 1):
            point = None if row == geometry.ZONE_ROWS else grid[row][col]
            if point is None:
                _plot_connected_run(ax, run)
                run = []
            else:
                run.append(point)


def build_figure(analysis, frame_index, points):
    plt = import_matplotlib()
    fig = plt.figure(figsize=(10, 8))
    ax = fig.add_subplot(111, projection="3d")

    valid = valid_points(points)
    xs = [point.x_mm for _, point in valid]
    ys = [point.y_mm for _, point in valid]
    zs = [point.z_mm for _, point in valid]

    ax.scatter(xs, ys, zs, s=28)
    _plot_grid_lines(ax, _physical_grid_points(points))

    ax.set_xlabel("tof_optical X (mm) — scene right")
    ax.set_ylabel("tof_optical Y (mm) — down")
    ax.set_zlabel("tof_optical Z (mm) — forward")
    ax.set_title(
        f"Rangeweave 64-zone projection — {analysis.input_path.name} — frame {frame_index}"
    )

    x_span = max(xs) - min(xs) if len(xs) > 1 else 1.0
    y_span = max(ys) - min(ys) if len(ys) > 1 else 1.0
    z_span = max(zs) - min(zs) if len(zs) > 1 else 1.0
    ax.set_box_aspect((max(x_span, 1.0), max(y_span, 1.0), max(z_span, 1.0)))
    ax.view_init(elev=22, azim=-55)
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
    parser.add_argument("--save", default=None, help="save point-cloud figure to this path")
    parser.add_argument("--no-show", action="store_true", help="do not open the figure")
    args = parser.parse_args()

    if args.summary_only and args.save:
        parser.error("--save cannot be used with --summary-only")

    try:
        analysis = depth.analyse_capture(Path(args.capture))
        frame_index, frame = selected_frame(analysis, args.frame)
        points = project_frame(frame)
        print_summary(analysis, frame_index, points)
    except (OSError, depth.DepthAnalysisError, geometry.GeometryError) as exc:
        parser.error(str(exc))

    if args.summary_only:
        return 0

    try:
        plt, fig = build_figure(analysis, frame_index, points)
    except RuntimeError as exc:
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
