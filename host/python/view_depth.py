"""Raw depth/health viewer for Rangeweave v0.1 captures.

The command-line summary uses the standard-library-only rangeweave_depth analysis
core. Matplotlib is imported only when a graphical figure is actually requested.
"""

from __future__ import annotations

import argparse
import math
from pathlib import Path
import statistics

import rangeweave_depth as depth
import rangeweave_protocol as rw


def decode_text_tlv(info: rw.StreamInfo | None, tag: int) -> str | None:
    if info is None:
        return None
    value = info.first_value(tag)
    if value is None:
        return None
    return value.decode("utf-8", "replace")


def print_float_grid(title, values, rows, cols, *, width=8, precision=1) -> None:
    print()
    print(title)
    for row_index, row in enumerate(depth.as_rows(values, rows, cols)):
        rendered = []
        for value in row:
            if math.isfinite(float(value)):
                rendered.append(f"{float(value):{width}.{precision}f}")
            else:
                rendered.append(f"{'---':>{width}}")
        print(f"  r{row_index:02d}: " + " ".join(rendered))


def print_int_grid(title, values, rows, cols, *, width=7) -> None:
    print()
    print(title)
    for row_index, row in enumerate(depth.as_rows(values, rows, cols)):
        print(f"  r{row_index:02d}: " + " ".join(f"{int(v):{width}d}" for v in row))


def selected_frame(analysis, requested_index):
    frame_count = len(analysis.tof_frames)
    if requested_index < -frame_count or requested_index >= frame_count:
        raise depth.DepthAnalysisError(
            "ToF frame index {} outside capture range {}..{}".format(
                requested_index, -frame_count, frame_count - 1
            )
        )
    return requested_index % frame_count, analysis.frame(requested_index)


def print_summary(analysis, *, selected_index, selected) -> None:
    stats = analysis.stream_stats
    firmware = decode_text_tlv(stats.last_info, rw.INFO_FIRMWARE_LABEL)
    source_profile = decode_text_tlv(stats.last_info, rw.INFO_SOURCE_PROFILE)
    metadata_status = (
        "PASS" if analysis.input_path.is_dir() and not analysis.metadata_errors
        else "FAIL" if analysis.input_path.is_dir()
        else "N/A"
    )

    print("Rangeweave raw depth/health summary")
    print(f"  packets:       {analysis.packets_path}")
    print(f"  bytes:         {analysis.packets_bytes}")
    print(f"  SHA-256:       {analysis.packets_sha256}")
    print(f"  valid frames:  {analysis.decoder.frames_ok}")
    print(f"  bad frames:    {analysis.decoder.frames_bad}")
    print(f"  semantic errs: {stats.semantic_errors}")
    print(f"  seq gaps:      {stats.sequence_gaps}")
    print(f"  ToF frames:    {len(analysis.tof_frames)}")
    print(f"  grid:          {analysis.rows}x{analysis.cols}")
    if analysis.layout_id == 0:
        print("  layout_id:     0 (producer-native flattened zone order)")
    else:
        print(f"  layout_id:     {analysis.layout_id}")
    print("  field masks:   " + ", ".join(f"0x{m:04X}" for m in analysis.field_masks))
    if analysis.observed_ready_rate_hz is not None:
        print(
            f"  observed rate: {analysis.observed_ready_rate_hz:.4f} Hz "
            "from MCU ready-observation timestamps"
        )
    if analysis.read_duration_us.count[0]:
        print(f"  mean read:     {analysis.read_duration_us.mean[0]:.1f} us")
    print(f"  selected ToF:  frame {selected_index} / {len(analysis.tof_frames) - 1}")
    if firmware:
        print(f"  firmware:      {firmware}")
    if source_profile:
        print(f"  source:        {source_profile}")
    print(f"  metadata:      {metadata_status}")
    print("  validity rule: distance_mm > 0")
    for error in analysis.metadata_errors:
        print(f"    metadata mismatch: {error}")

    health = stats.health_deltas()
    if health:
        print("  STATUS deltas:")
        for key in (
            "frames_dropped", "imu_samples_dropped", "fifo_overruns",
            "fifo_structural_errors", "mag_errors", "tof_errors",
            "clock_sync_errors",
        ):
            print(f"    {key + ':':24s} {health.get(key, 0)}")

    print_float_grid(
        "Distance valid-only mean (mm), producer-native rows/columns",
        analysis.distance.mean, analysis.rows, analysis.cols,
    )
    print_float_grid(
        "Distance valid-only population stddev (mm), producer-native rows/columns",
        analysis.distance.stddev, analysis.rows, analysis.cols,
    )
    print_int_grid(
        "Valid distance sample count, producer-native rows/columns",
        analysis.distance.count, analysis.rows, analysis.cols,
    )
    print_float_grid(
        "Invalid distance samples (%), producer-native rows/columns",
        analysis.distance.invalid_percent, analysis.rows, analysis.cols,
        width=7, precision=1,
    )
    if analysis.reflectance is not None:
        print_float_grid(
            "Reflectance mean (%) on valid distance samples, producer-native rows/columns",
            analysis.reflectance.mean, analysis.rows, analysis.cols,
        )

    print()
    print("Mean-distance plane fit")
    plane = analysis.mean_distance_plane
    if plane is None:
        print("  unavailable: fewer than 3 sufficiently valid non-collinear zones")
    else:
        print(
            "  model:         distance_mm = "
            f"{plane.intercept_mm:.3f} {plane.row_slope_mm:+.3f}*row "
            f"{plane.column_slope_mm:+.3f}*column"
        )
        print(
            f"  zones used:    {plane.zones_used} / {analysis.zones} "
            f"(>= {plane.min_valid_fraction * 100:.0f}% valid)"
        )
        print(f"  residual RMS:  {plane.rms_residual_mm:.3f} mm")
        print(f"  max |residual|:{plane.max_abs_residual_mm:8.3f} mm")
        print_float_grid(
            "Mean-distance plane residual (mm), producer-native rows/columns",
            plane.residual_mm, analysis.rows, analysis.cols,
        )

    if selected.distance_mm is not None:
        print_int_grid(
            "Selected frame distance (mm), producer-native rows/columns",
            selected.distance_mm, analysis.rows, analysis.cols,
        )


def add_grid_plot(fig, position, grid, title, value_format) -> None:
    ax = fig.add_subplot(2, 3, position)
    image = ax.imshow(grid)
    ax.set_title(title)
    ax.set_xlabel("producer-native column")
    ax.set_ylabel("producer-native row")
    ax.set_xticks(range(len(grid[0])))
    ax.set_yticks(range(len(grid)))
    finite = [float(v) for row in grid for v in row if math.isfinite(float(v))]
    midpoint = statistics.median(finite) if finite else 0.0
    for r, row in enumerate(grid):
        for c, raw in enumerate(row):
            value = float(raw)
            label = format(value, value_format) if math.isfinite(value) else "—"
            ax.text(
                c, r, label, ha="center", va="center", fontsize=7,
                color="white" if math.isfinite(value) and value < midpoint else "black",
            )
    fig.colorbar(image, ax=ax, shrink=0.80)


def build_figure(analysis, selected):
    try:
        import matplotlib.pyplot as plt
    except ModuleNotFoundError as exc:
        raise RuntimeError(
            "Matplotlib is required for the graphical viewer; install it or use --summary-only"
        ) from exc

    fig = plt.figure(figsize=(16, 9))
    fig.suptitle(f"Rangeweave raw depth/health viewer — {analysis.input_path.name}")
    if selected.distance_mm is None:
        selected_grid = [[float("nan")] * analysis.cols for _ in range(analysis.rows)]
    else:
        selected_grid = [
            [float(v) if int(v) > depth.DISTANCE_INVALID_MM else float("nan") for v in row]
            for row in depth.as_rows(selected.distance_mm, analysis.rows, analysis.cols)
        ]
    add_grid_plot(fig, 1, selected_grid, "Selected distance (mm)", ".0f")
    add_grid_plot(fig, 2, depth.as_rows(analysis.distance.mean, analysis.rows, analysis.cols), "Valid-only mean distance (mm)", ".1f")
    add_grid_plot(fig, 3, depth.as_rows(analysis.distance.stddev, analysis.rows, analysis.cols), "Valid-only population σ (mm)", ".1f")
    add_grid_plot(fig, 4, depth.as_rows(analysis.distance.invalid_percent, analysis.rows, analysis.cols), "Invalid distance samples (%)", ".1f")
    if analysis.mean_distance_plane is not None:
        add_grid_plot(fig, 5, depth.as_rows(analysis.mean_distance_plane.residual_mm, analysis.rows, analysis.cols), "Mean-plane residual (mm)", "+.1f")
    else:
        ax = fig.add_subplot(2, 3, 5); ax.axis("off"); ax.text(0.5, 0.5, "Insufficient valid zones\nfor plane fit", ha="center", va="center")
    if analysis.reflectance is not None:
        add_grid_plot(fig, 6, depth.as_rows(analysis.reflectance.mean, analysis.rows, analysis.cols), "Reflectance mean on valid ranges (%)", ".1f")
    else:
        ax = fig.add_subplot(2, 3, 6); ax.axis("off"); ax.text(0.5, 0.5, "No reflectance field", ha="center", va="center")
    fig.tight_layout(rect=(0, 0, 1, 0.96))
    return plt, fig


def main() -> int:
    parser = argparse.ArgumentParser(description="Rangeweave raw depth/health viewer")
    parser.add_argument("capture", help="capture directory or packets.bin")
    parser.add_argument("--frame", type=int, default=-1, help="ToF frame index; negative indices count from end (default: -1)")
    parser.add_argument("--summary-only", action="store_true", help="print diagnostics without importing Matplotlib")
    parser.add_argument("--save", default=None, help="save viewer figure to this path")
    parser.add_argument("--no-show", action="store_true", help="do not open an interactive Matplotlib window")
    args = parser.parse_args()
    if args.summary_only and args.save:
        parser.error("--save cannot be used with --summary-only")
    try:
        analysis = depth.analyse_capture(Path(args.capture))
        selected_index, selected = selected_frame(analysis, args.frame)
    except (OSError, depth.DepthAnalysisError) as exc:
        parser.error(str(exc))
    print_summary(analysis, selected_index=selected_index, selected=selected)
    if args.summary_only:
        return 0
    try:
        plt, fig = build_figure(analysis, selected)
    except RuntimeError as exc:
        parser.error(str(exc))
    if args.save:
        save_path = Path(args.save)
        fig.savefig(save_path, dpi=160)
        print()
        print(f"Saved viewer figure: {save_path}")
    if args.no_show:
        plt.close(fig)
    else:
        plt.show()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
