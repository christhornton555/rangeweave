"""Raw, temporal and live-adjacent depth viewer for Rangeweave v0.1 captures.

The command-line summary and temporal timing helpers remain dependency-light.
Matplotlib is imported only when a graphical figure, playback or video export is
actually requested.
"""

from __future__ import annotations

import argparse
import math
from pathlib import Path
import statistics
import time

import rangeweave_depth as depth
import rangeweave_protocol as rw
import rangeweave_temporal as temporal


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


def import_matplotlib():
    try:
        import matplotlib.pyplot as plt
    except ModuleNotFoundError as exc:
        raise RuntimeError(
            "Matplotlib is required for graphical viewing: py -m pip install matplotlib"
        ) from exc
    return plt


def frame_distance_grid(analysis, frame):
    if frame.distance_mm is None:
        return [[float("nan")] * analysis.cols for _ in range(analysis.rows)]
    return [
        [
            float(value) if int(value) > depth.DISTANCE_INVALID_MM else float("nan")
            for value in row
        ]
        for row in depth.as_rows(frame.distance_mm, analysis.rows, analysis.cols)
    ]


def temporal_depth_bounds(analysis, requested_min, requested_max):
    valid_values = [
        int(value)
        for frame in analysis.tof_frames
        if frame.distance_mm is not None
        for value in frame.distance_mm
        if int(value) > depth.DISTANCE_INVALID_MM
    ]
    if not valid_values:
        raise depth.DepthAnalysisError("capture contains no valid ToF distance samples")

    minimum = float(min(valid_values) if requested_min is None else requested_min)
    maximum = float(max(valid_values) if requested_max is None else requested_max)

    if not math.isfinite(minimum) or not math.isfinite(maximum) or minimum >= maximum:
        raise depth.DepthAnalysisError(
            "temporal depth scale requires finite --min-mm < --max-mm"
        )
    return minimum, maximum


def add_grid_plot(fig, position, grid, title, value_format) -> None:
    ax = fig.add_subplot(2, 3, position)
    image = ax.imshow(grid, interpolation="nearest", origin="upper")
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
    plt = import_matplotlib()
    fig = plt.figure(figsize=(16, 9))
    fig.suptitle(f"Rangeweave raw depth/health viewer — {analysis.input_path.name}")
    add_grid_plot(fig, 1, frame_distance_grid(analysis, selected), "Selected distance (mm)", ".0f")
    add_grid_plot(fig, 2, depth.as_rows(analysis.distance.mean, analysis.rows, analysis.cols), "Valid-only mean distance (mm)", ".1f")
    add_grid_plot(fig, 3, depth.as_rows(analysis.distance.stddev, analysis.rows, analysis.cols), "Valid-only population σ (mm)", ".1f")
    add_grid_plot(fig, 4, depth.as_rows(analysis.distance.invalid_percent, analysis.rows, analysis.cols), "Invalid distance samples (%)", ".1f")
    if analysis.mean_distance_plane is not None:
        add_grid_plot(fig, 5, depth.as_rows(analysis.mean_distance_plane.residual_mm, analysis.rows, analysis.cols), "Mean-plane residual (mm)", "+.1f")
    else:
        ax = fig.add_subplot(2, 3, 5)
        ax.axis("off")
        ax.text(0.5, 0.5, "Insufficient valid zones\nfor plane fit", ha="center", va="center")
    if analysis.reflectance is not None:
        add_grid_plot(fig, 6, depth.as_rows(analysis.reflectance.mean, analysis.rows, analysis.cols), "Reflectance mean on valid ranges (%)", ".1f")
    else:
        ax = fig.add_subplot(2, 3, 6)
        ax.axis("off")
        ax.text(0.5, 0.5, "No reflectance field", ha="center", va="center")
    fig.tight_layout(rect=(0, 0, 1, 0.96))
    return plt, fig


def build_temporal_figure(analysis, *, min_mm, max_mm, controls=True):
    plt = import_matplotlib()
    fig, ax = plt.subplots(figsize=(8, 8))
    fig.suptitle(f"Rangeweave temporal depth — {analysis.input_path.name}")
    grid = frame_distance_grid(analysis, analysis.tof_frames[0])
    image = ax.imshow(
        grid,
        interpolation="nearest",
        origin="upper",
        vmin=min_mm,
        vmax=max_mm,
    )
    ax.set_xlabel("producer-native column")
    ax.set_ylabel("producer-native row")
    ax.set_xticks(range(analysis.cols))
    ax.set_yticks(range(analysis.rows))
    colorbar = fig.colorbar(image, ax=ax, shrink=0.82)
    colorbar.set_label("distance (mm)")
    status = fig.text(0.5, 0.045, "", ha="center")
    if controls:
        fig.text(
            0.5,
            0.015,
            "Space pause/resume | ←/→ step | Home/End | Esc quit",
            ha="center",
            fontsize=9,
        )
    fig.tight_layout(rect=(0, 0.07 if controls else 0.05, 1, 0.95))
    return plt, fig, image, status


def update_temporal_frame(analysis, image, status, source_index, times):
    frame = analysis.tof_frames[source_index]
    image.set_data(frame_distance_grid(analysis, frame))
    status.set_text(
        "frame {}/{} | t={:.3f} s | MCU ready={} us".format(
            source_index,
            len(analysis.tof_frames) - 1,
            times[source_index],
            frame.mcu_ready_us,
        )
    )
    image.figure.canvas.draw_idle()


def play_capture(analysis, *, min_mm, max_mm) -> None:
    times = temporal.relative_ready_times_s(analysis.tof_frames)
    plt, fig, image, status = build_temporal_figure(
        analysis,
        min_mm=min_mm,
        max_mm=max_mm,
        controls=True,
    )

    state = {
        "index": 0,
        "paused": False,
        "quit": False,
        "retime": True,
    }

    def render(index):
        state["index"] = max(0, min(int(index), len(analysis.tof_frames) - 1))
        update_temporal_frame(analysis, image, status, state["index"], times)
        state["retime"] = True

    def on_key(event):
        key = (event.key or "").lower()
        if key in ("escape", "esc"):
            state["quit"] = True
            plt.close(fig)
        elif key == " ":
            if state["paused"] and state["index"] >= len(analysis.tof_frames) - 1:
                render(0)
            state["paused"] = not state["paused"]
            state["retime"] = True
        elif key == "left":
            state["paused"] = True
            render(state["index"] - 1)
        elif key == "right":
            state["paused"] = True
            render(state["index"] + 1)
        elif key == "home":
            state["paused"] = True
            render(0)
        elif key == "end":
            state["paused"] = True
            render(len(analysis.tof_frames) - 1)

    fig.canvas.mpl_connect("key_press_event", on_key)
    render(0)
    plt.show(block=False)

    deadline = time.monotonic()
    while not state["quit"] and plt.fignum_exists(fig.number):
        index = state["index"]

        if state["paused"]:
            plt.pause(0.02)
            continue

        if index >= len(analysis.tof_frames) - 1:
            state["paused"] = True
            plt.pause(0.02)
            continue

        if state["retime"]:
            delay = max(0.0, times[index + 1] - times[index])
            deadline = time.monotonic() + delay
            state["retime"] = False

        remaining = deadline - time.monotonic()
        if remaining > 0.0:
            plt.pause(min(0.02, remaining))
            continue

        render(index + 1)

    if plt.fignum_exists(fig.number):
        plt.close(fig)


def export_mp4(analysis, output_path, *, min_mm, max_mm, fps=None) -> float:
    plt, fig, image, status = build_temporal_figure(
        analysis,
        min_mm=min_mm,
        max_mm=max_mm,
        controls=False,
    )
    try:
        from matplotlib.animation import FFMpegWriter
    except ImportError as exc:
        plt.close(fig)
        raise RuntimeError("Matplotlib FFmpeg support is unavailable") from exc

    if not FFMpegWriter.isAvailable():
        plt.close(fig)
        raise RuntimeError(
            "FFmpeg is required for --export-mp4 and was not found on PATH"
        )

    export_fps = temporal.suggested_export_fps(analysis.tof_frames) if fps is None else float(fps)
    schedule = temporal.cfr_source_indices(analysis.tof_frames, export_fps)
    times = temporal.relative_ready_times_s(analysis.tof_frames)
    writer = FFMpegWriter(
        fps=export_fps,
        metadata={
            "title": "Rangeweave ToF depth",
            "comment": "producer-native 8x8 depth; nearest-neighbour rendering",
        },
    )

    with writer.saving(fig, str(output_path), dpi=160):
        for source_index in schedule:
            update_temporal_frame(analysis, image, status, source_index, times)
            writer.grab_frame()

    plt.close(fig)
    return export_fps


def main() -> int:
    parser = argparse.ArgumentParser(description="Rangeweave raw depth/health viewer")
    parser.add_argument("capture", help="capture directory or packets.bin")
    parser.add_argument("--frame", type=int, default=-1, help="ToF frame index; negative indices count from end (default: -1)")
    parser.add_argument("--summary-only", action="store_true", help="print diagnostics without importing Matplotlib")
    parser.add_argument("--save", default=None, help="save static viewer figure to this path")
    parser.add_argument("--no-show", action="store_true", help="do not open the static Matplotlib window")
    parser.add_argument("--play", action="store_true", help="replay ToF frames using recorded MCU-ready timing")
    parser.add_argument("--export-mp4", default=None, help="export timestamp-aware ToF playback to an MP4 file")
    parser.add_argument("--fps", type=float, default=None, help="constant output FPS for --export-mp4 (default: recorded cadence)")
    parser.add_argument("--min-mm", type=float, default=None, help="fixed temporal depth scale minimum; default is capture minimum")
    parser.add_argument("--max-mm", type=float, default=None, help="fixed temporal depth scale maximum; default is capture maximum")
    args = parser.parse_args()

    if args.summary_only and (args.save or args.play or args.export_mp4):
        parser.error("--summary-only cannot be combined with graphical/output modes")
    if args.play and (args.save or args.export_mp4 or args.no_show):
        parser.error("--play cannot be combined with --save, --export-mp4 or --no-show")
    if args.export_mp4 and (args.save or args.no_show):
        parser.error("--export-mp4 cannot be combined with --save or --no-show")
    if args.fps is not None and not args.export_mp4:
        parser.error("--fps is only valid with --export-mp4")
    if args.fps is not None and (not math.isfinite(args.fps) or args.fps <= 0.0):
        parser.error("--fps must be greater than zero")

    try:
        analysis = depth.analyse_capture(Path(args.capture))
        selected_index, selected = selected_frame(analysis, args.frame)
    except (OSError, depth.DepthAnalysisError) as exc:
        parser.error(str(exc))

    print_summary(analysis, selected_index=selected_index, selected=selected)

    if args.summary_only:
        return 0

    if args.play or args.export_mp4:
        try:
            min_mm, max_mm = temporal_depth_bounds(
                analysis,
                args.min_mm,
                args.max_mm,
            )
        except depth.DepthAnalysisError as exc:
            parser.error(str(exc))

        print()
        print(f"Temporal depth scale: {min_mm:.1f}..{max_mm:.1f} mm")

        if args.play:
            try:
                play_capture(analysis, min_mm=min_mm, max_mm=max_mm)
            except (RuntimeError, temporal.TemporalViewError) as exc:
                parser.error(str(exc))
            return 0

        output_path = Path(args.export_mp4)
        try:
            export_fps = export_mp4(
                analysis,
                output_path,
                min_mm=min_mm,
                max_mm=max_mm,
                fps=args.fps,
            )
        except (OSError, RuntimeError, temporal.TemporalViewError) as exc:
            parser.error(str(exc))
        print(f"Saved temporal MP4: {output_path} ({export_fps:g} fps)")
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
