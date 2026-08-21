"""Raw 8x8 Rangeweave depth/reflectance and stream-health viewer."""

from __future__ import annotations

import argparse
import math
import sys

import rangeweave_capture as cap
import rangeweave_depth as depth
import rangeweave_protocol as rw


def _format_grid(values, rows, cols, *, decimals=1, width=8):
    lines = []
    matrix = depth.as_rows(values, rows, cols)
    for row_index, row in enumerate(matrix):
        cells = []
        for value in row:
            if isinstance(value, float) and math.isnan(value):
                text = "nan"
            elif decimals == 0:
                text = "{:.0f}".format(value)
            else:
                text = ("{:." + str(decimals) + "f}").format(value)
            cells.append(text.rjust(width))
        lines.append("  r{:02d}:{}".format(row_index, "".join(cells)))
    return "\n".join(lines)


def _info_text(info, tag):
    if info is None:
        return None
    value = info.first_value(tag)
    if value is None:
        return None
    return value.decode("utf-8", "replace")


def _print_summary(result, frame_index):
    actual_index = frame_index if frame_index >= 0 else len(result.tof_frames) + frame_index
    frame = result.frame(frame_index)
    print("Rangeweave raw depth/health summary")
    print("  packets:       {}".format(result.packets_path))
    print("  bytes:         {}".format(result.packets_bytes))
    print("  SHA-256:       {}".format(result.packets_sha256))
    print("  valid frames:  {}".format(result.decoder.frames_ok))
    print("  bad frames:    {}".format(result.decoder.frames_bad))
    print("  semantic errs: {}".format(result.stream_stats.semantic_errors))
    print("  seq gaps:      {}".format(result.stream_stats.sequence_gaps))
    print("  ToF frames:    {}".format(len(result.tof_frames)))
    print("  grid:          {}x{}".format(result.rows, result.cols))
    print("  layout_id:     {} (producer-native flattened zone order)".format(result.layout_id))
    print(
        "  field masks:   {}".format(
            ", ".join("0x{:04X}".format(mask) for mask in result.field_masks)
        )
    )
    if result.observed_ready_rate_hz is not None:
        print("  observed rate: {:.4f} Hz from MCU ready-observation timestamps".format(
            result.observed_ready_rate_hz
        ))
    print("  mean read:     {:.1f} us".format(result.read_duration_us.mean[0]))
    print("  selected ToF:  frame {} / {}".format(actual_index, len(result.tof_frames) - 1))

    firmware = _info_text(result.stream_stats.last_info, rw.INFO_FIRMWARE_LABEL)
    source = _info_text(result.stream_stats.last_info, rw.INFO_SOURCE_PROFILE)
    if firmware:
        print("  firmware:      {}".format(firmware))
    if source:
        print("  source:        {}".format(source))

    if result.metadata_errors:
        print("  metadata:      FAIL")
        for error in result.metadata_errors:
            print("    - {}".format(error))
    elif result.input_path.is_dir():
        print("  metadata:      PASS")

    health = result.stream_stats.health_deltas()
    if health:
        print("  STATUS deltas:")
        for key, value in health.items():
            print("    {:24s} {}".format(key + ":", value))

    print()
    print("Distance mean (mm), producer-native rows/columns")
    print(_format_grid(result.distance.mean, result.rows, result.cols, decimals=1))
    print()
    print("Distance population stddev (mm), producer-native rows/columns")
    print(_format_grid(result.distance.stddev, result.rows, result.cols, decimals=1))

    if result.reflectance is not None:
        print()
        print("Reflectance mean (%), producer-native rows/columns")
        print(_format_grid(result.reflectance.mean, result.rows, result.cols, decimals=1))

    if frame.distance_mm is not None:
        print()
        print("Selected frame distance (mm), producer-native rows/columns")
        print(_format_grid(frame.distance_mm, result.rows, result.cols, decimals=0))


def _plot_heatmap(ax, values, rows, cols, title, unit, *, decimals=0):
    matrix = depth.as_rows(values, rows, cols)
    image = ax.imshow(matrix, aspect="equal", origin="upper")
    ax.set_title(title)
    ax.set_xlabel("producer-native column index")
    ax.set_ylabel("producer-native row index")
    ax.set_xticks(range(cols))
    ax.set_yticks(range(rows))

    for row in range(rows):
        for col in range(cols):
            value = matrix[row][col]
            if isinstance(value, float) and math.isnan(value):
                label = "nan"
            elif decimals == 0:
                label = "{:.0f}".format(value)
            else:
                label = ("{:." + str(decimals) + "f}").format(value)
            ax.text(col, row, label, ha="center", va="center", fontsize=7)

    colorbar = ax.figure.colorbar(image, ax=ax, fraction=0.046, pad=0.04)
    colorbar.set_label(unit)


def _make_plot(result, frame_index):
    try:
        import matplotlib.pyplot as plt
    except ImportError as exc:
        raise SystemExit(
            "matplotlib is required for the graphical viewer: py -m pip install matplotlib"
        ) from exc

    frame = result.frame(frame_index)
    actual_index = frame_index if frame_index >= 0 else len(result.tof_frames) + frame_index

    figure, axes = plt.subplots(2, 2, figsize=(13, 10))
    if frame.distance_mm is not None:
        _plot_heatmap(
            axes[0][0],
            frame.distance_mm,
            result.rows,
            result.cols,
            "Selected raw distance frame #{}".format(actual_index),
            "mm",
            decimals=0,
        )
    else:
        axes[0][0].text(0.5, 0.5, "distance array not present", ha="center", va="center")
        axes[0][0].set_axis_off()

    _plot_heatmap(
        axes[0][1],
        result.distance.mean,
        result.rows,
        result.cols,
        "Mean distance over {} ToF frames".format(len(result.tof_frames)),
        "mm",
        decimals=0,
    )
    _plot_heatmap(
        axes[1][0],
        result.distance.stddev,
        result.rows,
        result.cols,
        "Distance population stddev",
        "mm",
        decimals=1,
    )

    if result.reflectance is not None:
        _plot_heatmap(
            axes[1][1],
            result.reflectance.mean,
            result.rows,
            result.cols,
            "Mean reflectance",
            "%",
            decimals=0,
        )
    else:
        axes[1][1].text(0.5, 0.5, "reflectance array not present", ha="center", va="center")
        axes[1][1].set_axis_off()

    health = result.stream_stats.health_deltas()
    health_text = ", ".join("{}={}".format(key, value) for key, value in health.items())
    if not health_text:
        health_text = "no STATUS pair available"

    figure.suptitle(
        "Rangeweave raw depth viewer — layout_id {} (producer-native order)".format(
            result.layout_id
        )
    )
    figure.text(
        0.5,
        0.01,
        "bad={}  semantic={}  gaps={}  |  {}".format(
            result.decoder.frames_bad,
            result.stream_stats.semantic_errors,
            result.stream_stats.sequence_gaps,
            health_text,
        ),
        ha="center",
        fontsize=9,
    )
    figure.tight_layout(rect=(0, 0.035, 1, 0.95))
    return plt, figure


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Inspect raw Rangeweave TOF_GRID depth/reflectance and stream health"
    )
    parser.add_argument("capture", help="capture session directory or packets.bin")
    parser.add_argument(
        "--frame",
        type=int,
        default=-1,
        help="ToF frame index to display; negative indices count from the end (default: -1)",
    )
    parser.add_argument(
        "--summary-only",
        action="store_true",
        help="print numeric grids/health without opening Matplotlib",
    )
    parser.add_argument("--save", help="optional PNG/PDF/SVG output path for the figure")
    parser.add_argument(
        "--no-show",
        action="store_true",
        help="do not open an interactive window (useful with --save)",
    )
    args = parser.parse_args()

    try:
        result = depth.analyse_capture(args.capture)
        result.frame(args.frame)
    except depth.DepthAnalysisError as exc:
        print("depth analysis failed: {}".format(exc), file=sys.stderr)
        return 2

    _print_summary(result, args.frame)

    if not args.summary_only:
        plt, figure = _make_plot(result, args.frame)
        if args.save:
            figure.savefig(args.save, dpi=160, bbox_inches="tight")
            print("\nSaved viewer figure: {}".format(args.save))
        if not args.no_show:
            plt.show()
        else:
            plt.close(figure)

    issues = cap.stream_issue_count(result.decoder, result.stream_stats)
    return 1 if issues or result.metadata_errors else 0


if __name__ == "__main__":
    sys.exit(main())
