"""Raw depth/health viewer for Rangeweave v0.1 captures.

This is deliberately a host-side diagnostic tool. It uses the canonical
Rangeweave wire decoder and capture metadata helpers rather than defining
any additional protocol semantics.

Current validity rule:
    distance_mm == 0 is treated as an invalid/sentinel range measurement.

That rule belongs here in the viewer/analysis layer, not in the wire
protocol decoder.
"""

from __future__ import annotations

import argparse
import hashlib
from pathlib import Path
import statistics

import matplotlib.pyplot as plt
import numpy as np

import rangeweave_capture as rwc
import rangeweave_protocol as rw


PLANE_MIN_VALID_FRACTION = 0.50


def resolve_capture(path: Path) -> tuple[Path, Path | None]:
    """Return packets.bin path and optional capture-session directory."""
    if path.is_dir():
        packets = path / rwc.PACKETS_FILENAME
        if not packets.is_file():
            raise FileNotFoundError(
                f"capture directory does not contain {rwc.PACKETS_FILENAME}: {path}"
            )
        return packets, path

    if not path.is_file():
        raise FileNotFoundError(path)

    session_dir = path.parent
    metadata = session_dir / rwc.METADATA_FILENAME
    if not metadata.is_file():
        session_dir = None

    return path, session_dir


def load_capture(
    packets_path: Path,
) -> tuple[
    rw.StreamDecoder,
    rwc.StreamStats,
    int,
    str,
    list[rw.TofGrid],
]:
    """Decode once, collecting stream health plus all valid TOF_GRID records."""
    decoder = rw.StreamDecoder()
    stats = rwc.StreamStats()
    tof_frames: list[rw.TofGrid] = []

    digest = hashlib.sha256()
    byte_count = 0

    with packets_path.open("rb") as source:
        while True:
            chunk = source.read(4096)
            if not chunk:
                break

            byte_count += len(chunk)
            digest.update(chunk)

            for frame in decoder.feed(chunk):
                stats.consume(frame)

                if frame.record_type != rw.RECORD_TOF_GRID:
                    continue

                try:
                    record = rw.decode_record(frame)
                except rw.ProtocolError:
                    # StreamStats has already counted this semantic error.
                    continue

                tof_frames.append(record)

    return decoder, stats, byte_count, digest.hexdigest(), tof_frames


def decode_text_tlv(info: rw.StreamInfo | None, tag: int) -> str | None:
    if info is None:
        return None

    value = info.first_value(tag)
    if value is None:
        return None

    return value.decode("utf-8", "replace")


def print_float_grid(
    title: str,
    grid: np.ndarray,
    *,
    width: int = 8,
    precision: int = 1,
    suffix: str = "",
) -> None:
    print()
    print(title)

    for row_index, row in enumerate(grid):
        values = []
        for value in row:
            if np.isfinite(value):
                text = f"{value:{width}.{precision}f}{suffix}"
            else:
                text = f"{'---':>{width}}{suffix}"
            values.append(text)

        print(f"  r{row_index:02d}: " + " ".join(values))


def print_int_grid(title: str, grid: np.ndarray, *, width: int = 7) -> None:
    print()
    print(title)

    for row_index, row in enumerate(grid):
        values = [f"{int(value):{width}d}" for value in row]
        print(f"  r{row_index:02d}: " + " ".join(values))


def print_selected_distance(grid: np.ndarray) -> None:
    print()
    print("Selected frame distance (mm), producer-native rows/columns")

    for row_index, row in enumerate(grid):
        values = [f"{int(value):7d}" for value in row]
        print(f"  r{row_index:02d}: " + " ".join(values))


def validate_tof_geometry(tof_frames: list[rw.TofGrid]) -> tuple[int, int, int]:
    """Require one stable grid geometry/layout for this diagnostic viewer."""
    geometries = {
        (
            frame.rows,
            frame.cols,
            frame.targets_per_zone,
            frame.layout_id,
        )
        for frame in tof_frames
    }

    if len(geometries) != 1:
        descriptions = ", ".join(
            f"{rows}x{cols}/targets={targets}/layout={layout}"
            for rows, cols, targets, layout in sorted(geometries)
        )
        raise RuntimeError(
            "capture contains multiple ToF geometries/layouts; "
            f"viewer currently expects one: {descriptions}"
        )

    rows, cols, targets_per_zone, layout_id = next(iter(geometries))

    if targets_per_zone != 1:
        raise RuntimeError(
            f"viewer currently expects one target per zone, got {targets_per_zone}"
        )

    return rows, cols, layout_id


def build_distance_stack(
    tof_frames: list[rw.TofGrid],
    rows: int,
    cols: int,
) -> np.ndarray:
    values = []

    for frame in tof_frames:
        if frame.distance_mm is None:
            continue

        values.append(
            np.asarray(frame.distance_mm, dtype=np.float64).reshape(rows, cols)
        )

    if not values:
        raise RuntimeError("capture contains no ToF distance fields")

    return np.stack(values, axis=0)


def build_reflectance_stack(
    tof_frames: list[rw.TofGrid],
    rows: int,
    cols: int,
) -> np.ndarray | None:
    values = []

    for frame in tof_frames:
        if frame.reflectance_percent is None:
            return None

        values.append(
            np.asarray(
                frame.reflectance_percent,
                dtype=np.float64,
            ).reshape(rows, cols)
        )

    if not values:
        return None

    return np.stack(values, axis=0)


def masked_mean(
    values: np.ndarray,
    valid: np.ndarray,
) -> tuple[np.ndarray, np.ndarray]:
    counts = np.sum(valid, axis=0, dtype=np.int64)

    totals = np.sum(
        np.where(valid, values, 0.0),
        axis=0,
        dtype=np.float64,
    )

    means = np.full(counts.shape, np.nan, dtype=np.float64)

    np.divide(
        totals,
        counts,
        out=means,
        where=counts > 0,
    )

    return means, counts


def masked_population_stddev(
    values: np.ndarray,
    valid: np.ndarray,
    means: np.ndarray,
    counts: np.ndarray,
) -> np.ndarray:
    differences = np.where(
        valid,
        values - means[np.newaxis, :, :],
        0.0,
    )

    squared_sum = np.sum(
        differences * differences,
        axis=0,
        dtype=np.float64,
    )

    variance = np.full(counts.shape, np.nan, dtype=np.float64)

    np.divide(
        squared_sum,
        counts,
        out=variance,
        where=counts > 0,
    )

    return np.sqrt(variance)


def masked_auxiliary_mean(
    values: np.ndarray,
    valid_distance: np.ndarray,
) -> np.ndarray:
    """Mean an auxiliary field only where the matching distance is valid."""
    counts = np.sum(valid_distance, axis=0, dtype=np.int64)

    totals = np.sum(
        np.where(valid_distance, values, 0.0),
        axis=0,
        dtype=np.float64,
    )

    means = np.full(counts.shape, np.nan, dtype=np.float64)

    np.divide(
        totals,
        counts,
        out=means,
        where=counts > 0,
    )

    return means


def fit_mean_plane(
    mean_distance: np.ndarray,
    valid_fraction: np.ndarray,
) -> tuple[
    float,
    float,
    float,
    np.ndarray,
    np.ndarray,
    float,
    float,
    int,
] | None:
    """Least-squares plane z = a + b*row + c*column."""
    row_grid, col_grid = np.indices(mean_distance.shape)

    usable = (
        np.isfinite(mean_distance)
        & (valid_fraction >= PLANE_MIN_VALID_FRACTION)
    )

    if np.count_nonzero(usable) < 3:
        return None

    design = np.column_stack(
        (
            np.ones(np.count_nonzero(usable), dtype=np.float64),
            row_grid[usable].astype(np.float64),
            col_grid[usable].astype(np.float64),
        )
    )

    observed = mean_distance[usable]

    coefficients, _, _, _ = np.linalg.lstsq(
        design,
        observed,
        rcond=None,
    )

    intercept, row_slope, col_slope = coefficients

    predicted = (
        intercept
        + row_slope * row_grid
        + col_slope * col_grid
    )

    residual = np.full(mean_distance.shape, np.nan, dtype=np.float64)
    residual[usable] = mean_distance[usable] - predicted[usable]

    rms = float(
        np.sqrt(
            np.mean(
                residual[usable] * residual[usable]
            )
        )
    )

    max_abs = float(np.max(np.abs(residual[usable])))

    return (
        float(intercept),
        float(row_slope),
        float(col_slope),
        predicted,
        residual,
        rms,
        max_abs,
        int(np.count_nonzero(usable)),
    )


def add_grid_plot(
    fig,
    position: int,
    grid: np.ndarray,
    title: str,
    value_format: str,
) -> None:
    ax = fig.add_subplot(2, 3, position)

    image = ax.imshow(grid)
    ax.set_title(title)
    ax.set_xlabel("producer-native column")
    ax.set_ylabel("producer-native row")

    ax.set_xticks(range(grid.shape[1]))
    ax.set_yticks(range(grid.shape[0]))

    finite_values = grid[np.isfinite(grid)]
    midpoint = (
        float(np.median(finite_values))
        if finite_values.size
        else 0.0
    )

    for row in range(grid.shape[0]):
        for col in range(grid.shape[1]):
            value = grid[row, col]

            if not np.isfinite(value):
                label = "—"
            else:
                label = format(value, value_format)

            # Automatic contrast against the image, without assigning a
            # semantic meaning to a particular colour.
            text_colour = "white" if np.isfinite(value) and value < midpoint else "black"

            ax.text(
                col,
                row,
                label,
                ha="center",
                va="center",
                fontsize=7,
                color=text_colour,
            )

    fig.colorbar(image, ax=ax, shrink=0.80)


def build_figure(
    capture_name: str,
    selected_distance: np.ndarray,
    mean_distance: np.ndarray,
    stddev_distance: np.ndarray,
    invalid_percent: np.ndarray,
    residual: np.ndarray | None,
    reflectance_mean: np.ndarray | None,
):
    fig = plt.figure(figsize=(16, 9))
    fig.suptitle(f"Rangeweave raw depth/health viewer — {capture_name}")

    selected_display = selected_distance.astype(np.float64)
    selected_display[selected_display == 0] = np.nan

    add_grid_plot(
        fig,
        1,
        selected_display,
        "Selected distance (mm)",
        ".0f",
    )

    add_grid_plot(
        fig,
        2,
        mean_distance,
        "Valid-only mean distance (mm)",
        ".1f",
    )

    add_grid_plot(
        fig,
        3,
        stddev_distance,
        "Valid-only population σ (mm)",
        ".1f",
    )

    add_grid_plot(
        fig,
        4,
        invalid_percent,
        "Invalid distance samples (%)",
        ".1f",
    )

    if residual is not None:
        add_grid_plot(
            fig,
            5,
            residual,
            "Mean-plane residual (mm)",
            "+.1f",
        )
    else:
        ax = fig.add_subplot(2, 3, 5)
        ax.axis("off")
        ax.text(
            0.5,
            0.5,
            "Insufficient valid zones\nfor plane fit",
            ha="center",
            va="center",
        )

    if reflectance_mean is not None:
        add_grid_plot(
            fig,
            6,
            reflectance_mean,
            "Reflectance mean on valid ranges (%)",
            ".1f",
        )
    else:
        ax = fig.add_subplot(2, 3, 6)
        ax.axis("off")
        ax.text(
            0.5,
            0.5,
            "No reflectance field",
            ha="center",
            va="center",
        )

    fig.tight_layout(rect=(0, 0, 1, 0.96))
    return fig


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Rangeweave raw depth/health viewer"
    )

    parser.add_argument(
        "capture",
        help="capture directory or packets.bin",
    )

    parser.add_argument(
        "--frame",
        type=int,
        default=None,
        help="zero-based ToF frame index; default is the final frame",
    )

    parser.add_argument(
        "--save",
        default=None,
        help="save viewer figure to this path",
    )

    parser.add_argument(
        "--no-show",
        action="store_true",
        help="do not open an interactive matplotlib window",
    )

    args = parser.parse_args()

    capture_path = Path(args.capture)
    packets_path, session_dir = resolve_capture(capture_path)

    (
        decoder,
        stats,
        byte_count,
        sha256,
        tof_frames,
    ) = load_capture(packets_path)

    if not tof_frames:
        raise RuntimeError("capture contains no valid TOF_GRID records")

    rows, cols, layout_id = validate_tof_geometry(tof_frames)

    selected_index = (
        len(tof_frames) - 1
        if args.frame is None
        else args.frame
    )

    if not 0 <= selected_index < len(tof_frames):
        raise IndexError(
            f"--frame must be between 0 and {len(tof_frames) - 1}"
        )

    selected = tof_frames[selected_index]

    distance_stack = build_distance_stack(
        tof_frames,
        rows,
        cols,
    )

    reflectance_stack = build_reflectance_stack(
        tof_frames,
        rows,
        cols,
    )

    # Protocol v0.1 currently carries no explicit viewer-level validity
    # semantic. For these captures, zero distance is the invalid/sentinel.
    valid_distance = distance_stack > 0

    mean_distance, valid_count = masked_mean(
        distance_stack,
        valid_distance,
    )

    stddev_distance = masked_population_stddev(
        distance_stack,
        valid_distance,
        mean_distance,
        valid_count,
    )

    frame_count = distance_stack.shape[0]

    invalid_count = frame_count - valid_count
    valid_fraction = valid_count.astype(np.float64) / frame_count
    invalid_percent = 100.0 * invalid_count / frame_count

    reflectance_mean = None
    if reflectance_stack is not None:
        reflectance_mean = masked_auxiliary_mean(
            reflectance_stack,
            valid_distance,
        )

    plane = fit_mean_plane(
        mean_distance,
        valid_fraction,
    )

    if plane is None:
        plane_residual = None
    else:
        (
            plane_intercept,
            plane_row_slope,
            plane_col_slope,
            _plane_prediction,
            plane_residual,
            plane_rms,
            plane_max_abs,
            plane_zone_count,
        ) = plane

    field_masks = sorted({frame.field_mask for frame in tof_frames})

    ready_times = [frame.mcu_ready_us for frame in tof_frames]
    read_durations = [
        frame.mcu_read_complete_us - frame.mcu_ready_us
        for frame in tof_frames
    ]

    observed_rate = None
    if len(ready_times) >= 2:
        span_us = ready_times[-1] - ready_times[0]
        if span_us > 0:
            observed_rate = (
                (len(ready_times) - 1)
                * 1_000_000.0
                / span_us
            )

    firmware = decode_text_tlv(
        stats.last_info,
        rw.INFO_FIRMWARE_LABEL,
    )

    source_profile = decode_text_tlv(
        stats.last_info,
        rw.INFO_SOURCE_PROFILE,
    )

    metadata_status = "N/A"
    metadata_errors: list[str] = []

    if session_dir is not None:
        try:
            metadata = rwc.load_metadata(session_dir)
            metadata_errors = rwc.metadata_parity_errors(
                metadata,
                decoder=decoder,
                stats=stats,
                packets_bytes=byte_count,
                packets_sha256=sha256,
            )
            metadata_status = "PASS" if not metadata_errors else "FAIL"
        except (OSError, ValueError) as exc:
            metadata_status = f"ERROR ({exc})"

    print("Rangeweave raw depth/health summary")
    print(f"  packets:       {packets_path}")
    print(f"  bytes:         {byte_count}")
    print(f"  SHA-256:       {sha256}")
    print(f"  valid frames:  {decoder.frames_ok}")
    print(f"  bad frames:    {decoder.frames_bad}")
    print(f"  semantic errs: {stats.semantic_errors}")
    print(f"  seq gaps:      {stats.sequence_gaps}")
    print(f"  ToF frames:    {len(tof_frames)}")
    print(f"  grid:          {rows}x{cols}")

    if layout_id == 0:
        print(
            "  layout_id:     "
            "0 (producer-native flattened zone order)"
        )
    else:
        print(f"  layout_id:     {layout_id}")

    print(
        "  field masks:   "
        + ", ".join(f"0x{mask:04X}" for mask in field_masks)
    )

    if observed_rate is not None:
        print(
            f"  observed rate: {observed_rate:.4f} Hz "
            "from MCU ready-observation timestamps"
        )

    if read_durations:
        print(
            f"  mean read:     "
            f"{statistics.mean(read_durations):.1f} us"
        )

    print(
        f"  selected ToF:  frame {selected_index} / "
        f"{len(tof_frames) - 1}"
    )

    if firmware:
        print(f"  firmware:      {firmware}")

    if source_profile:
        print(f"  source:        {source_profile}")

    print(f"  metadata:      {metadata_status}")
    print("  validity rule: distance_mm > 0")

    if metadata_errors:
        for error in metadata_errors:
            print(f"    metadata mismatch: {error}")

    health_deltas = stats.health_deltas()

    if health_deltas:
        print("  STATUS deltas:")
        for key in (
            "frames_dropped",
            "imu_samples_dropped",
            "fifo_overruns",
            "fifo_structural_errors",
            "mag_errors",
            "tof_errors",
            "clock_sync_errors",
        ):
            print(
                f"    {key + ':':24s} "
                f"{health_deltas.get(key, 0)}"
            )

    print_float_grid(
        "Distance valid-only mean (mm), producer-native rows/columns",
        mean_distance,
    )

    print_float_grid(
        "Distance valid-only population stddev (mm), "
        "producer-native rows/columns",
        stddev_distance,
    )

    print_int_grid(
        "Valid distance sample count, producer-native rows/columns",
        valid_count,
    )

    print_float_grid(
        "Invalid distance samples (%), producer-native rows/columns",
        invalid_percent,
        width=7,
        precision=1,
    )

    if reflectance_mean is not None:
        print_float_grid(
            "Reflectance mean (%) on valid distance samples, "
            "producer-native rows/columns",
            reflectance_mean,
        )

    if plane is None:
        print()
        print("Mean-distance plane fit")
        print("  unavailable: fewer than 3 sufficiently valid zones")
    else:
        print()
        print("Mean-distance plane fit")
        print(
            "  model:         "
            "distance_mm = "
            f"{plane_intercept:.3f} "
            f"{plane_row_slope:+.3f}*row "
            f"{plane_col_slope:+.3f}*column"
        )
        print(
            f"  zones used:    {plane_zone_count} / "
            f"{rows * cols} "
            f"(>= {PLANE_MIN_VALID_FRACTION * 100:.0f}% valid)"
        )
        print(f"  residual RMS:  {plane_rms:.3f} mm")
        print(f"  max |residual|:{plane_max_abs:8.3f} mm")

        print_float_grid(
            "Mean-distance plane residual (mm), "
            "producer-native rows/columns",
            plane_residual,
            precision=1,
        )

    selected_distance = np.asarray(
        selected.distance_mm,
        dtype=np.int64,
    ).reshape(rows, cols)

    print_selected_distance(selected_distance)

    fig = build_figure(
        capture_path.name,
        selected_distance,
        mean_distance,
        stddev_distance,
        invalid_percent,
        plane_residual,
        reflectance_mean,
    )

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
    