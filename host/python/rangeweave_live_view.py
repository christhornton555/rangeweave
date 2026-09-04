"""Non-blocking live ToF viewer support for Rangeweave capture.py.

The serial recorder owns acquisition. Visualization runs in a separate spawned process
and receives only the latest decoded ToF frame through a size-1 queue, so stale display
updates may be dropped without ever dropping bytes from packets.bin.

The graphical presentation is rotated 180 degrees from producer-native storage to match
the physically observed sensor orientation. Raw capture ordering is unchanged.

Alongside the colour depth grid, the viewer fits the latest frame to a plane using the
current nominal ToF geometry and reports that plane's orientation relative to
``tof_optical``. This is an alignment aid, not a claim about device-body boresight.
"""

from __future__ import annotations

import importlib.util
import multiprocessing
from queue import Empty, Full
import sys

import rangeweave_geometry as geometry
import rangeweave_plane_alignment as plane_alignment


DISPLAY_ROTATION_DEGREES = 180


def replace_latest(queue, item) -> bool:
    """Offer an item without blocking, replacing one stale queued item if needed."""
    try:
        queue.put_nowait(item)
        return True
    except Full:
        pass

    try:
        queue.get_nowait()
    except Empty:
        pass

    try:
        queue.put_nowait(item)
        return True
    except Full:
        return False


def rotate_grid_180(grid):
    """Return a presentation-only 180-degree rotation of a 2D grid."""
    return [list(reversed(row)) for row in reversed(grid)]


def _render_grid(distances, rows, cols):
    values = iter(distances)
    producer_grid = [
        [
            float(value) if int(value) > 0 else float("nan")
            for value in (next(values) for _ in range(cols))
        ]
        for _ in range(rows)
    ]
    return rotate_grid_180(producer_grid)


def _configure_axes(ax, rows: int, cols: int) -> None:
    ax.set_xlabel("producer-native column")
    ax.set_ylabel("producer-native row")
    ax.set_xticks(range(cols))
    ax.set_yticks(range(rows))
    ax.set_xticklabels(list(reversed(range(cols))))
    ax.set_yticklabels(list(reversed(range(rows))))


def _rx_hint(angle_deg: float) -> str:
    if abs(angle_deg) < 0.5:
        return "top/bottom approximately aligned"
    if angle_deg > 0.0:
        return "top closer / bottom farther"
    return "bottom closer / top farther"


def _ry_hint(angle_deg: float) -> str:
    if abs(angle_deg) < 0.5:
        return "left/right approximately aligned"
    if angle_deg > 0.0:
        return "right closer / left farther"
    return "left closer / right farther"


def alignment_panel_text(fit: plane_alignment.PlaneAlignment) -> str:
    return (
        "Plane alignment\n"
        "(relative to tof_optical)\n\n"
        "Rx  {:+6.2f} deg\n"
        "    {}\n\n"
        "Ry  {:+6.2f} deg\n"
        "    {}\n\n"
        "fit RMS       {:6.2f} mm\n"
        "max residual  {:6.2f} mm\n"
        "valid zones   {:2d} / 64\n\n"
        "geometry: {}\n\n"
        "This panel describes the plane\n"
        "seen by the ToF optical frame.\n"
        "It does not assume the package\n"
        "is square to device_body."
    ).format(
        fit.rotation_x_deg,
        _rx_hint(fit.rotation_x_deg),
        fit.rotation_y_deg,
        _ry_hint(fit.rotation_y_deg),
        fit.rms_residual_mm,
        fit.max_abs_residual_mm,
        fit.valid_zones,
        geometry.NOMINAL_ST_PROFILE.role,
    )


def _viewer_process(queue, min_mm: float, max_mm: float) -> None:
    try:
        import matplotlib.pyplot as plt
    except Exception as exc:
        print("Rangeweave live viewer could not start: {}".format(exc), file=sys.stderr)
        return

    plt.ion()
    fig = plt.figure(figsize=(11, 7))
    layout = fig.add_gridspec(1, 2, width_ratios=(3.2, 2.0))
    ax = fig.add_subplot(layout[0, 0])
    info_ax = fig.add_subplot(layout[0, 1])
    info_ax.axis("off")
    info_text = info_ax.text(
        0.02,
        0.98,
        "Plane alignment\n\nwaiting for TOF_GRID...",
        va="top",
        ha="left",
        family="monospace",
        transform=info_ax.transAxes,
    )

    fig.suptitle(
        "Rangeweave live ToF depth + plane alignment (display rotated {}°)".format(
            DISPLAY_ROTATION_DEGREES
        )
    )
    status = fig.text(0.5, 0.02, "waiting for TOF_GRID...", ha="center")

    image = None
    colorbar = None
    current_shape = None
    running = True

    while running and plt.fignum_exists(fig.number):
        latest = None
        while True:
            try:
                message = queue.get_nowait()
            except Empty:
                break

            if message is None:
                running = False
                break
            latest = message

        if latest is not None:
            rows, cols, distances, ready_us = latest
            shape = (rows, cols)
            grid = _render_grid(distances, rows, cols)

            if image is None or shape != current_shape:
                ax.clear()
                _configure_axes(ax, rows, cols)
                image = ax.imshow(
                    grid,
                    origin="upper",
                    interpolation="nearest",
                    vmin=min_mm,
                    vmax=max_mm,
                )
                if colorbar is not None:
                    colorbar.remove()
                colorbar = fig.colorbar(image, ax=ax, shrink=0.80)
                colorbar.set_label("distance (mm)")
                current_shape = shape
            else:
                image.set_data(grid)

            try:
                fit = plane_alignment.fit_plane_from_distances(distances)
            except (plane_alignment.PlaneAlignmentError, geometry.GeometryError) as exc:
                info_text.set_text(
                    "Plane alignment\n"
                    "(relative to tof_optical)\n\n"
                    "unavailable:\n{}".format(exc)
                )
            else:
                info_text.set_text(alignment_panel_text(fit))

            status.set_text(
                "latest ToF ready timestamp: {:.3f} s | fixed range {:.0f}..{:.0f} mm".format(
                    ready_us / 1_000_000.0,
                    min_mm,
                    max_mm,
                )
            )
            fig.canvas.draw_idle()

        plt.pause(0.02)

    plt.close(fig)


class LiveDepthPublisher:
    """Publish decoded ToF frames to a non-blocking viewer process."""

    def __init__(self, *, min_mm: float = 100.0, max_mm: float = 1000.0) -> None:
        if min_mm >= max_mm:
            raise ValueError("live-view minimum depth must be less than maximum depth")
        self.min_mm = float(min_mm)
        self.max_mm = float(max_mm)
        self._queue = None
        self._process = None
        self._last_distances = None

    @property
    def alignment_geometry_role(self) -> str:
        return geometry.NOMINAL_ST_PROFILE.role

    def start(self) -> None:
        if importlib.util.find_spec("matplotlib") is None:
            raise RuntimeError(
                "Matplotlib is required for --live-view: py -m pip install matplotlib"
            )
        if self._process is not None:
            return

        context = multiprocessing.get_context("spawn")
        self._queue = context.Queue(maxsize=1)
        self._process = context.Process(
            target=_viewer_process,
            args=(self._queue, self.min_mm, self.max_mm),
            daemon=True,
        )
        self._process.start()

    def publish(self, record) -> None:
        if record.distance_mm is None:
            return

        distances = tuple(int(value) for value in record.distance_mm)
        self._last_distances = distances

        if self._queue is None:
            return

        message = (
            int(record.rows),
            int(record.cols),
            distances,
            int(record.mcu_ready_us),
        )
        replace_latest(self._queue, message)

    def final_alignment(self) -> plane_alignment.PlaneAlignment | None:
        """Fit the final valid ToF frame remembered by the parent recorder."""
        if self._last_distances is None:
            return None
        return plane_alignment.fit_plane_from_distances(self._last_distances)

    def close(self) -> None:
        if self._process is None:
            return

        if self._queue is not None:
            replace_latest(self._queue, None)

        self._process.join(timeout=1.0)
        if self._process.is_alive():
            self._process.terminate()
            self._process.join(timeout=1.0)

        if self._queue is not None:
            self._queue.close()

        self._queue = None
        self._process = None
