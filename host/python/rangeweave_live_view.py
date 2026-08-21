"""Non-blocking live ToF viewer support for Rangeweave capture.py.

The serial recorder owns acquisition. Visualization runs in a separate spawned process
and receives only the latest decoded ToF frame through a size-1 queue, so stale display
updates may be dropped without ever dropping bytes from packets.bin.
"""

from __future__ import annotations

import importlib.util
import multiprocessing
from queue import Empty, Full
import sys


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


def _render_grid(distances, rows, cols):
    values = iter(distances)
    return [
        [
            float(value) if int(value) > 0 else float("nan")
            for value in (next(values) for _ in range(cols))
        ]
        for _ in range(rows)
    ]


def _viewer_process(queue, min_mm: float, max_mm: float) -> None:
    try:
        import matplotlib.pyplot as plt
    except Exception as exc:
        print("Rangeweave live viewer could not start: {}".format(exc), file=sys.stderr)
        return

    plt.ion()
    fig, ax = plt.subplots(figsize=(7, 7))
    fig.suptitle("Rangeweave live ToF depth")
    ax.set_xlabel("producer-native column")
    ax.set_ylabel("producer-native row")
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
                ax.set_xlabel("producer-native column")
                ax.set_ylabel("producer-native row")
                ax.set_xticks(range(cols))
                ax.set_yticks(range(rows))
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
        if self._queue is None or record.distance_mm is None:
            return
        message = (
            int(record.rows),
            int(record.cols),
            tuple(int(value) for value in record.distance_mm),
            int(record.mcu_ready_us),
        )
        replace_latest(self._queue, message)

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
