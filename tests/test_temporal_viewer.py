"""Tests for Rangeweave temporal playback and live-view plumbing."""

from hashlib import sha256
from io import BytesIO
from pathlib import Path
from queue import Queue
import struct
import sys
from types import SimpleNamespace
import unittest

HERE = Path(__file__).resolve().parent
REPO_ROOT = HERE.parent
HOST_PYTHON = REPO_ROOT / "host" / "python"
if str(HOST_PYTHON) not in sys.path:
    sys.path.insert(0, str(HOST_PYTHON))

import capture
import rangeweave_capture as cap
import rangeweave_live_view as live
import rangeweave_protocol as rw
import rangeweave_temporal as temporal


def frame_at(ready_us):
    return SimpleNamespace(mcu_ready_us=ready_us)


def tof_wire(sequence, ready_us, distances, *, rows=2, cols=2):
    field_mask = rw.TOF_FIELD_DISTANCE_MM
    payload = bytearray(
        struct.pack(
            "<QQBBBBH",
            ready_us,
            ready_us + 100,
            rows,
            cols,
            1,
            0,
            field_mask,
        )
    )
    payload.extend(struct.pack("<{}H".format(len(distances)), *distances))
    return rw.encode_frame(rw.RECORD_TOF_GRID, sequence, bytes(payload))


class TemporalTimingTests(unittest.TestCase):
    def test_relative_ready_times_and_rate(self):
        frames = [frame_at(1_000_000), frame_at(1_066_667), frame_at(1_133_334)]
        self.assertEqual(
            temporal.relative_ready_times_s(frames),
            (0.0, 0.066667, 0.133334),
        )
        self.assertAlmostEqual(temporal.median_period_s(frames), 0.066667)
        self.assertEqual(temporal.suggested_export_fps(frames), 15.0)

    def test_backwards_timestamp_is_rejected(self):
        frames = [frame_at(2_000), frame_at(1_999)]
        with self.assertRaises(temporal.TemporalViewError):
            temporal.relative_ready_times_s(frames)

    def test_cfr_schedule_holds_last_frame_across_real_gap(self):
        frames = [
            frame_at(0),
            frame_at(100_000),
            frame_at(400_000),
        ]
        indices = temporal.cfr_source_indices(frames, 10.0)
        self.assertGreaterEqual(len(indices), 5)
        self.assertEqual(indices[:5], (0, 1, 1, 1, 2))

    def test_bad_export_fps_is_rejected(self):
        with self.assertRaises(temporal.TemporalViewError):
            temporal.cfr_source_indices([frame_at(0)], 0.0)


class LatestFrameQueueTests(unittest.TestCase):
    def test_replace_latest_drops_stale_display_item(self):
        queue = Queue(maxsize=1)
        self.assertTrue(live.replace_latest(queue, "old"))
        self.assertTrue(live.replace_latest(queue, "new"))
        self.assertEqual(queue.get_nowait(), "new")


class CaptureLiveTapTests(unittest.TestCase):
    def test_live_tap_preserves_exact_wire_bytes_and_normal_stats(self):
        wire = tof_wire(7, 123_456, [100, 200, 300, 400])
        raw_file = BytesIO()
        digest = sha256()
        decoder = rw.StreamDecoder()
        stats = cap.StreamStats()

        class Publisher:
            def __init__(self):
                self.records = []

            def publish(self, record):
                self.records.append(record)

        publisher = Publisher()

        written = capture._write_and_decode(
            raw_file,
            digest,
            decoder,
            stats,
            wire,
            publisher,
        )

        self.assertEqual(written, len(wire))
        self.assertEqual(raw_file.getvalue(), wire)
        self.assertEqual(digest.hexdigest(), sha256(wire).hexdigest())
        self.assertEqual(decoder.frames_ok, 1)
        self.assertEqual(decoder.frames_bad, 0)
        self.assertEqual(stats.record_counts["TOF_GRID"], 1)
        self.assertEqual(len(publisher.records), 1)
        self.assertEqual(publisher.records[0].distance_mm, (100, 200, 300, 400))


if __name__ == "__main__":
    unittest.main()
