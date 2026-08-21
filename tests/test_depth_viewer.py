"""Tests for replay-first raw Rangeweave depth analysis."""

from pathlib import Path
import math
import struct
import sys
import tempfile
import unittest

HERE = Path(__file__).resolve().parent
REPO_ROOT = HERE.parent
HOST_PYTHON = REPO_ROOT / "host" / "python"
if str(HOST_PYTHON) not in sys.path:
    sys.path.insert(0, str(HOST_PYTHON))

import rangeweave_depth as depth
import rangeweave_protocol as rw


def tof_wire(sequence, ready_us, distances, reflectance, *, rows=2, cols=2, layout_id=0):
    field_mask = rw.TOF_FIELD_DISTANCE_MM | rw.TOF_FIELD_REFLECTANCE_PERCENT
    payload = bytearray(
        struct.pack(
            "<QQBBBBH",
            ready_us,
            ready_us + 100,
            rows,
            cols,
            1,
            layout_id,
            field_mask,
        )
    )
    payload.extend(struct.pack("<{}H".format(len(distances)), *distances))
    payload.extend(bytes(reflectance))
    return rw.encode_frame(rw.RECORD_TOF_GRID, sequence, bytes(payload))


class DepthAnalysisTests(unittest.TestCase):
    def test_two_frame_zone_statistics(self):
        wire = b"".join(
            [
                tof_wire(10, 1_000, [100, 200, 300, 400], [10, 20, 30, 40]),
                tof_wire(11, 2_000, [110, 190, 310, 390], [12, 18, 32, 38]),
            ]
        )
        with tempfile.TemporaryDirectory() as temp:
            path = Path(temp) / "packets.bin"
            path.write_bytes(wire)
            result = depth.analyse_capture(path, chunk_size=7)

        self.assertEqual((result.rows, result.cols, result.layout_id), (2, 2, 0))
        self.assertEqual(len(result.tof_frames), 2)
        self.assertEqual(result.decoder.frames_ok, 2)
        self.assertEqual(result.decoder.frames_bad, 0)
        self.assertEqual(result.stream_stats.sequence_gaps, 0)
        self.assertEqual(result.distance.count, (2, 2, 2, 2))
        self.assertEqual(result.distance.invalid_count, (0, 0, 0, 0))
        self.assertEqual(result.distance.mean, (105.0, 195.0, 305.0, 395.0))
        self.assertEqual(result.distance.stddev, (5.0, 5.0, 5.0, 5.0))
        self.assertEqual(result.distance.minimum, (100.0, 190.0, 300.0, 390.0))
        self.assertEqual(result.distance.maximum, (110.0, 200.0, 310.0, 400.0))
        self.assertIsNotNone(result.reflectance)
        self.assertEqual(result.reflectance.mean, (11.0, 19.0, 31.0, 39.0))
        self.assertEqual(result.reflectance.invalid_count, (0, 0, 0, 0))
        self.assertEqual(result.ready_span_us, 1_000)
        self.assertAlmostEqual(result.observed_ready_rate_hz, 1_000.0)
        self.assertEqual(result.read_duration_us.mean, (100.0,))
        self.assertEqual(
            depth.as_rows(result.distance.mean, 2, 2),
            [[105.0, 195.0], [305.0, 395.0]],
        )

    def test_zero_distance_is_invalid_and_reflectance_follows_validity(self):
        wire = b"".join(
            [
                tof_wire(10, 1_000, [100, 0, 300, 400], [10, 20, 30, 40]),
                tof_wire(11, 2_000, [110, 200, 0, 390], [12, 18, 32, 38]),
            ]
        )
        with tempfile.TemporaryDirectory() as temp:
            path = Path(temp) / "packets.bin"
            path.write_bytes(wire)
            result = depth.analyse_capture(path, chunk_size=7)

        self.assertEqual(result.distance.count, (2, 1, 1, 2))
        self.assertEqual(result.distance.invalid_count, (0, 1, 1, 0))
        self.assertEqual(result.distance.invalid_percent, (0.0, 50.0, 50.0, 0.0))
        self.assertEqual(result.distance.mean, (105.0, 200.0, 300.0, 395.0))
        self.assertEqual(result.distance.stddev, (5.0, 0.0, 0.0, 5.0))
        self.assertEqual(result.distance.minimum, (100.0, 200.0, 300.0, 390.0))
        self.assertEqual(result.distance.maximum, (110.0, 200.0, 300.0, 400.0))
        self.assertIsNotNone(result.reflectance)
        self.assertEqual(result.reflectance.count, (2, 1, 1, 2))
        self.assertEqual(result.reflectance.invalid_count, (0, 1, 1, 0))
        self.assertEqual(result.reflectance.mean, (11.0, 18.0, 30.0, 39.0))

    def test_mean_distance_plane_fit(self):
        wire = b"".join(
            [
                tof_wire(1, 1_000, [100, 110, 120, 130], [1, 1, 1, 1]),
                tof_wire(2, 2_000, [100, 110, 120, 130], [1, 1, 1, 1]),
            ]
        )
        with tempfile.TemporaryDirectory() as temp:
            path = Path(temp) / "packets.bin"
            path.write_bytes(wire)
            result = depth.analyse_capture(path)

        plane = result.mean_distance_plane
        self.assertIsNotNone(plane)
        self.assertAlmostEqual(plane.intercept_mm, 100.0)
        self.assertAlmostEqual(plane.row_slope_mm, 20.0)
        self.assertAlmostEqual(plane.column_slope_mm, 10.0)
        self.assertAlmostEqual(plane.rms_residual_mm, 0.0)
        self.assertAlmostEqual(plane.max_abs_residual_mm, 0.0)
        self.assertEqual(plane.zones_used, 4)
        for residual in plane.residual_mm:
            self.assertTrue(math.isclose(residual, 0.0, abs_tol=1e-12))

    def test_geometry_change_is_rejected(self):
        wire = b"".join(
            [
                tof_wire(1, 1_000, [1, 2, 3, 4], [1, 2, 3, 4], layout_id=0),
                tof_wire(2, 2_000, [1, 2, 3, 4], [1, 2, 3, 4], layout_id=1),
            ]
        )
        with tempfile.TemporaryDirectory() as temp:
            path = Path(temp) / "packets.bin"
            path.write_bytes(wire)
            with self.assertRaises(depth.DepthAnalysisError):
                depth.analyse_capture(path)

    def test_capture_without_tof_is_rejected(self):
        payload = struct.pack("<QQQ", 100, 200, 300)
        wire = rw.encode_frame(rw.RECORD_CLOCK_SYNC, 1, payload)
        with tempfile.TemporaryDirectory() as temp:
            path = Path(temp) / "packets.bin"
            path.write_bytes(wire)
            with self.assertRaises(depth.DepthAnalysisError):
                depth.analyse_capture(path)


if __name__ == "__main__":
    unittest.main()
