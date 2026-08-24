"""Tests for reducing stationary ToF captures to calibration observations."""

from pathlib import Path
import sys
from types import SimpleNamespace
import unittest

HERE = Path(__file__).resolve().parent
REPO_ROOT = HERE.parent
HOST_PYTHON = REPO_ROOT / "host" / "python"
if str(HOST_PYTHON) not in sys.path:
    sys.path.insert(0, str(HOST_PYTHON))

import rangeweave_tof_calibration_capture as capture_cal
import rangeweave_tof_calibration_workflow as workflow


class FakeStreamStats:
    def __init__(self, *, semantic_errors=0, sequence_gaps=0, health_deltas=None):
        self.semantic_errors = semantic_errors
        self.sequence_gaps = sequence_gaps
        self._health_deltas = (
            {
                "frames_dropped": 0,
                "imu_samples_dropped": 0,
                "fifo_overruns": 0,
                "fifo_structural_errors": 0,
                "mag_errors": 0,
                "tof_errors": 0,
                "clock_sync_errors": 0,
            }
            if health_deltas is None
            else dict(health_deltas)
        )

    def health_deltas(self):
        return dict(self._health_deltas)


def make_analysis(frames, **overrides):
    values = {
        "input_path": Path("captures/synthetic-calibration"),
        "packets_sha256": "a" * 64,
        "rows": 8,
        "cols": 8,
        "layout_id": 0,
        "tof_frames": tuple(frames),
        "decoder": SimpleNamespace(frames_bad=0),
        "stream_stats": FakeStreamStats(),
        "metadata_errors": (),
        "observed_ready_rate_hz": 15.0,
    }
    values.update(overrides)
    return SimpleNamespace(**values)


def uniform_frames(count, distance=800.0):
    return [
        SimpleNamespace(distance_mm=tuple([distance] * 64))
        for _ in range(count)
    ]


class CalibrationCaptureReductionTests(unittest.TestCase):
    def test_median_rejects_single_large_outlier(self):
        frames = uniform_frames(40, 800.0)
        outlier = list(frames[10].distance_mm)
        outlier[7] = 5000.0
        frames[10] = SimpleNamespace(distance_mm=tuple(outlier))

        observation = capture_cal.reduce_capture_analysis(make_analysis(frames))

        self.assertTrue(observation.structurally_valid)
        self.assertEqual(observation.usable_zone_count, 64)
        self.assertEqual(observation.distances_mm[7], 800.0)
        self.assertEqual(observation.mad_mm[7], 0.0)
        self.assertEqual(observation.valid_fraction[7], 1.0)

    def test_low_coverage_zone_becomes_none_without_filling(self):
        frames = uniform_frames(40, 800.0)
        for frame_index in range(36):
            values = list(frames[frame_index].distance_mm)
            values[17] = 0
            frames[frame_index] = SimpleNamespace(distance_mm=tuple(values))

        observation = capture_cal.reduce_capture_analysis(
            make_analysis(frames),
            min_valid_fraction=0.90,
        )

        self.assertEqual(observation.valid_count[17], 4)
        self.assertEqual(observation.valid_fraction[17], 0.10)
        self.assertEqual(observation.raw_median_mm[17], 800.0)
        self.assertIsNone(observation.distances_mm[17])
        self.assertEqual(observation.usable_zone_count, 63)
        self.assertTrue(any("1 zone(s)" in warning for warning in observation.warnings))

    def test_mad_and_half_drift_are_reported_without_arbitrary_rejection(self):
        frames = uniform_frames(40, 800.0)
        for frame_index in range(20, 40):
            values = list(frames[frame_index].distance_mm)
            values[3] = 812.0
            frames[frame_index] = SimpleNamespace(distance_mm=tuple(values))

        observation = capture_cal.reduce_capture_analysis(make_analysis(frames))

        self.assertTrue(observation.structurally_valid)
        self.assertEqual(observation.distances_mm[3], 806.0)
        self.assertEqual(observation.mad_mm[3], 6.0)
        self.assertEqual(observation.half_drift_mm[3], 12.0)

    def test_too_few_frames_is_structural_error(self):
        observation = capture_cal.reduce_capture_analysis(
            make_analysis(uniform_frames(10)),
            min_frames=30,
        )
        self.assertFalse(observation.structurally_valid)
        self.assertTrue(any("at least 30" in error for error in observation.structural_errors))

    def test_stream_and_health_failures_are_structural_errors(self):
        stats = FakeStreamStats(
            semantic_errors=2,
            sequence_gaps=3,
            health_deltas={"tof_errors": 1},
        )
        analysis = make_analysis(
            uniform_frames(40),
            decoder=SimpleNamespace(frames_bad=4),
            stream_stats=stats,
            metadata_errors=("packets sha256 mismatch",),
        )
        observation = capture_cal.reduce_capture_analysis(analysis)
        joined = " | ".join(observation.structural_errors)
        self.assertIn("4 bad frame", joined)
        self.assertIn("2 semantic error", joined)
        self.assertIn("3 missing sequence", joined)
        self.assertIn("tof_errors increased by 1", joined)
        self.assertIn("metadata parity", joined)

    def test_missing_health_interval_is_warning_not_invented_success(self):
        stats = FakeStreamStats(health_deltas={})
        observation = capture_cal.reduce_capture_analysis(
            make_analysis(uniform_frames(40), stream_stats=stats)
        )
        self.assertTrue(observation.structurally_valid)
        self.assertTrue(any("no STATUS interval" in warning for warning in observation.warnings))

    def test_robust_observation_attaches_to_known_plane_pose(self):
        frames = uniform_frames(40, 700.0)
        observation = capture_cal.reduce_capture_analysis(make_analysis(frames))
        pose = workflow.KnownPlanePose.centre_on_optical_axis(
            700.0,
            rotation_x_deg=15.0,
        )

        plane = capture_cal.calibration_plane_from_observation(
            pose,
            observation,
            label="rx-plus15",
        )

        self.assertEqual(plane.distances_mm, observation.distances_mm)
        self.assertEqual(plane.label, "rx-plus15")

    def test_structurally_invalid_observation_cannot_be_used_for_solver(self):
        observation = capture_cal.reduce_capture_analysis(
            make_analysis(uniform_frames(5)),
            min_frames=30,
        )
        pose = workflow.KnownPlanePose.centre_on_optical_axis(700.0)
        with self.assertRaises(capture_cal.CalibrationCaptureError):
            capture_cal.calibration_plane_from_observation(pose, observation)

    def test_invalid_reduction_parameters_are_rejected(self):
        analysis = make_analysis(uniform_frames(40))
        for kwargs in (
            {"min_frames": 0},
            {"min_valid_fraction": 0.0},
            {"min_valid_fraction": 1.1},
        ):
            with self.subTest(kwargs=kwargs):
                with self.assertRaises(capture_cal.CalibrationCaptureError):
                    capture_cal.reduce_capture_analysis(analysis, **kwargs)


if __name__ == "__main__":
    unittest.main()
