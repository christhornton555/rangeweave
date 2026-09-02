"""Tests for stateless fixed-plane boresight sequence composition."""

from pathlib import Path
import sys
from types import SimpleNamespace
import unittest

HERE = Path(__file__).resolve().parent
REPO_ROOT = HERE.parent
HOST_PYTHON = REPO_ROOT / "host" / "python"
if str(HOST_PYTHON) not in sys.path:
    sys.path.insert(0, str(HOST_PYTHON))

import rangeweave_boresight_sequence as sequence
import rangeweave_extrinsics as ext


class BoresightSequenceTests(unittest.TestCase):
    def test_reference_rotation_composes_in_motion_order(self):
        first = ext.rotation_x_deg(20.0)
        second = ext.rotation_y_deg(-15.0)
        accumulated = sequence.compose_reference_from_body(ext.identity_matrix(), first)
        accumulated = sequence.compose_reference_from_body(accumulated, second)
        expected = ext.matrix_multiply(first, second)
        for row in range(3):
            for col in range(3):
                self.assertAlmostEqual(accumulated[row][col], expected[row][col])

    def test_solve_if_ready_waits_for_four_poses(self):
        self.assertIsNone(sequence.solve_if_ready([]))
        self.assertIsNone(sequence.solve_if_ready([None, None, None]))

    def test_clean_stationary_pose_passes_workflow_quality(self):
        fit = SimpleNamespace(rms_residual_mm=1.5, max_abs_residual_mm=4.0)
        self.assertEqual(sequence.stationary_pose_quality_errors(fit, 2.0), ())

    def test_temporally_moving_pose_is_rejected_even_with_good_plane_fit(self):
        fit = SimpleNamespace(rms_residual_mm=1.5, max_abs_residual_mm=4.0)
        errors = sequence.stationary_pose_quality_errors(fit, 34.0)
        self.assertEqual(len(errors), 1)
        self.assertIn("half-drift", errors[0])

    def test_off_target_plane_is_rejected(self):
        fit = SimpleNamespace(rms_residual_mm=59.0, max_abs_residual_mm=350.0)
        errors = sequence.stationary_pose_quality_errors(fit, 2.0)
        self.assertEqual(len(errors), 2)
        self.assertIn("plane RMS", errors[0])
        self.assertIn("plane max residual", errors[1])

    def test_missing_temporal_stability_measurement_is_rejected(self):
        fit = SimpleNamespace(rms_residual_mm=1.5, max_abs_residual_mm=4.0)
        errors = sequence.stationary_pose_quality_errors(fit, None)
        self.assertEqual(errors, ("no finite ToF half-capture drift measurement is available",))

    def test_pose_quality_limits_require_positive_thresholds(self):
        with self.assertRaises(ValueError):
            sequence.BoresightPoseQualityLimits(max_half_drift_mm=0.0)


if __name__ == "__main__":
    unittest.main()
