"""Tests for stateless fixed-plane boresight sequence composition."""

from pathlib import Path
import sys
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


if __name__ == "__main__":
    unittest.main()
