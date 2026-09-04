from pathlib import Path
from types import SimpleNamespace
import sys
import unittest

HERE = Path(__file__).resolve().parent
REPO_ROOT = HERE.parent
HOST_PYTHON = REPO_ROOT / "host" / "python"
if str(HOST_PYTHON) not in sys.path:
    sys.path.insert(0, str(HOST_PYTHON))

import continue_boresight_validation as continuation
import rangeweave_boresight_sequence as sequence


class ContinueBoresightValidationTests(unittest.TestCase):
    def test_validation_instruction_requests_distinct_multi_axis_pose(self):
        text = continuation.VALIDATION_INSTRUCTION.lower()
        self.assertIn("yaw left", text)
        self.assertIn("pitch", text)
        self.assertIn("roll", text)
        self.assertIn("same wall", text)

    def test_validation_prefix_keeps_source_session(self):
        session = "boresight-guided-20260904_010944"
        prefix = continuation._validation_prefix(session)
        self.assertTrue(prefix.startswith(session + "-validation-"))

    def test_motion_gate_keeps_two_degree_gravity_limit(self):
        motion = SimpleNamespace(rotation_angle_deg=20.0, gravity_closure_error_deg=2.001)
        range_usage = SimpleNamespace(rejected=False)
        with self.assertRaises(sequence.BoresightSequenceError):
            continuation._validate_motion(
                5,
                motion,
                range_usage,
                min_rotation_deg=5.0,
                max_gravity_closure_deg=2.0,
            )

    def test_motion_gate_accepts_clean_validation_motion(self):
        motion = SimpleNamespace(rotation_angle_deg=24.0, gravity_closure_error_deg=0.8)
        range_usage = SimpleNamespace(rejected=False)
        continuation._validate_motion(
            5,
            motion,
            range_usage,
            min_rotation_deg=5.0,
            max_gravity_closure_deg=2.0,
        )


if __name__ == "__main__":
    unittest.main()
