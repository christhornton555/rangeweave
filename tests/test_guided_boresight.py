from pathlib import Path
import sys
import unittest

HERE = Path(__file__).resolve().parent
REPO_ROOT = HERE.parent
HOST_PYTHON = REPO_ROOT / "host" / "python"
if str(HOST_PYTHON) not in sys.path:
    sys.path.insert(0, str(HOST_PYTHON))

import guided_boresight as guided


class GuidedBoresightTests(unittest.TestCase):
    def test_default_plan_has_four_distinct_motion_steps(self):
        self.assertEqual(len(guided.STEP_INSTRUCTIONS), 4)
        self.assertEqual(len(set(guided.STEP_INSTRUCTIONS)), 4)
        self.assertIn("Rx", guided.STEP_INSTRUCTIONS[0])
        self.assertIn("Ry", guided.STEP_INSTRUCTIONS[1])
        self.assertIn("Rz", guided.STEP_INSTRUCTIONS[3])

    def test_default_motion_schedule_leaves_long_final_hold(self):
        guided._validate_timing(
            warmup_seconds=guided.DEFAULT_WARMUP_SECONDS,
            motion_capture_seconds=guided.DEFAULT_MOTION_CAPTURE_SECONDS,
            recorded_initial_hold_seconds=guided.DEFAULT_RECORDED_INITIAL_HOLD_SECONDS,
            move_seconds=guided.DEFAULT_MOVE_SECONDS,
        )
        final_hold = (
            guided.DEFAULT_MOTION_CAPTURE_SECONDS
            - guided.DEFAULT_RECORDED_INITIAL_HOLD_SECONDS
            - guided.DEFAULT_MOVE_SECONDS
        )
        self.assertGreaterEqual(final_hold, 2.0)

    def test_motion_schedule_rejects_insufficient_final_hold(self):
        with self.assertRaises(ValueError):
            guided._validate_timing(
                warmup_seconds=3.0,
                motion_capture_seconds=8.0,
                recorded_initial_hold_seconds=5.0,
                move_seconds=2.0,
            )

    def test_capture_command_owns_warmup_and_root(self):
        command = guided._capture_command(
            port="COM5",
            root=Path("captures"),
            name="demo-p0",
            seconds=8.0,
            warmup_seconds=3.0,
            notes="test note",
        )
        self.assertIn("capture.py", Path(command[1]).name)
        self.assertEqual(command[2], "COM5")
        self.assertEqual(command[command.index("--warmup") + 1], "3.0")
        self.assertEqual(command[command.index("--seconds") + 1], "8.0")
        self.assertEqual(command[command.index("--root") + 1], "captures")
        self.assertEqual(command[command.index("--name") + 1], "demo-p0")
        self.assertEqual(command[command.index("--notes") + 1], "test note")

    def test_sequence_command_replays_all_prior_pairs_in_order(self):
        baseline = Path("captures/p0")
        steps = [
            (Path("captures/m1"), Path("captures/p1")),
            (Path("captures/m2"), Path("captures/p2")),
        ]
        command = guided._sequence_command(baseline, steps)
        self.assertEqual(command[2:4], ["--baseline", str(baseline)])
        self.assertEqual(
            command[4:],
            [
                "--step",
                "captures/m1",
                "captures/p1",
                "--step",
                "captures/m2",
                "captures/p2",
            ],
        )


if __name__ == "__main__":
    unittest.main()
