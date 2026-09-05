"""Tests for the empirical Phase 3 rotation-in-place wall gate."""

from pathlib import Path
import sys
from types import SimpleNamespace
import unittest

HERE = Path(__file__).resolve().parent
REPO_ROOT = HERE.parent
HOST_PYTHON = REPO_ROOT / "host" / "python"
if str(HOST_PYTHON) not in sys.path:
    sys.path.insert(0, str(HOST_PYTHON))

import rangeweave_phase3_gate as gate


def _result(**overrides):
    values = {
        "total_tof_grids": 452,
        "observations": tuple(range(443)),
        "orientation_excursion_deg": 38.503,
        "residual_rms_deg": 0.776,
        "residual_p95_deg": 1.664,
        "residual_max_deg": 3.530,
        "start_end_error_deg": 0.703,
    }
    values.update(overrides)
    return SimpleNamespace(**values)


class Phase3WallGateTests(unittest.TestCase):
    def test_worse_reference_capture_passes(self):
        assessment = gate.assess_wall_stability(_result())
        self.assertTrue(assessment.passed)
        self.assertFalse(assessment.failures)
        self.assertGreater(assessment.usable_fraction, gate.MIN_USABLE_FRACTION)

    def test_each_limit_can_reject(self):
        cases = (
            {"observations": tuple(range(420))},
            {"orientation_excursion_deg": 19.99},
            {"residual_rms_deg": 1.01},
            {"residual_p95_deg": 2.01},
            {"residual_max_deg": 5.01},
            {"start_end_error_deg": 1.01},
        )
        for overrides in cases:
            with self.subTest(overrides=overrides):
                assessment = gate.assess_wall_stability(_result(**overrides))
                self.assertFalse(assessment.passed)
                self.assertTrue(assessment.failures)

    def test_exact_boundaries_pass(self):
        total = 100
        assessment = gate.assess_wall_stability(
            _result(
                total_tof_grids=total,
                observations=tuple(range(95)),
                orientation_excursion_deg=20.0,
                residual_rms_deg=1.0,
                residual_p95_deg=2.0,
                residual_max_deg=5.0,
                start_end_error_deg=1.0,
            )
        )
        self.assertTrue(assessment.passed)

    def test_invalid_total_is_rejected(self):
        with self.assertRaises(ValueError):
            gate.assess_wall_stability(_result(total_tof_grids=0, observations=()))


if __name__ == "__main__":
    unittest.main()
