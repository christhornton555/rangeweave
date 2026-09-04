"""Tests for stationary-edge gyro bias inspection."""

from pathlib import Path
import sys
from types import SimpleNamespace
import unittest

HERE = Path(__file__).resolve().parent
REPO_ROOT = HERE.parent
HOST_PYTHON = REPO_ROOT / "host" / "python"
if str(HOST_PYTHON) not in sys.path:
    sys.path.insert(0, str(HOST_PYTHON))

import inspect_imu_bias_edges as bias_edges


def sample(gx=0, gy=0, gz=0):
    return SimpleNamespace(gyro_x=gx, gyro_y=gy, gyro_z=gz)


class ImuBiasEdgeInspectionTests(unittest.TestCase):
    def test_reports_start_end_and_shift_separately(self):
        samples = [sample(10, -20, 30) for _ in range(20)]
        samples += [sample(1000, -2000, 3000) for _ in range(60)]
        samples += [sample(14, -120, 34) for _ in range(20)]

        result = bias_edges.analyse_bias_edges(samples)

        self.assertEqual(result["initial_gyro_raw"], (10.0, -20.0, 30.0))
        self.assertEqual(result["final_gyro_raw"], (14.0, -120.0, 34.0))
        self.assertEqual(result["delta_gyro_raw"], (4.0, -100.0, 4.0))

    def test_too_few_samples_rejected(self):
        with self.assertRaises(bias_edges.ImuBiasEdgeError):
            bias_edges.analyse_bias_edges([sample()] * 10)


if __name__ == "__main__":
    unittest.main()
