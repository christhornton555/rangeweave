"""Tests for the dependency-light IMU axis inspection helper."""

from pathlib import Path
import sys
from types import SimpleNamespace
import unittest

HERE = Path(__file__).resolve().parent
REPO_ROOT = HERE.parent
HOST_PYTHON = REPO_ROOT / "host" / "python"
if str(HOST_PYTHON) not in sys.path:
    sys.path.insert(0, str(HOST_PYTHON))

import inspect_imu_axes as imu_axes


def sample(
    *,
    ax=100,
    ay=-200,
    az=8000,
    gx=0,
    gy=0,
    gz=0,
):
    return SimpleNamespace(
        accel_x=ax,
        accel_y=ay,
        accel_z=az,
        gyro_x=gx,
        gyro_y=gy,
        gyro_z=gz,
    )


class ImuAxisInspectionTests(unittest.TestCase):
    def test_positive_y_rotation_is_identified(self):
        samples = [sample() for _ in range(20)]
        samples += [sample(gy=1200) for _ in range(60)]
        samples += [sample(ax=5000, ay=-100, az=6000) for _ in range(20)]

        result = imu_axes.analyse_imu_samples(samples)

        self.assertEqual(result["dominant_axis"], "y")
        self.assertEqual(result["dominant_sign"], 1)
        self.assertGreater(result["dominance_ratio"], 10.0)
        self.assertEqual(result["initial_accel_raw"], (100.0, -200.0, 8000.0))
        self.assertEqual(result["final_accel_raw"], (5000.0, -100.0, 6000.0))

    def test_negative_x_rotation_is_identified_with_bias_removed(self):
        samples = [sample(gx=25, gy=-10, gz=5) for _ in range(20)]
        samples += [sample(gx=-975, gy=-10, gz=5) for _ in range(60)]
        samples += [sample(gx=25, gy=-10, gz=5) for _ in range(20)]

        result = imu_axes.analyse_imu_samples(samples)

        self.assertEqual(result["gyro_bias_raw"], (25.0, -10.0, 5.0))
        self.assertEqual(result["dominant_axis"], "x")
        self.assertEqual(result["dominant_sign"], -1)

    def test_current_firmware_scale_decoding(self):
        self.assertAlmostEqual(imu_axes.accel_g_per_lsb(0x48), 0.000122)
        self.assertAlmostEqual(imu_axes.gyro_dps_per_lsb(0x40), 0.00875)

    def test_too_few_samples_rejected(self):
        with self.assertRaises(imu_axes.ImuAxisInspectionError):
            imu_axes.analyse_imu_samples([sample()] * 10)


if __name__ == "__main__":
    unittest.main()
