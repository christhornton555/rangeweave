"""Tests for Rangeweave IMU motion-capture quality checks."""

from pathlib import Path
import sys
from types import SimpleNamespace
import unittest

HERE = Path(__file__).resolve().parent
REPO_ROOT = HERE.parent
HOST_PYTHON = REPO_ROOT / "host" / "python"
if str(HOST_PYTHON) not in sys.path:
    sys.path.insert(0, str(HOST_PYTHON))

import rangeweave_imu_quality as quality


class ImuQualityTests(unittest.TestCase):
    def test_full_scale_decodes_supported_lsm_ranges(self):
        self.assertEqual(quality.gyro_full_scale_dps(0x42), 125.0)
        self.assertEqual(quality.gyro_full_scale_dps(0x40), 250.0)
        self.assertEqual(quality.gyro_full_scale_dps(0x44), 500.0)
        self.assertEqual(quality.gyro_full_scale_dps(0x48), 1000.0)
        self.assertEqual(quality.gyro_full_scale_dps(0x4C), 2000.0)

    def test_range_usage_uses_body_mapping_and_rejects_above_90_percent(self):
        # imu_sensor X maps to -device_body X. At +/-250 dps, 26000 LSB is
        # 227.5 dps, or 91% of configured full scale.
        samples = [
            SimpleNamespace(gyro_x=26000, gyro_y=100, gyro_z=-50),
            SimpleNamespace(gyro_x=-1200, gyro_y=200, gyro_z=300),
        ]
        result = quality.analyse_gyro_range(samples, ctrl2_g=0x40)
        self.assertEqual(result.max_axis, "X")
        self.assertAlmostEqual(result.max_peak_dps, 227.5, places=6)
        self.assertAlmostEqual(result.max_fraction, 0.91, places=6)
        self.assertTrue(result.warning)
        self.assertTrue(result.rejected)

    def test_same_physical_peak_has_headroom_at_500_dps(self):
        # At +/-500 dps the sensitivity doubles, so roughly the same 227.5 dps
        # physical rate is represented by half as many raw counts.
        samples = [SimpleNamespace(gyro_x=13000, gyro_y=0, gyro_z=0)]
        result = quality.analyse_gyro_range(samples, ctrl2_g=0x44)
        self.assertAlmostEqual(result.max_peak_dps, 227.5, places=6)
        self.assertAlmostEqual(result.max_fraction, 0.455, places=6)
        self.assertFalse(result.warning)
        self.assertFalse(result.rejected)

    def test_threshold_order_is_validated(self):
        samples = [SimpleNamespace(gyro_x=0, gyro_y=0, gyro_z=0)]
        with self.assertRaises(quality.ImuQualityError):
            quality.analyse_gyro_range(
                samples,
                ctrl2_g=0x44,
                warning_fraction=0.95,
                reject_fraction=0.90,
            )


if __name__ == "__main__":
    unittest.main()
