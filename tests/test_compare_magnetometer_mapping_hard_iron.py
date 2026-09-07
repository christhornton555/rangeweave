"""Tests for the raw-vs-centred magnetometer mapping comparison helper."""

from pathlib import Path
import math
import sys
import unittest

HERE = Path(__file__).resolve().parent
REPO_ROOT = HERE.parent
HOST_PYTHON = REPO_ROOT / "host" / "python"
if str(HOST_PYTHON) not in sys.path:
    sys.path.insert(0, str(HOST_PYTHON))

import compare_magnetometer_mapping_hard_iron as inspector
import rangeweave_magnetometer as mag


class MagnetometerHardIronMappingComparisonTests(unittest.TestCase):
    def test_centre_samples_removes_additive_bias_and_preserves_metadata(self):
        sample = mag.MagneticSample(
            time_us=1234.5,
            read_duration_us=42,
            raw_x=100,
            raw_y=-200,
            raw_z=300,
            x_ut=13.0,
            y_ut=-2.0,
            z_ut=8.0,
            norm_ut=math.sqrt(13.0**2 + (-2.0)**2 + 8.0**2),
            status_reg=0x88,
            read_flags=1,
        )

        corrected = inspector.centre_samples((sample,), (3.0, -4.0, 5.0))[0]

        self.assertEqual((corrected.x_ut, corrected.y_ut, corrected.z_ut), (10.0, 2.0, 3.0))
        self.assertAlmostEqual(corrected.norm_ut, math.sqrt(113.0))
        self.assertEqual(corrected.time_us, sample.time_us)
        self.assertEqual(corrected.read_duration_us, sample.read_duration_us)
        self.assertEqual((corrected.raw_x, corrected.raw_y, corrected.raw_z), (100, -200, 300))
        self.assertEqual(corrected.status_reg, sample.status_reg)
        self.assertEqual(corrected.read_flags, sample.read_flags)

    def test_centre_requires_three_components(self):
        with self.assertRaises(ValueError):
            inspector.centre_samples((), (1.0, 2.0))  # type: ignore[arg-type]


if __name__ == "__main__":
    unittest.main()
