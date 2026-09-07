"""Tests for independent magnetometer hard-iron transfer helpers."""

from pathlib import Path
import sys
import unittest

HERE = Path(__file__).resolve().parent
REPO_ROOT = HERE.parent
HOST_PYTHON = REPO_ROOT / "host" / "python"
if str(HOST_PYTHON) not in sys.path:
    sys.path.insert(0, str(HOST_PYTHON))

import validate_magnetometer_hard_iron_transfer as validator


class MagnetometerHardIronTransferTests(unittest.TestCase):
    def test_centre_delta_reports_components_and_magnitude(self):
        components, magnitude = validator.centre_delta_ut(
            (-40.0, -18.0, 9.0),
            (-39.0, -20.0, 11.0),
        )
        self.assertEqual(components, (1.0, -2.0, 2.0))
        self.assertAlmostEqual(magnitude, 3.0)

    def test_centre_delta_requires_three_components(self):
        with self.assertRaises(ValueError):
            validator.centre_delta_ut((1.0, 2.0), (1.0, 2.0, 3.0))  # type: ignore[arg-type]


if __name__ == "__main__":
    unittest.main()
