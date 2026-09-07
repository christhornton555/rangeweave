"""Tests for diagnostic native-frame magnetometer calibration helpers."""

from pathlib import Path
import math
import sys
import unittest

HERE = Path(__file__).resolve().parent
REPO_ROOT = HERE.parent
HOST_PYTHON = REPO_ROOT / "host" / "python"
if str(HOST_PYTHON) not in sys.path:
    sys.path.insert(0, str(HOST_PYTHON))

import rangeweave_magnetometer_calibration as cal


class HardIronSphereFitTests(unittest.TestCase):
    def test_exact_biased_sphere_recovers_centre_and_radius(self):
        centre = (12.5, -8.0, 21.0)
        radius = 48.0
        directions = []
        for x in (-1.0, 0.0, 1.0):
            for y in (-1.0, 0.0, 1.0):
                for z in (-1.0, 0.0, 1.0):
                    if x == y == z == 0.0:
                        continue
                    norm = math.sqrt(x * x + y * y + z * z)
                    directions.append((x / norm, y / norm, z / norm))

        vectors = [
            (
                centre[0] + radius * direction[0],
                centre[1] + radius * direction[1],
                centre[2] + radius * direction[2],
            )
            for direction in directions
        ]

        fit = cal.fit_hard_iron_sphere(vectors)
        self.assertAlmostEqual(fit.center_ut[0], centre[0], places=9)
        self.assertAlmostEqual(fit.center_ut[1], centre[1], places=9)
        self.assertAlmostEqual(fit.center_ut[2], centre[2], places=9)
        self.assertAlmostEqual(fit.radius_ut, radius, places=9)
        self.assertLess(fit.radial_residual_rms_ut, 1e-9)
        self.assertGreater(fit.robust_span_reduction_fraction, 0.999999)

    def test_degenerate_planar_coverage_is_rejected(self):
        vectors = []
        for index in range(16):
            angle = 2.0 * math.pi * index / 16.0
            vectors.append((40.0 * math.cos(angle), 40.0 * math.sin(angle), 5.0))
        with self.assertRaises(cal.MagnetometerCalibrationError):
            cal.fit_hard_iron_sphere(vectors)

    def test_subtract_hard_iron_uses_fitted_centre(self):
        centre = (3.0, -4.0, 5.0)
        radius = 20.0
        directions = (
            (1.0, 0.0, 0.0), (-1.0, 0.0, 0.0),
            (0.0, 1.0, 0.0), (0.0, -1.0, 0.0),
            (0.0, 0.0, 1.0), (0.0, 0.0, -1.0),
            (1.0, 1.0, 1.0), (-1.0, -1.0, -1.0),
        )
        vectors = []
        for direction in directions:
            norm = math.sqrt(sum(value * value for value in direction))
            vectors.append(tuple(centre[i] + radius * direction[i] / norm for i in range(3)))
        fit = cal.fit_hard_iron_sphere(vectors)
        corrected = cal.subtract_hard_iron(vectors[0], fit)
        self.assertAlmostEqual(corrected[0], radius, places=9)
        self.assertAlmostEqual(corrected[1], 0.0, places=9)
        self.assertAlmostEqual(corrected[2], 0.0, places=9)


if __name__ == "__main__":
    unittest.main()
