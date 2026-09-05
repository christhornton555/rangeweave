"""Tests for orthogonal flat-plane diagnostics in local_reference."""

import math
from pathlib import Path
import sys
import unittest

HERE = Path(__file__).resolve().parent
REPO_ROOT = HERE.parent
HOST_PYTHON = REPO_ROOT / "host" / "python"
if str(HOST_PYTHON) not in sys.path:
    sys.path.insert(0, str(HOST_PYTHON))

import rangeweave_geometry as geometry
import rangeweave_reference_plane as plane


def normalise(values):
    length = math.sqrt(sum(value * value for value in values))
    return tuple(value / length for value in values)


def synthetic_points(normal, offset=500.0):
    nx, ny, nz = normalise(normal)
    points = []
    for x in (-150.0, -100.0, -50.0, 0.0, 50.0, 100.0, 150.0):
        for y in (-120.0, -60.0, 0.0, 60.0, 120.0):
            z = (offset - nx * x - ny * y) / nz
            points.append(geometry.Point3(x, y, z))
    return tuple(points)


class ReferencePlaneTests(unittest.TestCase):
    def test_orthogonal_fit_recovers_sloped_plane_normal(self):
        expected = normalise((0.20, -0.10, 1.0))
        fit = plane.fit_plane(synthetic_points(expected))
        self.assertLess(plane.angle_deg(fit.normal, expected), 1.0e-6)
        self.assertAlmostEqual(fit.rms_residual_mm, 0.0, places=8)
        self.assertAlmostEqual(fit.max_abs_residual_mm, 0.0, places=8)
        self.assertEqual(fit.point_count, 35)

    def test_fit_is_translation_invariant_in_normal(self):
        expected = normalise((-0.12, 0.08, 1.0))
        original = synthetic_points(expected, offset=450.0)
        shifted = tuple(
            geometry.Point3(point.x_mm + 75.0, point.y_mm - 40.0, point.z_mm + 120.0)
            for point in original
        )
        left = plane.fit_plane(original)
        right = plane.fit_plane(shifted)
        self.assertLess(plane.angle_deg(left.normal, right.normal), 1.0e-6)

    def test_invalid_slots_are_ignored(self):
        expected = normalise((0.05, 0.03, 1.0))
        points = list(synthetic_points(expected))
        points[3] = None
        points[9] = None
        fit = plane.fit_plane(tuple(points))
        self.assertEqual(fit.point_count, 33)
        self.assertLess(plane.angle_deg(fit.normal, expected), 1.0e-6)

    def test_mean_direction_handles_opposite_normal_sign(self):
        expected = normalise((0.08, -0.04, 1.0))
        averaged = plane.mean_direction((expected, tuple(-value for value in expected)))
        self.assertLess(plane.angle_deg(averaged, expected), 1.0e-6)

    def test_collinear_points_are_rejected(self):
        points = tuple(geometry.Point3(float(index), 0.0, 0.0) for index in range(8))
        with self.assertRaises(plane.ReferencePlaneError):
            plane.fit_plane(points)


if __name__ == "__main__":
    unittest.main()
