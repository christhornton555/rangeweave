"""Tests for VL53L5CX zone projection into Rangeweave tof_optical."""

from pathlib import Path
import math
import sys
import unittest

HERE = Path(__file__).resolve().parent
REPO_ROOT = HERE.parent
HOST_PYTHON = REPO_ROOT / "host" / "python"
if str(HOST_PYTHON) not in sys.path:
    sys.path.insert(0, str(HOST_PYTHON))

import rangeweave_geometry as geometry
import view_point_cloud as point_view


class ZoneMappingTests(unittest.TestCase):
    def test_four_corner_physical_mapping(self):
        self.assertEqual(geometry.physical_row_col(63), (0, 0))
        self.assertEqual(geometry.physical_row_col(56), (0, 7))
        self.assertEqual(geometry.physical_row_col(7), (7, 0))
        self.assertEqual(geometry.physical_row_col(0), (7, 7))

    def test_bad_zone_is_rejected(self):
        with self.assertRaises(geometry.GeometryError):
            geometry.projection_vector(-1)
        with self.assertRaises(geometry.GeometryError):
            geometry.projection_vector(64)


class ProjectionTests(unittest.TestCase):
    def assertPointAlmostEqual(self, point, expected, places=5):
        self.assertAlmostEqual(point.x_mm, expected[0], places=places)
        self.assertAlmostEqual(point.y_mm, expected[1], places=places)
        self.assertAlmostEqual(point.z_mm, expected[2], places=places)

    def test_golden_corner_points_at_one_metre_axial(self):
        self.assertPointAlmostEqual(
            geometry.project_axial_distance_mm(0, 1000),
            (362.623824, 362.623824, 1000.0),
        )
        self.assertPointAlmostEqual(
            geometry.project_axial_distance_mm(7, 1000),
            (-362.623824, 362.623824, 1000.0),
        )
        self.assertPointAlmostEqual(
            geometry.project_axial_distance_mm(56, 1000),
            (362.623824, -362.623824, 1000.0),
        )
        self.assertPointAlmostEqual(
            geometry.project_axial_distance_mm(63, 1000),
            (-362.623824, -362.623824, 1000.0),
        )

    def test_central_four_zones_are_symmetric(self):
        expected = 49.445723
        self.assertPointAlmostEqual(
            geometry.project_axial_distance_mm(27, 1000),
            (expected, expected, 1000.0),
        )
        self.assertPointAlmostEqual(
            geometry.project_axial_distance_mm(28, 1000),
            (-expected, expected, 1000.0),
        )
        self.assertPointAlmostEqual(
            geometry.project_axial_distance_mm(35, 1000),
            (expected, -expected, 1000.0),
        )
        self.assertPointAlmostEqual(
            geometry.project_axial_distance_mm(36, 1000),
            (-expected, -expected, 1000.0),
        )

    def test_flat_axial_wall_stays_flat(self):
        points = geometry.project_distances_mm([1000] * 64)
        self.assertEqual(len(points), 64)
        for point in points:
            self.assertIsNotNone(point)
            self.assertEqual(point.z_mm, 1000.0)

    def test_unit_ray_is_normalized_but_not_distance_contract(self):
        ray = geometry.unit_ray(0)
        self.assertAlmostEqual(math.sqrt(sum(component * component for component in ray)), 1.0)
        # The normalized ray Z component is less than one for an off-axis zone;
        # multiplying the sensor's axial distance by this ray would therefore be wrong.
        self.assertLess(ray[2], 1.0)
        point = geometry.project_axial_distance_mm(0, 1000)
        self.assertEqual(point.z_mm, 1000.0)

    def test_invalid_distances_preserve_zone_slots_as_none(self):
        distances = [1000] * 64
        distances[3] = 0
        distances[9] = float("nan")
        points = geometry.project_distances_mm(distances)
        self.assertIsNone(points[3])
        self.assertIsNone(points[9])
        self.assertIsNotNone(points[0])

    def test_wrong_grid_length_is_rejected(self):
        with self.assertRaises(geometry.GeometryError):
            geometry.project_distances_mm([1000] * 63)

    def test_single_invalid_distance_is_rejected(self):
        for bad in (0, -1, float("nan"), float("inf")):
            with self.subTest(bad=bad):
                with self.assertRaises(geometry.GeometryError):
                    geometry.project_axial_distance_mm(0, bad)


class PointCloudViewerPolicyTests(unittest.TestCase):
    @staticmethod
    def point(z):
        return geometry.Point3(x_mm=0.0, y_mm=0.0, z_mm=float(z))

    def test_connected_runs_break_at_depth_discontinuity(self):
        points = [self.point(300), self.point(320), self.point(850), self.point(870)]
        runs = point_view.connected_runs(points, 150)
        self.assertEqual([[point.z_mm for point in run] for run in runs], [[300.0, 320.0], [850.0, 870.0]])

    def test_connected_runs_break_at_invalid_zone(self):
        points = [self.point(300), self.point(320), None, self.point(330), self.point(340)]
        runs = point_view.connected_runs(points, 150)
        self.assertEqual([[point.z_mm for point in run] for run in runs], [[300.0, 320.0], [330.0, 340.0]])

    def test_singletons_are_not_drawn_as_mesh_edges(self):
        points = [self.point(300), self.point(900)]
        self.assertEqual(point_view.connected_runs(points, 150), [])

    def test_non_positive_link_threshold_is_rejected(self):
        for threshold in (0, -1):
            with self.subTest(threshold=threshold):
                with self.assertRaises(ValueError):
                    point_view.connected_runs([self.point(300), self.point(310)], threshold)


if __name__ == "__main__":
    unittest.main()
