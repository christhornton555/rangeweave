"""Tests for the optional physical ToF calibration pose convention."""

from pathlib import Path
import math
import sys
import unittest

HERE = Path(__file__).resolve().parent
REPO_ROOT = HERE.parent
HOST_PYTHON = REPO_ROOT / "host" / "python"
if str(HOST_PYTHON) not in sys.path:
    sys.path.insert(0, str(HOST_PYTHON))

import rangeweave_tof_calibration as calibration
import rangeweave_tof_calibration_workflow as workflow


def plane_z_at_xy(normal_and_offset, x_mm, y_mm):
    nx, ny, nz, offset = normal_and_offset
    return (offset - nx * x_mm - ny * y_mm) / nz


class KnownPlanePoseTests(unittest.TestCase):
    def test_fronto_parallel_centre_on_axis_pose_is_z_equals_distance(self):
        pose = workflow.KnownPlanePose.centre_on_optical_axis(800.0)
        nx, ny, nz, offset = pose.normal_and_offset()
        self.assertAlmostEqual(nx, 0.0)
        self.assertAlmostEqual(ny, 0.0)
        self.assertAlmostEqual(nz, 1.0)
        self.assertAlmostEqual(offset, 800.0)

    def test_positive_rx_makes_bottom_farther_and_top_closer(self):
        pose = workflow.KnownPlanePose.centre_on_optical_axis(
            800.0,
            rotation_x_deg=15.0,
        )
        geometry = pose.normal_and_offset()
        nx, ny, nz, offset = geometry
        self.assertAlmostEqual(nx, 0.0, places=12)
        self.assertAlmostEqual(ny, -math.sin(math.radians(15.0)), places=12)
        self.assertAlmostEqual(nz, math.cos(math.radians(15.0)), places=12)
        self.assertAlmostEqual(offset, 800.0 * nz, places=12)
        self.assertGreater(plane_z_at_xy(geometry, 0.0, +100.0), 800.0)
        self.assertLess(plane_z_at_xy(geometry, 0.0, -100.0), 800.0)

    def test_positive_ry_makes_right_side_closer_and_left_side_farther(self):
        pose = workflow.KnownPlanePose.centre_on_optical_axis(
            800.0,
            rotation_y_deg=15.0,
        )
        geometry = pose.normal_and_offset()
        nx, ny, nz, offset = geometry
        self.assertAlmostEqual(nx, math.sin(math.radians(15.0)), places=12)
        self.assertAlmostEqual(ny, 0.0, places=12)
        self.assertAlmostEqual(nz, math.cos(math.radians(15.0)), places=12)
        self.assertAlmostEqual(offset, 800.0 * nz, places=12)
        self.assertLess(plane_z_at_xy(geometry, +100.0, 0.0), 800.0)
        self.assertGreater(plane_z_at_xy(geometry, -100.0, 0.0), 800.0)

    def test_mixed_pose_has_frozen_rx_then_ry_golden_values(self):
        pose = workflow.KnownPlanePose.centre_on_optical_axis(
            800.0,
            rotation_x_deg=12.0,
            rotation_y_deg=10.0,
        )
        nx, ny, nz, offset = pose.normal_and_offset()
        self.assertAlmostEqual(nx, 0.169853548, places=9)
        self.assertAlmostEqual(ny, -0.207911691, places=9)
        self.assertAlmostEqual(nz, 0.963287341, places=9)
        self.assertAlmostEqual(offset, 770.629873, places=6)
        self.assertAlmostEqual(nx * nx + ny * ny + nz * nz, 1.0, places=12)

    def test_arbitrary_known_point_controls_plane_offset(self):
        pose = workflow.KnownPlanePose(
            point_x_mm=50.0,
            point_y_mm=-20.0,
            point_z_mm=800.0,
            rotation_x_deg=12.0,
            rotation_y_deg=10.0,
        )
        nx, ny, nz, offset = pose.normal_and_offset()
        expected = nx * 50.0 + ny * -20.0 + nz * 800.0
        self.assertAlmostEqual(offset, expected, places=12)

    def test_board_distance_may_change_between_captures(self):
        first = workflow.KnownPlanePose.centre_on_optical_axis(
            700.0,
            rotation_x_deg=15.0,
        )
        second = workflow.KnownPlanePose.centre_on_optical_axis(
            825.0,
            rotation_y_deg=-15.0,
        )
        self.assertNotEqual(first.point_z_mm, second.point_z_mm)
        self.assertNotEqual(first.normal_and_offset()[3], second.normal_and_offset()[3])

    def test_pose_can_be_attached_to_one_64_zone_observation(self):
        pose = workflow.KnownPlanePose(
            point_x_mm=10.0,
            point_y_mm=-5.0,
            point_z_mm=700.0,
            rotation_x_deg=-15.0,
            rotation_y_deg=10.0,
        )
        distances = tuple(700.0 + zone for zone in range(64))
        plane = workflow.calibration_plane_from_pose(
            pose,
            distances,
            label="fit-rx-minus15-ry-plus10",
        )
        nx, ny, nz, offset = pose.normal_and_offset()
        self.assertAlmostEqual(plane.normal_x, nx)
        self.assertAlmostEqual(plane.normal_y, ny)
        self.assertAlmostEqual(plane.normal_z, nz)
        self.assertAlmostEqual(plane.offset_mm, offset)
        self.assertEqual(plane.distances_mm, distances)
        self.assertEqual(plane.label, "fit-rx-minus15-ry-plus10")

    def test_invalid_pose_is_rejected(self):
        for kwargs in (
            {"point_x_mm": 0.0, "point_y_mm": 0.0, "point_z_mm": 0.0},
            {"point_x_mm": 0.0, "point_y_mm": 0.0, "point_z_mm": -1.0},
            {"point_x_mm": 0.0, "point_y_mm": 0.0, "point_z_mm": float("nan")},
            {
                "point_x_mm": 0.0,
                "point_y_mm": 0.0,
                "point_z_mm": 800.0,
                "rotation_x_deg": float("inf"),
            },
            {
                "point_x_mm": 0.0,
                "point_y_mm": 0.0,
                "point_z_mm": 800.0,
                "rotation_x_deg": 100.0,
            },
        ):
            with self.subTest(kwargs=kwargs):
                with self.assertRaises(calibration.CalibrationError):
                    workflow.KnownPlanePose(**kwargs)


if __name__ == "__main__":
    unittest.main()
