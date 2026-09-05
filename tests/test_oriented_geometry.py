"""Tests for rotation-only ToF projection into local_reference."""

from math import cos, pi, sin, sqrt
from pathlib import Path
import sys
from types import SimpleNamespace
import unittest

HERE = Path(__file__).resolve().parent
REPO_ROOT = HERE.parent
HOST_PYTHON = REPO_ROOT / "host" / "python"
if str(HOST_PYTHON) not in sys.path:
    sys.path.insert(0, str(HOST_PYTHON))

import rangeweave_geometry as geometry
import rangeweave_orientation as orientation
import rangeweave_oriented_geometry as oriented
import rangeweave_tof_timing as timing


IDENTITY = (
    (1.0, 0.0, 0.0),
    (0.0, 1.0, 0.0),
    (0.0, 0.0, 1.0),
)


class _ClockFit:
    def tick_to_us(self, tick):
        return float(tick)


def _orientation_sample(time_s, quaternion):
    return orientation.OrientationSample(
        time_s=float(time_s),
        quaternion_reference_from_body=quaternion,
        reference_from_body=orientation.quaternion_to_matrix(quaternion),
        accel_weight=1.0,
        gravity_innovation_deg=0.0,
    )


def _run(q0=(1.0, 0.0, 0.0, 0.0), q1=(1.0, 0.0, 0.0, 0.0)):
    return SimpleNamespace(
        samples=(
            _orientation_sample(0.0, q0),
            _orientation_sample(1.0, q1),
        ),
        clock_fit=_ClockFit(),
    )


def _timing(offset_ms=0.0, role=timing.ROLE_NOMINAL):
    return timing.TimingResolution(
        mode=timing.MODE_QUICK_START,
        role=role,
        effective_offset_ms=float(offset_ms),
        source="test timing",
        artifact_name="test-profile",
        fingerprint={},
    )


def _grid(distances, ready_us=500_000, rows=8, cols=8):
    return SimpleNamespace(
        distance_mm=tuple(distances),
        rows=rows,
        cols=cols,
        mcu_ready_us=int(ready_us),
    )


def _single_x_profile():
    slopes = [(0.0, 0.0)] * geometry.ZONE_COUNT
    slopes[0] = (1.0, 0.0)
    return geometry.TofGeometryProfile(
        name="test-single-x-ray",
        role="test",
        xy_per_z=tuple(slopes),
    )


class OrientedGeometryTests(unittest.TestCase):
    def test_identity_rotation_preserves_projection_and_invalid_zone_slots(self):
        distances = [1000.0] * geometry.ZONE_COUNT
        distances[7] = 0.0
        grid = _grid(distances)

        frame = oriented.project_tof_grid_to_reference(
            grid,
            (SimpleNamespace(lsm_tick=0),),
            _run(),
            IDENTITY,
            _timing(),
        )

        expected = geometry.project_distances_mm(distances)
        self.assertEqual(len(frame.points_reference), geometry.ZONE_COUNT)
        self.assertEqual(frame.valid_point_count, geometry.ZONE_COUNT - 1)
        self.assertIsNone(frame.points_reference[7])
        self.assertEqual(frame.points_reference[0], expected[0])
        self.assertEqual(frame.geometry_role, geometry.GEOMETRY_PROFILE_ROLE)
        self.assertAlmostEqual(frame.time_s, 0.5)

    def test_transform_order_is_tof_then_body_then_reference(self):
        q_z_90 = (sqrt(0.5), 0.0, 0.0, sqrt(0.5))
        r_z_90 = orientation.quaternion_to_matrix(q_z_90)
        distances = [100.0] * geometry.ZONE_COUNT

        frame = oriented.project_tof_grid_to_reference(
            _grid(distances),
            (SimpleNamespace(lsm_tick=0),),
            _run(q_z_90, q_z_90),
            r_z_90,
            _timing(),
            geometry_profile=_single_x_profile(),
        )

        # Zone 0 starts at (100, 0, 100).  Two successive +90 deg Z rotations
        # produce (-100, 0, 100).  This catches transform reversal/order mistakes.
        point = frame.points_reference[0]
        self.assertIsNotNone(point)
        self.assertAlmostEqual(point.x_mm, -100.0, places=6)
        self.assertAlmostEqual(point.y_mm, 0.0, places=6)
        self.assertAlmostEqual(point.z_mm, 100.0, places=6)

    def test_timing_resolution_controls_attitude_interpolation_time(self):
        q_z_90 = (sqrt(0.5), 0.0, 0.0, sqrt(0.5))
        distances = [100.0] * geometry.ZONE_COUNT

        frame = oriented.project_tof_grid_to_reference(
            _grid(distances, ready_us=500_000),
            (SimpleNamespace(lsm_tick=0),),
            _run((1.0, 0.0, 0.0, 0.0), q_z_90),
            IDENTITY,
            _timing(offset_ms=250.0),
            geometry_profile=_single_x_profile(),
        )

        self.assertAlmostEqual(frame.time_s, 0.75)
        point = frame.points_reference[0]
        self.assertIsNotNone(point)
        angle = 0.75 * pi / 2.0
        self.assertAlmostEqual(point.x_mm, 100.0 * cos(angle), places=5)
        self.assertAlmostEqual(point.y_mm, 100.0 * sin(angle), places=5)
        self.assertAlmostEqual(point.z_mm, 100.0, places=5)
        self.assertEqual(frame.timing_offset_ms, 250.0)

    def test_timing_outside_orientation_run_is_rejected(self):
        with self.assertRaisesRegex(oriented.OrientedGeometryError, "outside the orientation run"):
            oriented.project_tof_grid_to_reference(
                _grid([100.0] * geometry.ZONE_COUNT, ready_us=900_000),
                (SimpleNamespace(lsm_tick=0),),
                _run(),
                IDENTITY,
                _timing(offset_ms=200.0),
            )

    def test_wrong_grid_shape_is_rejected(self):
        with self.assertRaisesRegex(oriented.OrientedGeometryError, "expected 8x8"):
            oriented.project_tof_grid_to_reference(
                _grid([100.0] * geometry.ZONE_COUNT, rows=4, cols=16),
                (SimpleNamespace(lsm_tick=0),),
                _run(),
                IDENTITY,
                _timing(),
            )

    def test_missing_distance_field_is_rejected(self):
        grid = SimpleNamespace(rows=8, cols=8, mcu_ready_us=500_000)
        with self.assertRaisesRegex(oriented.OrientedGeometryError, "does not contain distance_mm"):
            oriented.project_tof_grid_to_reference(
                grid,
                (SimpleNamespace(lsm_tick=0),),
                _run(),
                IDENTITY,
                _timing(),
            )


if __name__ == "__main__":
    unittest.main()
