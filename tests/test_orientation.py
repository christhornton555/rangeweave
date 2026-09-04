"""Golden tests for Rangeweave Phase 3 attitude conventions and orientation core."""

from pathlib import Path
import math
import sys
import unittest

HERE = Path(__file__).resolve().parent
REPO_ROOT = HERE.parent
HOST_PYTHON = REPO_ROOT / "host" / "python"
if str(HOST_PYTHON) not in sys.path:
    sys.path.insert(0, str(HOST_PYTHON))

import rangeweave_extrinsics as ext
import rangeweave_orientation as ori


def _assert_matrix_close(testcase, actual, expected, places=7):
    for row in range(3):
        for col in range(3):
            testcase.assertAlmostEqual(actual[row][col], expected[row][col], places=places)


class QuaternionConventionTests(unittest.TestCase):
    def test_identity_quaternion_maps_to_identity_matrix(self):
        _assert_matrix_close(
            self,
            ori.quaternion_to_matrix((1.0, 0.0, 0.0, 0.0)),
            ext.identity_matrix(),
        )

    def test_positive_x_golden_rotation(self):
        root_half = math.sqrt(0.5)
        matrix = ori.quaternion_to_matrix((root_half, root_half, 0.0, 0.0))
        _assert_matrix_close(self, matrix, ext.rotation_x_deg(90.0))
        self.assertEqual(
            tuple(round(value, 7) for value in ext.matrix_vector(matrix, (0.0, 1.0, 0.0))),
            (0.0, 0.0, 1.0),
        )
        self.assertEqual(
            tuple(round(value, 7) for value in ext.matrix_vector(matrix, (0.0, 0.0, 1.0))),
            (0.0, -1.0, 0.0),
        )

    def test_hamilton_composition_matches_matrix_composition(self):
        q_a_from_b = ori.matrix_to_quaternion(ext.rotation_z_deg(30.0))
        q_b_from_c = ori.matrix_to_quaternion(ext.rotation_x_deg(-20.0))
        q_a_from_c = ori.quaternion_multiply(q_a_from_b, q_b_from_c)
        expected = ext.matrix_multiply(
            ext.rotation_z_deg(30.0),
            ext.rotation_x_deg(-20.0),
        )
        _assert_matrix_close(self, ori.quaternion_to_matrix(q_a_from_c), expected)

    def test_matrix_round_trip(self):
        matrix = ext.rotation_xyz_deg(17.0, -23.0, 41.0)
        quaternion = ori.matrix_to_quaternion(matrix)
        _assert_matrix_close(self, ori.quaternion_to_matrix(quaternion), matrix)
        self.assertGreaterEqual(quaternion[0], 0.0)


class LocalReferenceInitialisationTests(unittest.TestCase):
    def test_level_body_initialises_to_identity(self):
        matrix = ori.initial_reference_from_body((0.0, -1.0, 0.0))
        _assert_matrix_close(self, matrix, ext.identity_matrix())

    def test_tilt_about_x_is_recovered_from_specific_force(self):
        expected = ext.rotation_x_deg(20.0)
        up_body = ext.matrix_vector(ext.transpose(expected), ori.UP_REFERENCE)
        actual = ori.initial_reference_from_body(up_body)
        _assert_matrix_close(self, actual, expected)

    def test_accel_confidence_is_magnitude_gated(self):
        self.assertAlmostEqual(ori.accel_confidence((0.0, -1.02, 0.0)), 1.0)
        self.assertAlmostEqual(ori.accel_confidence((0.0, -1.30, 0.0)), 0.0)
        middle = ori.accel_confidence((0.0, -1.125, 0.0))
        self.assertGreater(middle, 0.0)
        self.assertLess(middle, 1.0)


class SixAxisOrientationFilterTests(unittest.TestCase):
    def test_constant_body_x_rate_right_multiplies_attitude(self):
        filter_ = ori.SixAxisOrientationFilter(
            quaternion_reference_from_body=(1.0, 0.0, 0.0, 0.0),
            gravity_gain_per_s=0.0,
        )
        dt = 0.01
        for index in range(101):
            filter_.observe(
                index * dt,
                (90.0, 0.0, 0.0),
                (0.0, -1.0, 0.0),
            )
        _assert_matrix_close(self, filter_.reference_from_body, ext.rotation_x_deg(90.0), places=5)

    def test_gravity_correction_reduces_observable_roll_error(self):
        initial = ori.matrix_to_quaternion(ext.rotation_z_deg(10.0))
        filter_ = ori.SixAxisOrientationFilter(
            quaternion_reference_from_body=initial,
            gravity_gain_per_s=4.0,
        )
        first = filter_.observe(0.0, (0.0, 0.0, 0.0), (0.0, -1.0, 0.0))
        last = first
        for index in range(1, 301):
            last = filter_.observe(
                index * 0.01,
                (0.0, 0.0, 0.0),
                (0.0, -1.0, 0.0),
            )
        self.assertGreater(first.gravity_innovation_deg, 9.0)
        self.assertLess(last.gravity_innovation_deg, 0.1)

    def test_yaw_about_gravity_is_not_corrected_by_accelerometer(self):
        expected = ext.rotation_y_deg(30.0)
        filter_ = ori.SixAxisOrientationFilter(
            quaternion_reference_from_body=ori.matrix_to_quaternion(expected),
            gravity_gain_per_s=4.0,
        )
        last = None
        for index in range(101):
            last = filter_.observe(
                index * 0.01,
                (0.0, 0.0, 0.0),
                (0.0, -1.0, 0.0),
            )
        self.assertIsNotNone(last)
        self.assertAlmostEqual(last.gravity_innovation_deg, 0.0, places=6)
        _assert_matrix_close(self, filter_.reference_from_body, expected, places=6)

    def test_high_specific_force_disables_gravity_correction(self):
        initial = ori.matrix_to_quaternion(ext.rotation_z_deg(10.0))
        filter_ = ori.SixAxisOrientationFilter(
            quaternion_reference_from_body=initial,
            gravity_gain_per_s=4.0,
        )
        last = None
        for index in range(101):
            last = filter_.observe(
                index * 0.01,
                (0.0, 0.0, 0.0),
                (0.0, -1.30, 0.0),
            )
        self.assertIsNotNone(last)
        self.assertEqual(last.accel_weight, 0.0)
        _assert_matrix_close(self, filter_.reference_from_body, ext.rotation_z_deg(10.0), places=6)


if __name__ == "__main__":
    unittest.main()
