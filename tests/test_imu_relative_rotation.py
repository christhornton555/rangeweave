"""Tests for short-baseline Rangeweave relative body rotation."""

from pathlib import Path
import math
import sys
from types import SimpleNamespace
import unittest

HERE = Path(__file__).resolve().parent
REPO_ROOT = HERE.parent
HOST_PYTHON = REPO_ROOT / "host" / "python"
if str(HOST_PYTHON) not in sys.path:
    sys.path.insert(0, str(HOST_PYTHON))

import rangeweave_imu_relative as imu


CTRL1_XL = 0x48
CTRL2_G = 0x40
ACCEL_SCALE = imu.accel_g_per_lsb(CTRL1_XL)
GYRO_SCALE = imu.gyro_dps_per_lsb(CTRL2_G)


def _body_to_sensor(vector):
    # The validated permutation/sign matrix is symmetric, so its inverse is itself.
    return imu.imu_vector_to_body(vector)


def _rotation(axis, angle_deg):
    angle = math.radians(angle_deg)
    c, s = math.cos(angle), math.sin(angle)
    if axis == "x":
        return ((1.0, 0.0, 0.0), (0.0, c, -s), (0.0, s, c))
    if axis == "y":
        return ((c, 0.0, s), (0.0, 1.0, 0.0), (-s, 0.0, c))
    if axis == "z":
        return ((c, -s, 0.0), (s, c, 0.0), (0.0, 0.0, 1.0))
    raise ValueError(axis)


def _transpose(matrix):
    return tuple(tuple(matrix[col][row] for col in range(3)) for row in range(3))


def _matvec(matrix, vector):
    return tuple(
        sum(matrix[row][col] * vector[col] for col in range(3))
        for row in range(3)
    )


def synthetic_capture(axis="x", angle_deg=20.0):
    duration_s = 10.0
    dt_s = 0.01
    move_start_s = 3.0
    move_end_s = 5.0
    true_rate_dps = angle_deg / (move_end_s - move_start_s)
    axis_index = {"x": 0, "y": 1, "z": 2}[axis]

    tick_us = 25.0
    ticks_per_second = 1_000_000.0 / tick_us
    sample_count = int(round(duration_s / dt_s)) + 1
    samples = []
    gravity_reference = (0.0, -1.0, 0.0)

    # Deliberately include modest linear gyro-bias drift. The estimator should use
    # stationary endpoint estimates and interpolate between them.
    bias_start = (0.45, -0.25, 0.20)
    bias_end = (0.65, -0.10, 0.30)

    for index in range(sample_count):
        time_s = index * dt_s
        tick = int(round(time_s * ticks_per_second))

        if time_s < move_start_s:
            angle = 0.0
        elif time_s > move_end_s:
            angle = angle_deg
        else:
            angle = (time_s - move_start_s) * true_rate_dps

        true_rate = [0.0, 0.0, 0.0]
        if move_start_s <= time_s <= move_end_s:
            true_rate[axis_index] = true_rate_dps

        fraction = time_s / duration_s
        bias = tuple(
            bias_start[i] + fraction * (bias_end[i] - bias_start[i])
            for i in range(3)
        )
        measured_body_rate = tuple(true_rate[i] + bias[i] for i in range(3))
        sensor_rate = _body_to_sensor(measured_body_rate)

        reference_from_body = _rotation(axis, angle)
        gravity_body = _matvec(_transpose(reference_from_body), gravity_reference)
        gravity_sensor = _body_to_sensor(gravity_body)

        samples.append(
            SimpleNamespace(
                lsm_tick=tick,
                accel_x=int(round(gravity_sensor[0] / ACCEL_SCALE)),
                accel_y=int(round(gravity_sensor[1] / ACCEL_SCALE)),
                accel_z=int(round(gravity_sensor[2] / ACCEL_SCALE)),
                gyro_x=int(round(sensor_rate[0] / GYRO_SCALE)),
                gyro_y=int(round(sensor_rate[1] / GYRO_SCALE)),
                gyro_z=int(round(sensor_rate[2] / GYRO_SCALE)),
            )
        )

    clock_syncs = []
    for second in range(11):
        tick = int(round(second * ticks_per_second))
        midpoint_us = second * 1_000_000.0
        clock_syncs.append(
            SimpleNamespace(
                mcu_before_us=midpoint_us - 10.0,
                lsm_tick=tick,
                mcu_after_us=midpoint_us + 10.0,
            )
        )
    return samples, clock_syncs


class RelativeRotationTests(unittest.TestCase):
    def test_validated_imu_to_body_mapping_is_proper_rotation(self):
        matrix = imu.REFERENCE_ROTATION_BODY_FROM_IMU
        determinant = (
            matrix[0][0] * (matrix[1][1] * matrix[2][2] - matrix[1][2] * matrix[2][1])
            - matrix[0][1] * (matrix[1][0] * matrix[2][2] - matrix[1][2] * matrix[2][0])
            + matrix[0][2] * (matrix[1][0] * matrix[2][1] - matrix[1][1] * matrix[2][0])
        )
        self.assertAlmostEqual(determinant, 1.0)
        self.assertEqual(imu.imu_vector_to_body((1.0, 0.0, 0.0)), (-1.0, 0.0, 0.0))
        self.assertEqual(imu.imu_vector_to_body((0.0, 0.0, 1.0)), (0.0, -1.0, 0.0))
        self.assertEqual(imu.imu_vector_to_body((0.0, 1.0, 0.0)), (0.0, 0.0, -1.0))

    def test_clock_fit_uses_clock_sync_observations(self):
        _, clock_syncs = synthetic_capture()
        fit = imu.fit_lsm_clock(clock_syncs)
        self.assertEqual(fit.observation_count, 11)
        self.assertAlmostEqual(fit.us_per_tick, 25.0, places=9)
        self.assertLess(fit.rms_residual_us, 1.0e-6)
        self.assertAlmostEqual(fit.max_half_bracket_us, 10.0)

    def test_pitch_rotation_recovered_with_bias_drift(self):
        samples, clock_syncs = synthetic_capture("x", 20.0)
        result = imu.estimate_relative_body_rotation(
            samples,
            clock_syncs,
            ctrl1_xl=CTRL1_XL,
            ctrl2_g=CTRL2_G,
        )
        self.assertAlmostEqual(result.rotation_angle_deg, 20.0, delta=0.25)
        self.assertAlmostEqual(result.rotation_xyz_deg[0], 20.0, delta=0.25)
        self.assertAlmostEqual(result.rotation_xyz_deg[1], 0.0, delta=0.10)
        self.assertAlmostEqual(result.rotation_xyz_deg[2], 0.0, delta=0.10)
        self.assertAlmostEqual(result.gravity_direction_change_deg, 20.0, delta=0.10)
        self.assertLess(result.gravity_closure_error_deg, 0.25)
        self.assertLess(result.initial_stationary.gyro_spread_dps, 0.1)
        self.assertLess(result.final_stationary.gyro_spread_dps, 0.1)

    def test_yaw_rotation_recovered_even_though_gravity_does_not_change(self):
        samples, clock_syncs = synthetic_capture("y", 20.0)
        result = imu.estimate_relative_body_rotation(
            samples,
            clock_syncs,
            ctrl1_xl=CTRL1_XL,
            ctrl2_g=CTRL2_G,
        )
        self.assertAlmostEqual(result.rotation_xyz_deg[1], 20.0, delta=0.25)
        self.assertLess(result.gravity_direction_change_deg, 0.05)
        self.assertLess(result.gravity_closure_error_deg, 0.10)

    def test_missing_clock_sync_rejected(self):
        samples, _ = synthetic_capture()
        with self.assertRaises(imu.RelativeRotationError):
            imu.estimate_relative_body_rotation(
                samples,
                [],
                ctrl1_xl=CTRL1_XL,
                ctrl2_g=CTRL2_G,
            )


if __name__ == "__main__":
    unittest.main()
