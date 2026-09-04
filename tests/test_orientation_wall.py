"""Tests for continuous fixed-wall orientation validation."""

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

import rangeweave_extrinsics as ext
import rangeweave_geometry as geometry
import rangeweave_imu_relative as imu
import rangeweave_orientation as ori
import rangeweave_orientation_wall as wall


def _synthetic_plane_distances(normal, offset_mm=1000.0):
    distances = []
    for zone_id in range(geometry.ZONE_COUNT):
        ray = geometry.projection_vector(zone_id)
        denominator = (
            normal[0] * ray.x_per_z
            + normal[1] * ray.y_per_z
            + normal[2]
        )
        if denominator <= 0.05:
            distances.append(0)
        else:
            distances.append(int(round(offset_mm / denominator)))
    return tuple(distances)


def _orientation_run():
    samples = []
    for time_s, yaw_deg in ((0.0, 0.0), (1.0, 10.0), (2.0, 20.0)):
        matrix = ext.rotation_y_deg(yaw_deg)
        samples.append(
            ori.OrientationSample(
                time_s=time_s,
                quaternion_reference_from_body=ori.matrix_to_quaternion(matrix),
                reference_from_body=matrix,
                accel_weight=1.0,
                gravity_innovation_deg=0.0,
            )
        )
    init = ori.InitialisationDiagnostics(
        sample_count=100,
        duration_s=1.0,
        gyro_bias_body_dps=(0.0, 0.0, 0.0),
        gyro_mad_body_dps=(0.0, 0.0, 0.0),
        gyro_spread_dps=0.0,
        accel_median_body_g=(0.0, -1.0, 0.0),
        accel_norm_g=1.0,
        accel_median_deviation_g=0.0,
    )
    clock = imu.ClockFit(
        observation_count=2,
        reference_tick=0.0,
        reference_us=0.0,
        us_per_tick=1.0,
        rms_residual_us=0.0,
        max_abs_residual_us=0.0,
        max_half_bracket_us=0.0,
    )
    return ori.OrientationRun(
        samples=tuple(samples),
        initialisation=init,
        clock_fit=clock,
        imu_mapping_role=imu.REFERENCE_IMU_MAPPING_ROLE,
        gravity_gain_per_s=2.0,
    )


class OrientationWallTests(unittest.TestCase):
    def test_slerp_midpoint_matches_axis_rotation(self):
        q0 = ori.matrix_to_quaternion(ext.identity_matrix())
        q1 = ori.matrix_to_quaternion(ext.rotation_y_deg(20.0))
        midpoint = wall.quaternion_slerp(q0, q1, 0.5)
        matrix = ori.quaternion_to_matrix(midpoint)
        difference = ext.matrix_multiply(
            ext.transpose(ext.rotation_y_deg(10.0)),
            matrix,
        )
        self.assertLess(ext.rotation_angle_deg(difference), 1.0e-6)

    def test_static_reference_wall_remains_stable_under_yaw(self):
        run = _orientation_run()
        reference_normal = (0.0, 0.0, 1.0)
        grids = []
        for index in range(21):
            time_s = index / 10.0
            yaw_deg = 10.0 * time_s
            reference_from_body = ext.rotation_y_deg(yaw_deg)
            normal_body = ext.matrix_vector(
                ext.transpose(reference_from_body),
                reference_normal,
            )
            grids.append(
                SimpleNamespace(
                    mcu_ready_us=int(round(time_s * 1_000_000.0)),
                    rows=8,
                    cols=8,
                    distance_mm=_synthetic_plane_distances(normal_body),
                )
            )

        imu_samples = (SimpleNamespace(lsm_tick=0),)
        result = wall.evaluate_wall_stability(
            grids,
            imu_samples,
            run,
            ext.identity_matrix(),
            min_valid_zones=48,
            max_plane_rms_mm=10.0,
            max_plane_residual_mm=30.0,
            start_end_window_s=0.3,
        )

        self.assertEqual(len(result.observations), 21)
        self.assertLess(result.residual_rms_deg, 0.1)
        self.assertLess(result.residual_p95_deg, 0.1)
        self.assertLess(result.start_end_error_deg, 0.1)
        self.assertAlmostEqual(result.orientation_excursion_deg, 20.0, places=6)
        self.assertLess(wall.angle_deg(result.reference_normal, reference_normal), 0.1)

    def test_time_offset_scan_prefers_zero_for_synthetic_aligned_data(self):
        run = _orientation_run()
        reference_normal = (0.0, 0.0, 1.0)
        grids = []
        for index in range(21):
            time_s = index / 10.0
            reference_from_body = ext.rotation_y_deg(10.0 * time_s)
            normal_body = ext.matrix_vector(ext.transpose(reference_from_body), reference_normal)
            grids.append(
                SimpleNamespace(
                    mcu_ready_us=int(round(time_s * 1_000_000.0)),
                    rows=8,
                    cols=8,
                    distance_mm=_synthetic_plane_distances(normal_body),
                )
            )
        candidates = wall.scan_time_offsets(
            grids,
            (SimpleNamespace(lsm_tick=0),),
            run,
            ext.identity_matrix(),
            (-20.0, 0.0, 20.0),
            start_end_window_s=0.3,
        )
        self.assertTrue(candidates)
        self.assertAlmostEqual(candidates[0].offset_ms, 0.0, places=6)


if __name__ == "__main__":
    unittest.main()
