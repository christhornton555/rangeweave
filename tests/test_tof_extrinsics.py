"""Tests for live plane alignment and ToF/device-body rotational extrinsics."""

import math
from pathlib import Path
import sys
import unittest

HERE = Path(__file__).resolve().parent
REPO_ROOT = HERE.parent
HOST_PYTHON = REPO_ROOT / "host" / "python"
if str(HOST_PYTHON) not in sys.path:
    sys.path.insert(0, str(HOST_PYTHON))

import rangeweave_extrinsics as extrinsics
import rangeweave_geometry as geometry
import rangeweave_live_view as live
import rangeweave_plane_alignment as alignment


def normalise(vector):
    length = math.sqrt(sum(value * value for value in vector))
    return tuple(value / length for value in vector)


def plane_normal(rx_deg, ry_deg):
    rx = math.radians(rx_deg)
    ry = math.radians(ry_deg)
    return (
        math.sin(ry) * math.cos(rx),
        -math.sin(rx),
        math.cos(ry) * math.cos(rx),
    )


def synthetic_plane_distances(rx_deg, ry_deg, offset_mm):
    nx, ny, nz = plane_normal(rx_deg, ry_deg)
    distances = []
    for zone_id in range(geometry.ZONE_COUNT):
        vector = geometry.projection_vector(zone_id)
        denominator = nx * vector.x_per_z + ny * vector.y_per_z + nz
        distances.append(offset_mm / denominator)
    return tuple(distances)


class LivePlaneAlignmentTests(unittest.TestCase):
    def test_fit_recovers_known_plane_pose_with_nominal_geometry(self):
        distances = synthetic_plane_distances(8.25, -5.5, 575.0)
        fit = alignment.fit_plane_from_distances(distances)

        self.assertAlmostEqual(fit.rotation_x_deg, 8.25, places=8)
        self.assertAlmostEqual(fit.rotation_y_deg, -5.5, places=8)
        self.assertAlmostEqual(fit.offset_mm, 575.0, places=8)
        self.assertAlmostEqual(fit.rms_residual_mm, 0.0, places=8)
        self.assertEqual(fit.valid_zones, 64)

    def test_alignment_panel_keeps_plane_frame_explicit(self):
        fit = alignment.fit_plane_from_distances(
            synthetic_plane_distances(2.0, 3.0, 550.0)
        )
        text = live.alignment_panel_text(fit)
        self.assertIn("relative to tof_optical", text)
        self.assertIn("geometry: nominal-fallback", text)
        self.assertIn("device_body", text)

    def test_too_few_valid_zones_is_rejected(self):
        distances = (500.0,) * 5 + (0.0,) * 59
        with self.assertRaises(alignment.PlaneAlignmentError):
            alignment.fit_plane_from_distances(distances)


class TofBodyExtrinsicTests(unittest.TestCase):
    def test_nominal_fallback_is_identity(self):
        vector = normalise((0.2, -0.1, 1.0))
        mapped = extrinsics.NOMINAL_TOF_BODY_ROTATION.tof_vector_to_body(vector)
        for actual, expected in zip(mapped, vector):
            self.assertAlmostEqual(actual, expected)

    def test_json_round_trip_preserves_rotation(self):
        original = extrinsics.TofBodyRotation(
            name="test",
            role="calibrated",
            rotation_body_from_tof=extrinsics.rotation_xyz_deg(1.0, 4.0, -2.0),
            metadata={"example": True},
        )
        restored = extrinsics.TofBodyRotation.from_dict(original.to_dict())
        self.assertEqual(restored.name, "test")
        self.assertEqual(restored.role, "calibrated")
        for row_actual, row_expected in zip(
            restored.rotation_body_from_tof, original.rotation_body_from_tof
        ):
            for actual, expected in zip(row_actual, row_expected):
                self.assertAlmostEqual(actual, expected)

    def test_fixed_plane_and_relative_body_rotations_recover_boresight(self):
        true_rx = 1.7
        true_ry = 3.6
        true_rz = -0.8
        rotation_body_from_tof = extrinsics.rotation_xyz_deg(
            true_rx, true_ry, true_rz
        )
        reference_plane_normal = normalise((0.12, -0.08, 1.0))

        reference_from_body_rotations = (
            extrinsics.rotation_xyz_deg(0.0, 0.0, 0.0),
            extrinsics.rotation_xyz_deg(15.0, 0.0, 0.0),
            extrinsics.rotation_xyz_deg(-12.0, 0.0, 0.0),
            extrinsics.rotation_xyz_deg(0.0, 14.0, 0.0),
            extrinsics.rotation_xyz_deg(0.0, -13.0, 0.0),
            extrinsics.rotation_xyz_deg(8.0, 10.0, 4.0),
            extrinsics.rotation_xyz_deg(-7.0, -9.0, -5.0),
        )

        observations = []
        for reference_from_body in reference_from_body_rotations:
            normal_body = extrinsics.matrix_vector(
                extrinsics.transpose(reference_from_body),
                reference_plane_normal,
            )
            normal_tof = extrinsics.matrix_vector(
                extrinsics.transpose(rotation_body_from_tof),
                normal_body,
            )
            observations.append(
                extrinsics.FixedPlaneBoresightObservation(
                    tof_plane_normal=normal_tof,
                    reference_from_body=reference_from_body,
                )
            )

        fit = extrinsics.solve_fixed_plane_boresight(observations)

        self.assertAlmostEqual(fit.rotation_x_deg, true_rx, delta=0.02)
        self.assertAlmostEqual(fit.rotation_y_deg, true_ry, delta=0.02)
        self.assertAlmostEqual(fit.rotation_z_deg, true_rz, delta=0.02)
        self.assertLess(fit.rms_normal_error_deg, 1.0e-5)

        dot = sum(
            fit.reference_plane_normal[index] * reference_plane_normal[index]
            for index in range(3)
        )
        self.assertGreater(dot, 0.999999)

    def test_no_relative_body_motion_is_rejected_as_underconstrained(self):
        normal = normalise((0.1, 0.05, 1.0))
        observations = [
            extrinsics.FixedPlaneBoresightObservation(
                tof_plane_normal=normal,
                reference_from_body=extrinsics.identity_matrix(),
            )
            for _ in range(4)
        ]
        with self.assertRaises(extrinsics.ExtrinsicError):
            extrinsics.solve_fixed_plane_boresight(observations)


if __name__ == "__main__":
    unittest.main()
