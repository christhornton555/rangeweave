"""Tests for portable ToF geometry profiles and generic plane calibration."""

from pathlib import Path
import math
import sys
import tempfile
import unittest

HERE = Path(__file__).resolve().parent
REPO_ROOT = HERE.parent
HOST_PYTHON = REPO_ROOT / "host" / "python"
if str(HOST_PYTHON) not in sys.path:
    sys.path.insert(0, str(HOST_PYTHON))

import rangeweave_geometry as geometry
import rangeweave_tof_calibration as calibration


def synthetic_outward_asymmetric_profile():
    slopes = []
    for zone_id in range(geometry.ZONE_COUNT):
        physical_row, physical_col = geometry.physical_row_col(zone_id)
        u = (physical_col - 3.5) / 3.5
        v = (physical_row - 3.5) / 3.5

        # Deliberately not symmetric: cross-terms and offsets are included.
        # The edge magnitude grows toward the middle of the opposite coordinate,
        # giving an outward-bowing lattice rather than the ST fallback shape.
        x_per_z = 0.32 * u * (1.0 + 0.22 * (1.0 - v * v)) + 0.012 * v + 0.004
        y_per_z = 0.31 * v * (1.0 + 0.18 * (1.0 - u * u)) - 0.009 * u - 0.003
        slopes.append((x_per_z, y_per_z))

    return geometry.TofGeometryProfile(
        name="synthetic-outward-asymmetric",
        role="test-truth",
        xy_per_z=tuple(slopes),
        metadata={"purpose": "prove calibration does not assume ST symmetry"},
    )


def plane_normal(x_tilt_degrees, y_tilt_degrees):
    nx = math.sin(math.radians(x_tilt_degrees))
    ny = math.sin(math.radians(y_tilt_degrees))
    nz_squared = 1.0 - nx * nx - ny * ny
    if nz_squared <= 0.0:
        raise ValueError("synthetic plane tilt is too large")
    return nx, ny, math.sqrt(nz_squared)


def synthetic_plane(profile, x_tilt, y_tilt, label, offset_mm=800.0):
    nx, ny, nz = plane_normal(x_tilt, y_tilt)
    distances = tuple(
        calibration.expected_axial_distance_mm(
            profile,
            zone_id,
            normal_x=nx,
            normal_y=ny,
            normal_z=nz,
            offset_mm=offset_mm,
        )
        for zone_id in range(geometry.ZONE_COUNT)
    )
    return calibration.CalibrationPlane(
        normal_x=nx,
        normal_y=ny,
        normal_z=nz,
        offset_mm=offset_mm,
        distances_mm=distances,
        label=label,
    )


class GeometryProfileTests(unittest.TestCase):
    def test_existing_default_projection_still_uses_nominal_fallback(self):
        legacy = geometry.project_axial_distance_mm(0, 1000)
        explicit = geometry.project_axial_distance_mm(
            0,
            1000,
            geometry.NOMINAL_ST_PROFILE,
        )
        self.assertEqual(legacy, explicit)
        self.assertEqual(geometry.GEOMETRY_MODEL, geometry.NOMINAL_ST_PROFILE.name)
        self.assertEqual(
            geometry.GEOMETRY_PROFILE_ROLE,
            geometry.NOMINAL_ST_PROFILE.role,
        )

    def test_custom_profile_controls_projection_without_reordering_zones(self):
        slopes = tuple((0.1 + zone * 0.001, -0.2 - zone * 0.002) for zone in range(64))
        profile = geometry.TofGeometryProfile(
            name="custom",
            role="calibrated",
            xy_per_z=slopes,
        )
        point = geometry.project_axial_distance_mm(10, 500, profile)
        self.assertAlmostEqual(point.x_mm, slopes[10][0] * 500)
        self.assertAlmostEqual(point.y_mm, slopes[10][1] * 500)
        self.assertEqual(point.z_mm, 500.0)

    def test_profile_json_round_trip_preserves_all_64_independent_slopes(self):
        original = synthetic_outward_asymmetric_profile()
        with tempfile.TemporaryDirectory() as temp_dir:
            path = Path(temp_dir) / "tof_geometry.json"
            geometry.save_geometry_profile(original, path)
            loaded = geometry.load_geometry_profile(path)

        self.assertEqual(loaded.name, original.name)
        self.assertEqual(loaded.role, original.role)
        self.assertEqual(loaded.xy_per_z, original.xy_per_z)
        self.assertEqual(loaded.metadata, original.metadata)

    def test_profile_rejects_wrong_zone_count(self):
        with self.assertRaises(geometry.GeometryError):
            geometry.TofGeometryProfile(
                name="bad",
                role="calibrated",
                xy_per_z=((0.0, 0.0),) * 63,
            )


class PlaneCalibrationTests(unittest.TestCase):
    def calibration_planes(self, profile):
        return [
            synthetic_plane(profile, +15, 0, "x+15"),
            synthetic_plane(profile, -15, 0, "x-15"),
            synthetic_plane(profile, 0, +15, "y+15"),
            synthetic_plane(profile, 0, -15, "y-15"),
            synthetic_plane(profile, +12, +10, "xy-mixed"),
        ]

    def test_synthetic_truth_is_outward_bowing_and_asymmetric(self):
        profile = synthetic_outward_asymmetric_profile()
        # Physical top edge: producer 60 is near top-centre, 63 is top-left.
        self.assertGreater(
            abs(profile.xy_per_z[60][1]),
            abs(profile.xy_per_z[63][1]),
        )
        # Deliberate offsets/cross-terms prevent mirror symmetry.
        self.assertNotAlmostEqual(
            profile.xy_per_z[0][0],
            -profile.xy_per_z[7][0],
        )

    def test_recovers_outward_bowing_asymmetric_profile_without_symmetry_assumptions(self):
        truth = synthetic_outward_asymmetric_profile()
        result = calibration.calibrate_geometry_profile(
            self.calibration_planes(truth),
            name="synthetic-recovered",
        )

        self.assertEqual(result.profile.role, "calibrated")
        self.assertEqual(
            result.profile.metadata["calibration"]["method"],
            calibration.CALIBRATION_METHOD,
        )
        for expected, actual in zip(truth.xy_per_z, result.profile.xy_per_z):
            self.assertAlmostEqual(actual[0], expected[0], places=12)
            self.assertAlmostEqual(actual[1], expected[1], places=12)

        self.assertLess(
            result.profile.metadata["calibration"]["max_abs_plane_residual_mm"],
            1.0e-9,
        )

    def test_recovered_profile_predicts_held_out_plane(self):
        truth = synthetic_outward_asymmetric_profile()
        result = calibration.calibrate_geometry_profile(
            self.calibration_planes(truth),
            name="synthetic-recovered",
        )
        nx, ny, nz = plane_normal(+9, -11)
        for zone_id in range(geometry.ZONE_COUNT):
            expected = calibration.expected_axial_distance_mm(
                truth,
                zone_id,
                normal_x=nx,
                normal_y=ny,
                normal_z=nz,
                offset_mm=850.0,
            )
            actual = calibration.expected_axial_distance_mm(
                result.profile,
                zone_id,
                normal_x=nx,
                normal_y=ny,
                normal_z=nz,
                offset_mm=850.0,
            )
            self.assertAlmostEqual(actual, expected, places=9)

    def test_invalid_measurement_can_be_skipped_when_other_planes_constrain_zone(self):
        truth = synthetic_outward_asymmetric_profile()
        planes = self.calibration_planes(truth)
        first = planes[0]
        distances = list(first.distances_mm)
        distances[17] = None
        planes[0] = calibration.CalibrationPlane(
            normal_x=first.normal_x,
            normal_y=first.normal_y,
            normal_z=first.normal_z,
            offset_mm=first.offset_mm,
            distances_mm=tuple(distances),
            label=first.label,
        )

        fit = calibration.solve_zone_slopes(17, planes)
        self.assertEqual(fit.observations, 4)
        self.assertAlmostEqual(fit.x_per_z, truth.xy_per_z[17][0], places=12)
        self.assertAlmostEqual(fit.y_per_z, truth.xy_per_z[17][1], places=12)

    def test_rank_deficient_plane_set_is_rejected_instead_of_inventing_y(self):
        truth = synthetic_outward_asymmetric_profile()
        planes = [
            synthetic_plane(truth, +15, 0, "x+15"),
            synthetic_plane(truth, -15, 0, "x-15"),
            synthetic_plane(truth, +25, 0, "x+25"),
        ]
        with self.assertRaises(calibration.CalibrationError):
            calibration.solve_zone_slopes(0, planes)


if __name__ == "__main__":
    unittest.main()
