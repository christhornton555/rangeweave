"""Tests for diagnostic LIS3MDL body-axis mapping ranking."""

from pathlib import Path
import sys
import unittest

HERE = Path(__file__).resolve().parent
REPO_ROOT = HERE.parent
HOST_PYTHON = REPO_ROOT / "host" / "python"
if str(HOST_PYTHON) not in sys.path:
    sys.path.insert(0, str(HOST_PYTHON))

import rangeweave_extrinsics as ext
import rangeweave_magnetometer_mapping as mapping


class MagnetometerMappingTests(unittest.TestCase):
    def test_generates_24_proper_signed_permutations(self):
        candidates = mapping.proper_signed_permutation_mappings()
        self.assertEqual(len(candidates), 24)
        self.assertEqual(len({candidate.name for candidate in candidates}), 24)
        for candidate in candidates:
            matrix = candidate.rotation_body_from_mag
            product = ext.matrix_multiply(matrix, ext.transpose(matrix))
            for row in range(3):
                for col in range(3):
                    self.assertAlmostEqual(
                        product[row][col],
                        1.0 if row == col else 0.0,
                        places=12,
                    )

    def test_correct_mapping_stabilises_synthetic_reference_field(self):
        true_matrix = (
            (0.0, -1.0, 0.0),
            (0.0, 0.0, -1.0),
            (1.0, 0.0, 0.0),
        )
        candidates = mapping.proper_signed_permutation_mappings()
        true_mapping = next(
            candidate
            for candidate in candidates
            if candidate.rotation_body_from_mag == true_matrix
        )

        field_reference = (24.0, -11.0, 43.0)
        attitudes = (
            ext.identity_matrix(),
            ext.rotation_x_deg(22.0),
            ext.rotation_y_deg(-31.0),
            ext.rotation_z_deg(47.0),
            ext.rotation_xyz_deg(18.0, -27.0, 35.0),
            ext.rotation_xyz_deg(-33.0, 21.0, -16.0),
        )

        sensor_vectors = []
        body_from_mag_t = ext.transpose(true_matrix)
        for reference_from_body in attitudes:
            field_body = ext.matrix_vector(
                ext.transpose(reference_from_body), field_reference
            )
            sensor_vectors.append(ext.matrix_vector(body_from_mag_t, field_body))

        scores = tuple(
            mapping.score_aligned_vectors(sensor_vectors, attitudes, candidate)
            for candidate in candidates
        )
        ranked = sorted(scores, key=lambda item: item.direction_rms_deg)
        self.assertEqual(ranked[0].mapping.name, true_mapping.name)
        self.assertLess(ranked[0].direction_rms_deg, 1.0e-6)
        self.assertGreater(ranked[1].direction_rms_deg, 1.0)

    def test_mismatched_lengths_are_rejected(self):
        candidate = mapping.proper_signed_permutation_mappings()[0]
        with self.assertRaises(mapping.MagnetometerMappingError):
            mapping.score_aligned_vectors(
                [(1.0, 2.0, 3.0)],
                [ext.identity_matrix(), ext.identity_matrix()],
                candidate,
            )


if __name__ == "__main__":
    unittest.main()
