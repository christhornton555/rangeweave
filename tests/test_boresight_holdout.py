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

import inspect_boresight_holdout as inspect_holdout
import rangeweave_boresight_holdout as holdout
import rangeweave_extrinsics as ext


class BoresightHoldoutTests(unittest.TestCase):
    def assertVectorAlmostEqual(self, left, right, places=9):
        self.assertEqual(len(left), len(right))
        for a, b in zip(left, right):
            self.assertAlmostEqual(a, b, places=places)

    def test_predict_holdout_normal_inverts_boresight_model(self):
        rotation_body_from_tof = ext.rotation_xyz_deg(2.0, -3.0, 1.5)
        reference_from_body = ext.rotation_xyz_deg(12.0, 17.0, -8.0)
        observed_tof = (0.11, -0.23, 0.967)
        length = math.sqrt(sum(value * value for value in observed_tof))
        observed_tof = tuple(value / length for value in observed_tof)
        reference_normal = ext.matrix_vector(
            reference_from_body,
            ext.matrix_vector(rotation_body_from_tof, observed_tof),
        )

        predicted = holdout.predict_holdout_tof_normal(
            reference_plane_normal=reference_normal,
            rotation_body_from_tof=rotation_body_from_tof,
            reference_from_body=reference_from_body,
        )
        self.assertVectorAlmostEqual(predicted, observed_tof)
        self.assertAlmostEqual(holdout.normal_angle_deg(predicted, observed_tof), 0.0, places=7)

    def test_rotation_difference_reports_known_separation(self):
        identity = ext.identity_matrix()
        rotated = ext.rotation_x_deg(5.0)
        self.assertAlmostEqual(
            holdout.rotation_difference_deg(rotated, identity),
            5.0,
            places=9,
        )

    def test_synthetic_holdout_recovers_unseen_tof_normal(self):
        true_extrinsic = ext.rotation_xyz_deg(2.3, -1.7, 1.1)
        reference_normal = (0.08, -0.12, 0.9895)
        norm = math.sqrt(sum(value * value for value in reference_normal))
        reference_normal = tuple(value / norm for value in reference_normal)
        body_poses = (
            ext.identity_matrix(),
            ext.rotation_xyz_deg(16.0, 0.0, 1.0),
            ext.rotation_xyz_deg(5.0, 23.0, -4.0),
            ext.rotation_xyz_deg(-7.0, -19.0, 8.0),
            ext.rotation_xyz_deg(11.0, 14.0, 17.0),
        )

        observations = []
        for reference_from_body in body_poses:
            tof_normal = ext.matrix_vector(
                ext.transpose(true_extrinsic),
                ext.matrix_vector(ext.transpose(reference_from_body), reference_normal),
            )
            observations.append(
                ext.FixedPlaneBoresightObservation(
                    tof_plane_normal=tof_normal,
                    reference_from_body=reference_from_body,
                )
            )

        result = holdout.evaluate_holdout(observations[:4], observations[4])
        self.assertLess(result.holdout_normal_error_deg, 0.05)
        # The dependency-free coordinate search terminates on a 0.01 deg grid;
        # coupled Euler parameters can therefore shift by a few hundredths of a
        # degree between equivalent four- and five-pose synthetic fits.
        self.assertLess(result.fit_rotation_change_deg, 0.10)
        self.assertLess(result.training_fit.rms_normal_error_deg, 0.05)

    def test_session_capture_resolution_requires_unique_stage(self):
        session = "boresight-guided-20260904_010944"
        with tempfile.TemporaryDirectory() as temp:
            root = Path(temp)
            expected = root / f"capture_20260904_010952Z_{session}-p0"
            expected.mkdir()
            self.assertEqual(inspect_holdout._capture_for(root, session, "p0"), expected)

            duplicate = root / f"capture_20260904_020000Z_{session}-p0"
            duplicate.mkdir()
            with self.assertRaises(ValueError):
                inspect_holdout._capture_for(root, session, "p0")


if __name__ == "__main__":
    unittest.main()
