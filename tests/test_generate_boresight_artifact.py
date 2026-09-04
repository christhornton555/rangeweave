import json
from pathlib import Path
import sys
import tempfile
import unittest

HERE = Path(__file__).resolve().parent
REPO_ROOT = HERE.parent
HOST_PYTHON = REPO_ROOT / "host" / "python"
if str(HOST_PYTHON) not in sys.path:
    sys.path.insert(0, str(HOST_PYTHON))

import generate_boresight_artifact as generator
import rangeweave_boresight_holdout as holdout
import rangeweave_extrinsics as ext


class GenerateBoresightArtifactTests(unittest.TestCase):
    def test_validation_capture_resolution_supports_continuation_prefix(self):
        session = "boresight-guided-20260904_010944"
        validation = session + "-validation-20260904_014426"
        with tempfile.TemporaryDirectory() as temp:
            root = Path(temp)
            m5 = root / f"capture_20260904_014426Z_{validation}-m5"
            p5 = root / f"capture_20260904_014457Z_{validation}-p5"
            m5.mkdir()
            p5.mkdir()
            self.assertEqual(
                generator._capture_for_optional_validation(root, session, "m5"),
                (m5, validation),
            )
            self.assertEqual(
                generator._capture_for_optional_validation(root, session, "p5"),
                (p5, validation),
            )

    def test_artifact_document_embeds_schema_validation_and_capture_hashes(self):
        rotation = ext.rotation_xyz_deg(5.88, 3.93, 0.16)
        extrinsic = ext.TofBodyRotation(
            name="test-fit",
            role="calibrated",
            rotation_body_from_tof=rotation,
        )
        fit = ext.BoresightFit(
            extrinsic=extrinsic,
            reference_plane_normal=(0.0, 0.0, 1.0),
            rotation_x_deg=5.88,
            rotation_y_deg=3.93,
            rotation_z_deg=0.16,
            rms_normal_error_deg=0.797,
            max_normal_error_deg=1.648,
            observability_cost_increase_at_1deg=(5.169e-6, 4.278e-6, 2.742e-5),
        )
        evaluation = holdout.BoresightHoldoutEvaluation(
            training_fit=fit,
            predicted_holdout_tof_normal=(0.0, 0.0, 1.0),
            observed_holdout_tof_normal=(0.0, 0.0, 1.0),
            holdout_normal_error_deg=0.644,
            all_pose_fit=fit,
            fit_parameter_delta_deg=(0.45, 0.15, 0.46),
            fit_rotation_change_deg=0.639,
        )

        with tempfile.TemporaryDirectory() as temp:
            capture_dir = Path(temp) / "capture_demo"
            capture_dir.mkdir()
            (capture_dir / "metadata.json").write_text(
                json.dumps(
                    {
                        "started_at_utc": "2026-09-04T01:00:00Z",
                        "packets": {"sha256": "abc123", "bytes": 42},
                        "stream": {"stream_info": {"known": {"lsm_ctrl2_g": 68}}},
                    }
                ),
                encoding="utf-8",
            )
            document = generator._artifact_document(
                session="demo-session",
                validation_prefix="demo-session",
                all_pose_fit=fit,
                evaluation=evaluation,
                capture_paths={"p0": capture_dir},
                imu_mapping_role="physically-validated-reference-rig",
                gyro_full_scales_dps=[500.0],
            )

        self.assertEqual(document["schema"], "rangeweave.tof-body-rotation")
        self.assertEqual(document["schema_version"], 1)
        self.assertEqual(document["role"], "calibrated")
        self.assertEqual(
            document["metadata"]["validation"]["held_out_pose"], "P5"
        )
        self.assertEqual(
            document["metadata"]["provenance"]["captures"]["p0"]["packets_sha256"],
            "abc123",
        )
        self.assertEqual(
            document["metadata"]["provenance"]["geometry_profile"]["role"],
            "nominal-fallback",
        )


if __name__ == "__main__":
    unittest.main()
