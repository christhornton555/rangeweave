"""Tests for Rangeweave ToF timing calibration/profile resolution."""

from pathlib import Path
import sys
import unittest

HERE = Path(__file__).resolve().parent
REPO_ROOT = HERE.parent
HOST_PYTHON = REPO_ROOT / "host" / "python"
if str(HOST_PYTHON) not in sys.path:
    sys.path.insert(0, str(HOST_PYTHON))

import rangeweave_protocol as rw
import rangeweave_tof_timing as timing


def _info(*, firmware="rangeweave-pico2w-acq-0.1", source_profile=None):
    if source_profile is None:
        source_profile = "pico2w-lsm6dsox-lis3mdl-vl53l5cx-8x8-15hz"
    return rw.StreamInfo(
        session_id=0x1234,
        info_revision=0,
        tlvs=(
            rw.InfoTlv(rw.INFO_FIRMWARE_LABEL, firmware.encode("utf-8")),
            rw.InfoTlv(rw.INFO_SOURCE_PROFILE, source_profile.encode("utf-8")),
            rw.InfoTlv(rw.INFO_TOF_I2C_ADDRESS, bytes((0x29,))),
            rw.InfoTlv(rw.INFO_TOF_GRID_CONFIG, bytes((8, 8, 15))),
            rw.InfoTlv(rw.INFO_TOF_DEFAULT_FIELD_MASK, (3).to_bytes(2, "little")),
        ),
    )


def _calibrated_artifact():
    return timing.TofTimingArtifact(
        name="unit-a-timing",
        role=timing.ROLE_CALIBRATED,
        effective_offset_ms=-24.0,
        applies_to={
            "assembly_id": "unit-a",
            "protocol": {"major": 0, "minor": 1},
            "stream_info": {
                "firmware_label": "rangeweave-pico2w-acq-0.1",
                "source_profile": "pico2w-lsm6dsox-lis3mdl-vl53l5cx-8x8-15hz",
                "tof_grid": {"rows": 8, "cols": 8, "hz": 15},
            },
        },
        evidence={"method": "test"},
    )


class TofTimingTests(unittest.TestCase):
    def test_quick_start_matches_known_reference_profile(self):
        result = timing.resolve_tof_timing(
            _info(),
            ((0, 1),),
            mode=timing.MODE_QUICK_START,
        )
        self.assertEqual(result.role, timing.ROLE_NOMINAL)
        self.assertEqual(result.effective_offset_ms, 0.0)
        self.assertEqual(result.artifact_name, timing.REFERENCE_QUICK_START_PROFILE.name)

    def test_quick_start_unknown_configuration_is_visible_uncalibrated_zero(self):
        result = timing.resolve_tof_timing(
            _info(firmware="different-producer-build"),
            ((0, 1),),
            mode=timing.MODE_QUICK_START,
        )
        self.assertEqual(result.role, timing.ROLE_UNCALIBRATED)
        self.assertEqual(result.effective_offset_ms, 0.0)
        self.assertIsNone(result.artifact_name)

    def test_calibrated_artifact_requires_matching_assembly_and_config(self):
        artifact = _calibrated_artifact()
        result = timing.resolve_tof_timing(
            _info(),
            ((0, 1),),
            mode=timing.MODE_CALIBRATED,
            assembly_id="unit-a",
            artifact=artifact,
        )
        self.assertEqual(result.role, timing.ROLE_CALIBRATED)
        self.assertEqual(result.effective_offset_ms, -24.0)

        with self.assertRaisesRegex(timing.TofTimingError, "assembly_id"):
            timing.resolve_tof_timing(
                _info(),
                ((0, 1),),
                mode=timing.MODE_CALIBRATED,
                assembly_id="unit-b",
                artifact=artifact,
            )

    def test_calibrated_artifact_rejects_material_configuration_change(self):
        with self.assertRaisesRegex(timing.TofTimingError, "source_profile"):
            timing.resolve_tof_timing(
                _info(source_profile="changed-tof-mode"),
                ((0, 1),),
                mode=timing.MODE_CALIBRATED,
                assembly_id="unit-a",
                artifact=_calibrated_artifact(),
            )

    def test_quick_start_refuses_calibrated_artifact(self):
        with self.assertRaisesRegex(timing.TofTimingError, "timing-mode calibrated"):
            timing.resolve_tof_timing(
                _info(),
                ((0, 1),),
                mode=timing.MODE_QUICK_START,
                assembly_id="unit-a",
                artifact=_calibrated_artifact(),
            )

    def test_explicit_override_has_highest_precedence_without_claiming_calibration(self):
        result = timing.resolve_tof_timing(
            _info(firmware="unknown"),
            ((0, 1),),
            mode=timing.MODE_QUICK_START,
            explicit_offset_ms=-17.5,
        )
        self.assertEqual(result.role, timing.ROLE_OVERRIDE)
        self.assertEqual(result.effective_offset_ms, -17.5)
        self.assertIsNone(result.artifact_name)

    def test_artifact_round_trip_preserves_contract(self):
        artifact = _calibrated_artifact()
        restored = timing.TofTimingArtifact.from_dict(artifact.to_dict())
        self.assertEqual(restored.name, artifact.name)
        self.assertEqual(restored.role, artifact.role)
        self.assertEqual(restored.effective_offset_ms, artifact.effective_offset_ms)
        self.assertEqual(restored.applies_to, artifact.applies_to)


if __name__ == "__main__":
    unittest.main()
