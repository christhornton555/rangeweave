"""Tests for fail-closed stream health in the executable Phase 3 wall gate."""

from collections import Counter
from pathlib import Path
import sys
from types import SimpleNamespace
import unittest

HERE = Path(__file__).resolve().parent
REPO_ROOT = HERE.parent
HOST_PYTHON = REPO_ROOT / "host" / "python"
if str(HOST_PYTHON) not in sys.path:
    sys.path.insert(0, str(HOST_PYTHON))

import validate_phase3_wall as validate


class _Stats:
    def __init__(
        self,
        *,
        semantic_errors=0,
        sequence_gaps=0,
        status_count=2,
        health=None,
    ):
        self.semantic_errors = semantic_errors
        self.sequence_gaps = sequence_gaps
        self.record_counts = Counter({"STATUS": status_count})
        self._health = (
            {
                "frames_dropped": 0,
                "imu_samples_dropped": 0,
                "fifo_overruns": 0,
                "fifo_structural_errors": 0,
                "mag_errors": 0,
                "tof_errors": 0,
                "clock_sync_errors": 0,
            }
            if health is None
            else health
        )

    def health_deltas(self):
        return dict(self._health)


class ValidatePhase3WallHealthTests(unittest.TestCase):
    def test_clean_stream_passes(self):
        failures = validate._stream_health_failures(
            SimpleNamespace(frames_bad=0),
            _Stats(),
        )
        self.assertEqual(failures, ())

    def test_missing_status_coverage_fails_closed(self):
        failures = validate._stream_health_failures(
            SimpleNamespace(frames_bad=0),
            _Stats(status_count=0, health={}),
        )
        self.assertTrue(any("STATUS" in item for item in failures))
        self.assertTrue(any("health deltas unavailable" in item for item in failures))

    def test_decoder_semantic_and_sequence_faults_are_rejected(self):
        failures = validate._stream_health_failures(
            SimpleNamespace(frames_bad=2),
            _Stats(semantic_errors=1, sequence_gaps=3),
        )
        self.assertTrue(any("decoder bad frames" in item for item in failures))
        self.assertTrue(any("semantic decode errors" in item for item in failures))
        self.assertTrue(any("sequence gaps" in item for item in failures))

    def test_nonzero_health_counter_is_rejected(self):
        failures = validate._stream_health_failures(
            SimpleNamespace(frames_bad=0),
            _Stats(health={"tof_errors": 1}),
        )
        self.assertTrue(any("tof_errors" in item for item in failures))


if __name__ == "__main__":
    unittest.main()
