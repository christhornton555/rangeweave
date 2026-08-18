"""Conformance tests for the dependency-light Rangeweave protocol reference."""

from dataclasses import asdict, is_dataclass
import json
from pathlib import Path
import sys
import unittest

HERE = Path(__file__).resolve().parent
REPO_ROOT = HERE.parent
HOST_PYTHON = REPO_ROOT / "host" / "python"
if str(HOST_PYTHON) not in sys.path:
    sys.path.insert(0, str(HOST_PYTHON))

import rangeweave_protocol as rw


def _jsonable(value):
    if is_dataclass(value):
        return {key: _jsonable(item) for key, item in asdict(value).items()}
    if isinstance(value, tuple):
        return [_jsonable(item) for item in value]
    if isinstance(value, list):
        return [_jsonable(item) for item in value]
    return value


class ProtocolVectorTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        path = REPO_ROOT / "protocol" / "test-vectors" / "v0.1.json"
        cls.fixture = json.loads(path.read_text(encoding="utf-8"))

    def test_golden_vectors(self):
        for vector in self.fixture["vectors"]:
            with self.subTest(vector=vector["name"]):
                wire = bytes.fromhex(vector["wire_hex"])
                self.assertEqual(wire[-1], 0)

                decoded = rw.cobs_decode(wire[:-1])
                self.assertEqual(decoded.hex(), vector["decoded_frame_hex"])

                frame = rw.decode_wire_frame(wire[:-1])
                self.assertEqual(frame.protocol_major, 0)
                self.assertEqual(frame.protocol_minor, 1)
                self.assertEqual(frame.record_type, vector["record_type"])
                self.assertEqual(frame.sequence, vector["sequence"])

                record = rw.decode_record(frame)
                self.assertEqual(_jsonable(record), vector["expected_record"])

    def test_corrupt_frame_resynchronizes(self):
        case = self.fixture["stream_cases"][0]
        decoder = rw.StreamDecoder()

        # Feed deliberately awkward chunks to ensure stream parsing is incremental.
        wire = bytes.fromhex(case["wire_hex"])
        output = []
        for start in range(0, len(wire), 7):
            output.extend(decoder.feed(wire[start:start + 7]))

        self.assertEqual(
            [frame.sequence for frame in output],
            case["expected_valid_sequences"],
        )
        self.assertEqual(decoder.frames_bad, case["expected_bad_frames"])

    def test_cobs_round_trip_with_zeroes(self):
        raw = bytes([0, 1, 2, 0, 0, 3, 255, 0, 4])
        self.assertEqual(rw.cobs_decode(rw.cobs_encode(raw)), raw)

    def test_crc_known_check_value(self):
        # Canonical check value for CRC-16/CCITT-FALSE.
        self.assertEqual(rw.crc16_ccitt_false(b"123456789"), 0x29B1)


if __name__ == "__main__":
    unittest.main()
