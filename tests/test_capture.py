"""Capture/replay parity tests for exact Rangeweave wire bytes."""

from pathlib import Path
import struct
import sys
import tempfile
import unittest

HERE = Path(__file__).resolve().parent
REPO_ROOT = HERE.parent
HOST_PYTHON = REPO_ROOT / "host" / "python"
if str(HOST_PYTHON) not in sys.path:
    sys.path.insert(0, str(HOST_PYTHON))

import rangeweave_capture as cap
import rangeweave_protocol as rw


def _tlv(tag, value):
    return bytes([tag, len(value)]) + value


def _sample_stream():
    stream_info = (
        struct.pack("<QH", 0x0123456789ABCDEF, 0)
        + _tlv(rw.INFO_FIRMWARE_LABEL, b"capture-test")
        + _tlv(rw.INFO_SOURCE_PROFILE, b"synthetic")
        + _tlv(rw.INFO_TOF_GRID_CONFIG, bytes([8, 8, 15]))
    )
    status_a = struct.pack("<QIIIIIIIIIHH", 1_000_000, 10, 0, 0, 0, 0, 0, 0, 0, 0, 2, 3)
    clock = struct.pack("<QQQ", 1_100_000, 42_000, 1_100_250)
    status_b = struct.pack("<QIIIIIIIIIHH", 2_000_000, 13, 0, 0, 0, 0, 0, 0, 0, 0, 3, 3)
    return b"".join((
        rw.encode_frame(rw.RECORD_STREAM_INFO, 100, stream_info),
        rw.encode_frame(rw.RECORD_STATUS, 101, status_a),
        rw.encode_frame(rw.RECORD_CLOCK_SYNC, 102, clock),
        rw.encode_frame(rw.RECORD_STATUS, 103, status_b),
    ))


class MemorySource:
    def __init__(self, data, chunk_size):
        self.data = data
        self.chunk_size = chunk_size
        self.offset = 0

    def read(self, size):
        if self.offset >= len(self.data):
            return b""
        take = min(size, self.chunk_size, len(self.data) - self.offset)
        chunk = self.data[self.offset:self.offset + take]
        self.offset += take
        return chunk


class CaptureReplayTests(unittest.TestCase):
    def test_chunking_does_not_change_decoded_summary(self):
        wire = _sample_stream()
        baseline = cap.inspect_source(MemorySource(wire, len(wire)), chunk_size=4096)
        awkward = cap.inspect_source(MemorySource(wire, 7), chunk_size=4096)

        decoder_a, stats_a, bytes_a, hash_a = baseline
        decoder_b, stats_b, bytes_b, hash_b = awkward
        self.assertEqual(bytes_a, bytes_b)
        self.assertEqual(hash_a, hash_b)
        self.assertEqual(decoder_a.frames_ok, decoder_b.frames_ok)
        self.assertEqual(decoder_a.frames_bad, decoder_b.frames_bad)
        self.assertEqual(stats_a.to_dict(decoder_a), stats_b.to_dict(decoder_b))
        self.assertEqual(stats_b.sequence_gaps, 0)
        self.assertEqual(stats_b.record_counts["STATUS"], 2)
        self.assertEqual(stats_b.last_info.session_id, 0x0123456789ABCDEF)

    def test_sequence_gap_is_preserved_as_stream_health(self):
        payload = struct.pack("<QQQ", 1, 2, 3)
        wire = (
            rw.encode_frame(rw.RECORD_CLOCK_SYNC, 8, payload)
            + rw.encode_frame(rw.RECORD_CLOCK_SYNC, 10, payload)
        )
        decoder, stats, _, _ = cap.inspect_source(MemorySource(wire, 5))
        self.assertEqual(decoder.frames_ok, 2)
        self.assertEqual(stats.sequence_gaps, 1)
        self.assertGreater(cap.stream_issue_count(decoder, stats), 0)

    def test_metadata_replay_parity_and_hash_detection(self):
        wire = _sample_stream()
        with tempfile.TemporaryDirectory() as temp:
            session = Path(temp) / "capture_test"
            session.mkdir()
            packets = session / cap.PACKETS_FILENAME
            packets.write_bytes(wire)

            decoder, stats, byte_count, sha256 = cap.inspect_file(packets, chunk_size=9)
            metadata = cap.build_metadata(
                status="complete",
                started_at_utc="2026-08-20T12:00:00.000Z",
                ended_at_utc="2026-08-20T12:00:01.000Z",
                requested_duration_seconds=1.0,
                recorded_duration_seconds=1.0,
                source={"kind": "test"},
                packets_bytes=byte_count,
                packets_sha256=sha256,
                decoder=decoder,
                stats=stats,
            )
            cap.write_json_atomic(session / cap.METADATA_FILENAME, metadata)
            loaded = cap.load_metadata(session)

            replay_decoder, replay_stats, replay_bytes, replay_hash = cap.inspect_file(packets, chunk_size=3)
            self.assertEqual(
                cap.metadata_parity_errors(
                    loaded,
                    decoder=replay_decoder,
                    stats=replay_stats,
                    packets_bytes=replay_bytes,
                    packets_sha256=replay_hash,
                ),
                [],
            )

            loaded["packets"]["sha256"] = "00" * 32
            errors = cap.metadata_parity_errors(
                loaded,
                decoder=replay_decoder,
                stats=replay_stats,
                packets_bytes=replay_bytes,
                packets_sha256=replay_hash,
            )
            self.assertIn("packets SHA-256 differs from metadata", errors)

    def test_stream_info_metadata_keeps_raw_tlvs_and_known_fields(self):
        wire = _sample_stream()
        decoder, stats, _, _ = cap.inspect_source(MemorySource(wire, 11))
        data = stats.to_dict(decoder)
        info = data["stream"]["stream_info"]
        self.assertEqual(info["session_id"], "0x0123456789ABCDEF")
        self.assertEqual(info["known"]["firmware_label"], "capture-test")
        self.assertEqual(info["known"]["tof_grid"], {"rows": 8, "cols": 8, "hz": 15})
        self.assertEqual(len(info["tlvs"]), 3)


if __name__ == "__main__":
    unittest.main()
