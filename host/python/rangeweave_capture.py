"""Canonical capture/replay helpers for Rangeweave protocol v0.1 streams.

This module deliberately sits above ``rangeweave_protocol`` and below visualization or
mapping. Live serial capture and file replay both feed exact wire bytes through the same
``StreamDecoder`` and ``StreamStats`` path.
"""

from __future__ import annotations

from collections import Counter
from dataclasses import asdict
from datetime import datetime, timezone
import hashlib
import json
from pathlib import Path
from typing import BinaryIO, Optional, Protocol

import rangeweave_protocol as rw

CAPTURE_FORMAT = "rangeweave-capture"
CAPTURE_FORMAT_VERSION = 1
PACKETS_FILENAME = "packets.bin"
METADATA_FILENAME = "metadata.json"
NOTES_FILENAME = "notes.txt"

_HEALTH_COUNTERS = (
    "frames_dropped",
    "imu_samples_dropped",
    "fifo_overruns",
    "fifo_structural_errors",
    "mag_errors",
    "tof_errors",
    "clock_sync_errors",
)


class ByteSource(Protocol):
    """Minimal byte-source contract shared conceptually by live and replay inputs."""

    def read(self, size: int) -> bytes:
        ...


class FileByteSource:
    """Binary file source implementing the same ``read(size)`` boundary as serial."""

    def __init__(self, path: Path | str):
        self.path = Path(path)
        self._file: Optional[BinaryIO] = None

    def __enter__(self) -> "FileByteSource":
        self._file = self.path.open("rb")
        return self

    def __exit__(self, exc_type, exc, tb) -> None:
        self.close()

    def read(self, size: int) -> bytes:
        if self._file is None:
            raise RuntimeError("FileByteSource is not open")
        return self._file.read(size)

    def close(self) -> None:
        if self._file is not None:
            self._file.close()
            self._file = None


def utc_now_iso() -> str:
    return datetime.now(timezone.utc).isoformat(timespec="milliseconds").replace("+00:00", "Z")


def record_name(record_type: int) -> str:
    return {
        rw.RECORD_IMU_BATCH: "IMU_BATCH",
        rw.RECORD_MAG: "MAG",
        rw.RECORD_TOF_GRID: "TOF_GRID",
        rw.RECORD_CLOCK_SYNC: "CLOCK_SYNC",
        rw.RECORD_STATUS: "STATUS",
        rw.RECORD_STREAM_INFO: "STREAM_INFO",
    }.get(record_type, "0x{:02X}".format(record_type))


def counter_delta(new_value: int, old_value: int) -> int:
    return (new_value - old_value) & 0xFFFFFFFF


def _decode_text(value: Optional[bytes]) -> Optional[str]:
    if value is None:
        return None
    return value.decode("utf-8", "replace")


def _stream_info_dict(info: Optional[rw.StreamInfo]) -> Optional[dict]:
    if info is None:
        return None

    raw_tlvs = [
        {
            "tag": item.tag,
            "tag_hex": "0x{:02X}".format(item.tag),
            "value_hex": item.value.hex(),
        }
        for item in info.tlvs
    ]

    known: dict[str, object] = {}
    firmware = _decode_text(info.first_value(rw.INFO_FIRMWARE_LABEL))
    source_profile = _decode_text(info.first_value(rw.INFO_SOURCE_PROFILE))
    if firmware is not None:
        known["firmware_label"] = firmware
    if source_profile is not None:
        known["source_profile"] = source_profile

    for tag, key in (
        (rw.INFO_LSM_WHOAMI, "lsm_whoami"),
        (rw.INFO_MAG_WHOAMI, "mag_whoami"),
        (rw.INFO_TOF_I2C_ADDRESS, "tof_i2c_address"),
        (rw.INFO_LSM_CTRL1_XL, "lsm_ctrl1_xl"),
        (rw.INFO_LSM_CTRL2_G, "lsm_ctrl2_g"),
        (rw.INFO_LSM_FIFO_CTRL3, "lsm_fifo_ctrl3"),
        (rw.INFO_LSM_FIFO_CTRL4, "lsm_fifo_ctrl4"),
    ):
        value = info.first_value(tag)
        if value and len(value) == 1:
            known[key] = value[0]

    freq_fine = info.first_value(rw.INFO_LSM_FREQ_FINE)
    if freq_fine and len(freq_fine) == 1:
        known["lsm_freq_fine"] = int.from_bytes(freq_fine, "little", signed=True)

    mag_regs = info.first_value(rw.INFO_MAG_CTRL_REGS_1_TO_5)
    if mag_regs is not None:
        known["mag_ctrl_regs_1_to_5_hex"] = mag_regs.hex()

    tof_config = info.first_value(rw.INFO_TOF_GRID_CONFIG)
    if tof_config and len(tof_config) >= 3:
        known["tof_grid"] = {
            "rows": tof_config[0],
            "cols": tof_config[1],
            "hz": tof_config[2],
        }

    field_mask = info.first_value(rw.INFO_TOF_DEFAULT_FIELD_MASK)
    if field_mask and len(field_mask) == 2:
        known["tof_default_field_mask"] = int.from_bytes(field_mask, "little")

    return {
        "session_id": "0x{:016X}".format(info.session_id),
        "info_revision": info.info_revision,
        "known": known,
        "tlvs": raw_tlvs,
    }


class StreamStats:
    """Accumulate stream-level and semantic health while frames are decoded."""

    def __init__(self) -> None:
        self.record_counts: Counter[str] = Counter()
        self.protocol_versions: set[tuple[int, int]] = set()
        self.first_sequence: Optional[int] = None
        self.last_sequence: Optional[int] = None
        self.sequence_gaps = 0
        self.semantic_errors = 0
        self.first_status: Optional[rw.Status] = None
        self.last_status: Optional[rw.Status] = None
        self.last_info: Optional[rw.StreamInfo] = None

    def consume(self, frame: rw.Frame) -> None:
        self.record_counts[record_name(frame.record_type)] += 1
        self.protocol_versions.add((frame.protocol_major, frame.protocol_minor))

        if self.first_sequence is None:
            self.first_sequence = frame.sequence
        if self.last_sequence is not None:
            expected = (self.last_sequence + 1) & 0xFFFFFFFF
            if frame.sequence != expected:
                self.sequence_gaps += (frame.sequence - expected) & 0xFFFFFFFF
        self.last_sequence = frame.sequence

        try:
            record = rw.decode_record(frame)
        except rw.ProtocolError:
            self.semantic_errors += 1
            return

        if frame.record_type == rw.RECORD_STATUS:
            if self.first_status is None:
                self.first_status = record
            self.last_status = record
        elif frame.record_type == rw.RECORD_STREAM_INFO:
            self.last_info = record

    def health_deltas(self) -> dict[str, int]:
        if self.first_status is None or self.last_status is None:
            return {}
        return {
            key: counter_delta(
                getattr(self.last_status, key),
                getattr(self.first_status, key),
            )
            for key in _HEALTH_COUNTERS
        }

    def to_dict(self, decoder: rw.StreamDecoder) -> dict:
        return {
            "decoder": {
                "frames_ok": decoder.frames_ok,
                "frames_bad": decoder.frames_bad,
                "empty_delimiters": decoder.empty_delimiters,
                "semantic_errors": self.semantic_errors,
            },
            "stream": {
                "protocol_versions": [
                    {"major": major, "minor": minor}
                    for major, minor in sorted(self.protocol_versions)
                ],
                "first_sequence": self.first_sequence,
                "last_sequence": self.last_sequence,
                "sequence_gaps": self.sequence_gaps,
                "record_counts": dict(sorted(self.record_counts.items())),
                "stream_info": _stream_info_dict(self.last_info),
            },
            "health": {
                "first_status": asdict(self.first_status) if self.first_status else None,
                "last_status": asdict(self.last_status) if self.last_status else None,
                "deltas": self.health_deltas(),
            },
        }


def feed_chunk(decoder: rw.StreamDecoder, stats: StreamStats, chunk: bytes) -> None:
    for frame in decoder.feed(chunk):
        stats.consume(frame)


def inspect_source(source: ByteSource, chunk_size: int = 4096) -> tuple[rw.StreamDecoder, StreamStats, int, str]:
    """Read a source to EOF and return decoder/stats plus byte count and SHA-256."""
    decoder = rw.StreamDecoder()
    stats = StreamStats()
    digest = hashlib.sha256()
    byte_count = 0

    while True:
        chunk = source.read(chunk_size)
        if not chunk:
            break
        byte_count += len(chunk)
        digest.update(chunk)
        feed_chunk(decoder, stats, chunk)

    return decoder, stats, byte_count, digest.hexdigest()


def inspect_file(path: Path | str, chunk_size: int = 4096) -> tuple[rw.StreamDecoder, StreamStats, int, str]:
    with FileByteSource(path) as source:
        return inspect_source(source, chunk_size=chunk_size)


def make_session_directory(root: Path | str, label: Optional[str] = None) -> Path:
    root_path = Path(root)
    root_path.mkdir(parents=True, exist_ok=True)
    timestamp = datetime.now(timezone.utc).strftime("%Y%m%d_%H%M%SZ")
    suffix = ""
    if label:
        safe = "".join(ch if ch.isalnum() or ch in "-_." else "-" for ch in label).strip("-.")
        if safe:
            suffix = "_" + safe
    base = "capture_{}{}".format(timestamp, suffix)
    candidate = root_path / base
    serial = 2
    while candidate.exists():
        candidate = root_path / "{}_{}".format(base, serial)
        serial += 1
    candidate.mkdir()
    return candidate


def write_json_atomic(path: Path | str, value: dict) -> None:
    path = Path(path)
    temp = path.with_suffix(path.suffix + ".tmp")
    temp.write_text(json.dumps(value, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    temp.replace(path)


def load_metadata(session_dir: Path | str) -> dict:
    path = Path(session_dir) / METADATA_FILENAME
    return json.loads(path.read_text(encoding="utf-8"))


def build_metadata(
    *,
    status: str,
    started_at_utc: str,
    ended_at_utc: str,
    requested_duration_seconds: Optional[float],
    recorded_duration_seconds: float,
    source: dict,
    packets_bytes: int,
    packets_sha256: str,
    decoder: rw.StreamDecoder,
    stats: StreamStats,
) -> dict:
    metadata = {
        "format": CAPTURE_FORMAT,
        "format_version": CAPTURE_FORMAT_VERSION,
        "status": status,
        "started_at_utc": started_at_utc,
        "ended_at_utc": ended_at_utc,
        "requested_duration_seconds": requested_duration_seconds,
        "recorded_duration_seconds": recorded_duration_seconds,
        "source": source,
        "packets": {
            "path": PACKETS_FILENAME,
            "bytes": packets_bytes,
            "sha256": packets_sha256,
        },
    }
    metadata.update(stats.to_dict(decoder))
    return metadata


def metadata_parity_errors(metadata: dict, *, decoder: rw.StreamDecoder, stats: StreamStats, packets_bytes: int, packets_sha256: str) -> list[str]:
    """Return integrity/parity mismatches between metadata and a replayed packets.bin."""
    errors: list[str] = []
    if metadata.get("format") != CAPTURE_FORMAT:
        errors.append("unsupported capture format")
    if metadata.get("format_version") != CAPTURE_FORMAT_VERSION:
        errors.append("unsupported capture format version")

    packets = metadata.get("packets", {})
    if packets.get("bytes") != packets_bytes:
        errors.append("packets byte count differs from metadata")
    if packets.get("sha256") != packets_sha256:
        errors.append("packets SHA-256 differs from metadata")

    decoder_meta = metadata.get("decoder", {})
    for key, actual in (
        ("frames_ok", decoder.frames_ok),
        ("frames_bad", decoder.frames_bad),
        ("empty_delimiters", decoder.empty_delimiters),
        ("semantic_errors", stats.semantic_errors),
    ):
        if decoder_meta.get(key) != actual:
            errors.append("decoder.{} differs from metadata".format(key))

    stream_meta = metadata.get("stream", {})
    if stream_meta.get("first_sequence") != stats.first_sequence:
        errors.append("stream.first_sequence differs from metadata")
    if stream_meta.get("last_sequence") != stats.last_sequence:
        errors.append("stream.last_sequence differs from metadata")
    if stream_meta.get("sequence_gaps") != stats.sequence_gaps:
        errors.append("stream.sequence_gaps differs from metadata")
    if stream_meta.get("record_counts") != dict(sorted(stats.record_counts.items())):
        errors.append("stream.record_counts differs from metadata")

    return errors


def stream_issue_count(decoder: rw.StreamDecoder, stats: StreamStats) -> int:
    return (
        decoder.frames_bad
        + stats.semantic_errors
        + stats.sequence_gaps
        + sum(stats.health_deltas().values())
    )
