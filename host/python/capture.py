"""Record a canonical Rangeweave capture session from a live serial byte stream."""

from __future__ import annotations

import argparse
import hashlib
import sys
import time

import rangeweave_capture as cap
import rangeweave_protocol as rw


def _read_until_delimiter(port, *, timeout_seconds: float = 2.0) -> bytes:
    """Discard through one delimiter and return any bytes after it from the same read."""
    deadline = time.monotonic() + timeout_seconds
    while time.monotonic() < deadline:
        chunk = port.read(4096)
        if not chunk:
            continue
        index = chunk.find(b"\x00")
        if index >= 0:
            return chunk[index + 1:]
    raise RuntimeError("no Rangeweave frame delimiter observed while aligning capture")


def _write_and_decode(raw_file, digest, decoder, stats, chunk: bytes) -> int:
    if not chunk:
        return 0
    raw_file.write(chunk)
    digest.update(chunk)
    cap.feed_chunk(decoder, stats, chunk)
    return len(chunk)


def _finish_at_delimiter(port, raw_file, digest, decoder, stats, *, timeout_seconds: float = 1.0) -> int:
    """Extend a timed capture only far enough to close the current wire frame."""
    deadline = time.monotonic() + timeout_seconds
    written = 0
    while time.monotonic() < deadline:
        chunk = port.read(4096)
        if not chunk:
            continue
        index = chunk.find(b"\x00")
        if index >= 0:
            prefix = chunk[:index + 1]
            written += _write_and_decode(raw_file, digest, decoder, stats, prefix)
            return written
        written += _write_and_decode(raw_file, digest, decoder, stats, chunk)
    raise RuntimeError("timed out before a closing Rangeweave frame delimiter was observed")


def main() -> int:
    parser = argparse.ArgumentParser(description="Record a canonical Rangeweave capture session")
    parser.add_argument("port", help="serial port, e.g. COM5 or /dev/ttyACM0")
    parser.add_argument("--seconds", type=float, default=30.0, help="requested capture duration")
    parser.add_argument("--warmup", type=float, default=3.0, help="discard startup/backlog for this long")
    parser.add_argument("--baud", type=int, default=115200,
                        help="USB CDC ignores physical baud; retained for serial API compatibility")
    parser.add_argument("--root", default="captures", help="parent directory for capture sessions")
    parser.add_argument("--name", help="optional short label appended to the capture directory name")
    parser.add_argument("--notes", default="", help="text written to notes.txt")
    args = parser.parse_args()

    if args.seconds <= 0:
        parser.error("--seconds must be greater than zero")
    if args.warmup < 0:
        parser.error("--warmup cannot be negative")

    try:
        import serial
    except ImportError:
        raise SystemExit("pyserial is required: python -m pip install pyserial")

    session_dir = cap.make_session_directory(args.root, args.name)
    packets_path = session_dir / cap.PACKETS_FILENAME
    metadata_path = session_dir / cap.METADATA_FILENAME
    notes_path = session_dir / cap.NOTES_FILENAME
    notes_path.write_text(args.notes.rstrip() + ("\n" if args.notes else ""), encoding="utf-8")

    source_metadata = {
        "kind": "serial",
        "port": args.port,
        "baud_api_value": args.baud,
        "warmup_seconds": args.warmup,
    }

    # Leave an explicit marker if the process is interrupted before final metadata is written.
    cap.write_json_atomic(metadata_path, {
        "format": cap.CAPTURE_FORMAT,
        "format_version": cap.CAPTURE_FORMAT_VERSION,
        "status": "recording",
        "source": source_metadata,
        "packets": {"path": cap.PACKETS_FILENAME},
    })

    decoder = rw.StreamDecoder()
    stats = cap.StreamStats()
    digest = hashlib.sha256()
    byte_count = 0
    started_at_utc = None
    started_monotonic = None
    final_status = "complete"

    try:
        with serial.Serial(args.port, args.baud, timeout=0.1) as port, packets_path.open("wb") as raw_file:
            warmup_deadline = time.monotonic() + args.warmup
            while time.monotonic() < warmup_deadline:
                port.read(4096)

            initial = _read_until_delimiter(port)
            started_at_utc = cap.utc_now_iso()
            started_monotonic = time.monotonic()
            byte_count += _write_and_decode(raw_file, digest, decoder, stats, initial)

            deadline = started_monotonic + args.seconds
            while time.monotonic() < deadline:
                chunk = port.read(4096)
                if not chunk:
                    continue
                byte_count += _write_and_decode(raw_file, digest, decoder, stats, chunk)

            # Canonical packets.bin should end at a complete COBS frame boundary.
            byte_count += _finish_at_delimiter(port, raw_file, digest, decoder, stats)
            raw_file.flush()
    except KeyboardInterrupt:
        final_status = "interrupted"
    except Exception:
        final_status = "error"
        raise
    finally:
        ended_at_utc = cap.utc_now_iso()
        if started_at_utc is None:
            started_at_utc = ended_at_utc
        recorded_duration = (
            max(0.0, time.monotonic() - started_monotonic)
            if started_monotonic is not None
            else 0.0
        )
        metadata = cap.build_metadata(
            status=final_status,
            started_at_utc=started_at_utc,
            ended_at_utc=ended_at_utc,
            requested_duration_seconds=args.seconds,
            recorded_duration_seconds=recorded_duration,
            source=source_metadata,
            packets_bytes=byte_count,
            packets_sha256=digest.hexdigest(),
            decoder=decoder,
            stats=stats,
        )
        cap.write_json_atomic(metadata_path, metadata)

    print("Rangeweave capture complete")
    print("  session:       {}".format(session_dir))
    print("  bytes:         {}".format(byte_count))
    print("  SHA-256:       {}".format(digest.hexdigest()))
    print("  frames:        {}".format(decoder.frames_ok))
    print("  bad frames:    {}".format(decoder.frames_bad))
    print("  seq gaps:      {}".format(stats.sequence_gaps))
    for name, count in sorted(stats.record_counts.items()):
        print("  {:14s} {}".format(name + ":", count))
    if stats.last_info is not None:
        print("  session_id:    0x{:016X}".format(stats.last_info.session_id))
    for key, value in stats.health_deltas().items():
        print("  delta {:24s} {}".format(key + ":", value))

    if final_status != "complete":
        return 2
    return 1 if cap.stream_issue_count(decoder, stats) else 0


if __name__ == "__main__":
    sys.exit(main())
