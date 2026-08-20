"""Replay and integrity-check a Rangeweave capture session."""

from __future__ import annotations

import argparse
from pathlib import Path
import sys

import rangeweave_capture as cap


def main() -> int:
    parser = argparse.ArgumentParser(description="Replay Rangeweave packets through the reference StreamDecoder")
    parser.add_argument("capture", help="capture session directory or a raw packets.bin file")
    parser.add_argument("--chunk-size", type=int, default=4096, help="file read chunk size")
    parser.add_argument("--strict", action="store_true",
                        help="return non-zero if the recorded stream contains gaps/errors/health deltas")
    args = parser.parse_args()

    if args.chunk_size <= 0:
        parser.error("--chunk-size must be greater than zero")

    supplied = Path(args.capture)
    metadata = None
    if supplied.is_dir():
        session_dir = supplied
        packets_path = session_dir / cap.PACKETS_FILENAME
        metadata_path = session_dir / cap.METADATA_FILENAME
        if not packets_path.is_file():
            raise SystemExit("capture directory has no {}".format(cap.PACKETS_FILENAME))
        if metadata_path.is_file():
            metadata = cap.load_metadata(session_dir)
    else:
        packets_path = supplied
        if not packets_path.is_file():
            raise SystemExit("capture file not found: {}".format(packets_path))

    decoder, stats, byte_count, sha256 = cap.inspect_file(packets_path, chunk_size=args.chunk_size)

    print("Rangeweave replay summary")
    print("  packets:       {}".format(packets_path))
    print("  bytes:         {}".format(byte_count))
    print("  SHA-256:       {}".format(sha256))
    print("  valid frames:  {}".format(decoder.frames_ok))
    print("  bad frames:    {}".format(decoder.frames_bad))
    print("  semantic errs: {}".format(stats.semantic_errors))
    print("  seq gaps:      {}".format(stats.sequence_gaps))
    for name, count in sorted(stats.record_counts.items()):
        print("  {:14s} {}".format(name + ":", count))

    if stats.last_info is not None:
        print("  session_id:    0x{:016X}".format(stats.last_info.session_id))

    parity_errors = []
    if metadata is not None:
        parity_errors = cap.metadata_parity_errors(
            metadata,
            decoder=decoder,
            stats=stats,
            packets_bytes=byte_count,
            packets_sha256=sha256,
        )
        print("  metadata:      {}".format("PASS" if not parity_errors else "FAIL"))
        for error in parity_errors:
            print("    - {}".format(error))
    else:
        print("  metadata:      not supplied")

    health = stats.health_deltas()
    if health:
        print("  STATUS deltas:")
        for key, value in health.items():
            print("    {}: {}".format(key, value))

    if parity_errors:
        return 2
    if decoder.frames_ok == 0:
        return 2
    if args.strict and cap.stream_issue_count(decoder, stats):
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
