"""Small live USB-CDC smoke probe for Rangeweave v0.1 streams.

This is intentionally not yet the full capture/replay layer. It exists so Pico acquisition
firmware can be hardware-validated without dumping binary data into a terminal.
"""

import argparse
from collections import Counter
from dataclasses import asdict
import sys
import time

try:
    import serial
except ImportError:
    raise SystemExit("pyserial is required: python -m pip install pyserial")

import rangeweave_protocol as rw


def record_name(record_type):
    return {
        rw.RECORD_IMU_BATCH: "IMU_BATCH",
        rw.RECORD_MAG: "MAG",
        rw.RECORD_TOF_GRID: "TOF_GRID",
        rw.RECORD_CLOCK_SYNC: "CLOCK_SYNC",
        rw.RECORD_STATUS: "STATUS",
        rw.RECORD_STREAM_INFO: "STREAM_INFO",
    }.get(record_type, "0x{:02X}".format(record_type))


def counter_delta(new_value, old_value):
    return (new_value - old_value) & 0xFFFFFFFF


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("port", help="serial port, e.g. COM7 or /dev/ttyACM0")
    parser.add_argument("--seconds", type=float, default=15.0,
                        help="measurement duration after warm-up")
    parser.add_argument("--warmup", type=float, default=3.0,
                        help="decode/discard backlog for this many seconds first")
    parser.add_argument(
        "--baud",
        type=int,
        default=115200,
        help="USB CDC ignores physical baud; kept for the serial API",
    )
    parser.add_argument("--output", help="optional raw packets.bin output path")
    args = parser.parse_args()

    decoder = rw.StreamDecoder()
    counts = Counter()
    last_sequence = None
    sequence_gaps = 0
    first_status = None
    last_status = None
    last_info = None
    bad_frames_at_measurement_start = 0
    raw_file = open(args.output, "wb") if args.output else None

    opened_at = time.monotonic()
    measurement_started = None

    try:
        with serial.Serial(args.port, args.baud, timeout=0.1) as port:
            while True:
                now = time.monotonic()
                if measurement_started is None and now - opened_at >= args.warmup:
                    measurement_started = now
                    bad_frames_at_measurement_start = decoder.frames_bad
                    counts.clear()
                    last_sequence = None
                    sequence_gaps = 0
                    first_status = None
                    last_status = None
                    last_info = None

                if measurement_started is not None:
                    if now - measurement_started >= args.seconds:
                        break

                chunk = port.read(4096)
                if not chunk:
                    continue
                if raw_file is not None:
                    raw_file.write(chunk)

                for frame in decoder.feed(chunk):
                    if measurement_started is None:
                        continue

                    counts[record_name(frame.record_type)] += 1

                    if last_sequence is not None:
                        expected = (last_sequence + 1) & 0xFFFFFFFF
                        if frame.sequence != expected:
                            sequence_gaps += (frame.sequence - expected) & 0xFFFFFFFF
                    last_sequence = frame.sequence

                    try:
                        record = rw.decode_record(frame)
                    except rw.ProtocolError:
                        counts["semantic_errors"] += 1
                        continue

                    if frame.record_type == rw.RECORD_STATUS:
                        if first_status is None:
                            first_status = record
                        last_status = record
                    elif frame.record_type == rw.RECORD_STREAM_INFO:
                        last_info = record
    finally:
        if raw_file is not None:
            raw_file.close()

    measurement_bad_frames = decoder.frames_bad - bad_frames_at_measurement_start

    print("Rangeweave live probe summary")
    print("  warm-up:       {:.1f} s".format(args.warmup))
    print("  measured:      {:.1f} s".format(args.seconds))
    print("  bad frames:    {}".format(measurement_bad_frames))
    print("  seq gaps:      {}".format(sequence_gaps))
    for name in sorted(counts):
        print("  {:14s} {}".format(name + ":", counts[name]))

    if last_info is not None:
        print("  session_id:    0x{:016X}".format(last_info.session_id))
        print("  info_revision: {}".format(last_info.info_revision))

    if last_status is not None:
        print("  last STATUS:")
        for key, value in asdict(last_status).items():
            print("    {}: {}".format(key, value))

    health_error_deltas = 0
    if first_status is not None and last_status is not None:
        print("  STATUS deltas during measured window:")
        for key in (
            "frames_dropped",
            "imu_samples_dropped",
            "fifo_overruns",
            "fifo_structural_errors",
            "mag_errors",
            "tof_errors",
            "clock_sync_errors",
        ):
            delta = counter_delta(getattr(last_status, key), getattr(first_status, key))
            health_error_deltas += delta
            print("    {}: {}".format(key, delta))

    if sum(counts.values()) == 0:
        return 2
    if measurement_bad_frames or sequence_gaps or health_error_deltas:
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
