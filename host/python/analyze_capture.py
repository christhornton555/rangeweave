"""Offline timing analysis for raw Rangeweave v0.1 captures.

This is a validation/debugging aid for packets.bin files produced by probe_serial.py.
It intentionally uses only the existing standard-library Rangeweave decoder.
"""

import argparse
from collections import Counter
from pathlib import Path
import statistics

import rangeweave_protocol as rw


RECORD_NAMES = {
    rw.RECORD_IMU_BATCH: "IMU_BATCH",
    rw.RECORD_MAG: "MAG",
    rw.RECORD_TOF_GRID: "TOF_GRID",
    rw.RECORD_CLOCK_SYNC: "CLOCK_SYNC",
    rw.RECORD_STATUS: "STATUS",
    rw.RECORD_STREAM_INFO: "STREAM_INFO",
}


def percentile(values, fraction):
    if not values:
        return None
    ordered = sorted(values)
    if len(ordered) == 1:
        return float(ordered[0])
    position = fraction * (len(ordered) - 1)
    lower = int(position)
    upper = min(lower + 1, len(ordered) - 1)
    weight = position - lower
    return ordered[lower] * (1.0 - weight) + ordered[upper] * weight


def print_us_stats(label, values):
    if not values:
        print("  {}: no data".format(label))
        return
    print(
        "  {}: min={:.1f} us median={:.1f} us mean={:.1f} us "
        "p90={:.1f} us p99={:.1f} us max={:.1f} us".format(
            label,
            min(values),
            statistics.median(values),
            statistics.mean(values),
            percentile(values, 0.90),
            percentile(values, 0.99),
            max(values),
        )
    )


def sequence_gap(previous, current):
    expected = (previous + 1) & 0xFFFFFFFF
    if current == expected:
        return 0
    return (current - expected) & 0xFFFFFFFF


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("capture", help="raw packets.bin capture")
    parser.add_argument(
        "--expected-tof-hz",
        type=float,
        default=None,
        help="override expected ToF rate; otherwise read STREAM_INFO when available",
    )
    args = parser.parse_args()

    path = Path(args.capture)
    data = path.read_bytes()

    decoder = rw.StreamDecoder()
    counts = Counter()
    semantic_errors = 0
    seq_gaps = 0
    last_sequence = None

    tof_ready = []
    tof_read_duration = []
    imu_ticks = []
    statuses = []
    stream_infos = []

    # Feed in chunks so this exercises the same incremental decoder behaviour used live.
    for offset in range(0, len(data), 4096):
        for frame in decoder.feed(data[offset:offset + 4096]):
            counts[RECORD_NAMES.get(frame.record_type, "0x{:02X}".format(frame.record_type))] += 1

            if last_sequence is not None:
                seq_gaps += sequence_gap(last_sequence, frame.sequence)
            last_sequence = frame.sequence

            try:
                record = rw.decode_record(frame)
            except rw.ProtocolError:
                semantic_errors += 1
                continue

            if frame.record_type == rw.RECORD_TOF_GRID:
                tof_ready.append(record.mcu_ready_us)
                tof_read_duration.append(record.mcu_read_complete_us - record.mcu_ready_us)
            elif frame.record_type == rw.RECORD_IMU_BATCH:
                imu_ticks.extend(sample.lsm_tick for sample in record.samples)
            elif frame.record_type == rw.RECORD_STATUS:
                statuses.append(record)
            elif frame.record_type == rw.RECORD_STREAM_INFO:
                stream_infos.append(record)

    expected_tof_hz = args.expected_tof_hz
    firmware_label = None
    source_profile = None
    if stream_infos:
        info = stream_infos[-1]
        value = info.first_value(rw.INFO_FIRMWARE_LABEL)
        if value:
            firmware_label = value.decode("utf-8", "replace")
        value = info.first_value(rw.INFO_SOURCE_PROFILE)
        if value:
            source_profile = value.decode("utf-8", "replace")
        if expected_tof_hz is None:
            value = info.first_value(rw.INFO_TOF_GRID_CONFIG)
            if value and len(value) >= 3 and value[2]:
                expected_tof_hz = float(value[2])

    print("Rangeweave capture analysis")
    print("  file:          {}".format(path))
    print("  bytes:         {}".format(len(data)))
    print("  valid frames:  {}".format(decoder.frames_ok))
    print("  bad frames:    {}".format(decoder.frames_bad))
    print("  seq gaps:      {}".format(seq_gaps))
    print("  semantic errs: {}".format(semantic_errors))
    if firmware_label:
        print("  firmware:      {}".format(firmware_label))
    if source_profile:
        print("  source:        {}".format(source_profile))
    for name in sorted(counts):
        print("  {:14s} {}".format(name + ":", counts[name]))

    print()
    print("ToF timing from MCU timestamps")
    if len(tof_ready) < 2:
        print("  fewer than two TOF_GRID records")
    else:
        ready_intervals = [b - a for a, b in zip(tof_ready, tof_ready[1:])]
        span_us = tof_ready[-1] - tof_ready[0]
        observed_hz = (len(tof_ready) - 1) * 1_000_000.0 / span_us if span_us > 0 else 0.0
        print("  frames:        {}".format(len(tof_ready)))
        print("  ready span:    {:.6f} s".format(span_us / 1_000_000.0))
        print("  observed rate: {:.4f} Hz".format(observed_hz))
        print_us_stats("ready interval", ready_intervals)
        print_us_stats("get_data duration", tof_read_duration)

        if expected_tof_hz:
            period_us = 1_000_000.0 / expected_tof_hz
            print("  expected rate: {:.3f} Hz ({:.1f} us period)".format(expected_tof_hz, period_us))

            multiple_histogram = Counter()
            inferred_skipped_periods = 0
            long_intervals = []
            for interval in ready_intervals:
                multiple = max(1, int(round(interval / period_us)))
                multiple_histogram[multiple] += 1
                inferred_skipped_periods += max(0, multiple - 1)
                if interval > 1.5 * period_us:
                    long_intervals.append(interval)

            print("  interval multiples of nominal period:")
            for multiple in sorted(multiple_histogram):
                print("    {:2d}x: {}".format(multiple, multiple_histogram[multiple]))
            print("  inferred skipped sensor periods: {}".format(inferred_skipped_periods))
            if long_intervals:
                print_us_stats("long ready intervals", long_intervals)

    print()
    print("IMU native timestamp cadence")
    if len(imu_ticks) < 2:
        print("  fewer than two IMU samples")
    else:
        imu_deltas = [b - a for a, b in zip(imu_ticks, imu_ticks[1:]) if b >= a]
        print("  samples:       {}".format(len(imu_ticks)))
        print_us_stats("LSM tick delta (ticks, label only)", imu_deltas)

    if statuses:
        print()
        print("STATUS range in capture")
        first = statuses[0]
        last = statuses[-1]
        print("  status records: {}".format(len(statuses)))
        print("  MCU span:       {:.6f} s".format((last.mcu_time_us - first.mcu_time_us) / 1_000_000.0))
        print("  frames_dropped: {} -> {}".format(first.frames_dropped, last.frames_dropped))
        print("  imu_dropped:    {} -> {}".format(first.imu_samples_dropped, last.imu_samples_dropped))
        print("  tof_errors:     {} -> {}".format(first.tof_errors, last.tof_errors))

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
