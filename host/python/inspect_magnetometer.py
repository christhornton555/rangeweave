"""Inspect raw LIS3MDL data in a canonical Rangeweave capture.

The output is intentionally limited to acquisition/configuration quality and the
native ``mag_sensor`` vector.  It does not apply a guessed sensor/body mapping,
hard/soft-iron calibration, magnetic declination, or heading correction.
"""

from __future__ import annotations

import argparse
from pathlib import Path
import statistics

import rangeweave_capture as capture_model
import rangeweave_magnetometer as magnetometer
import rangeweave_protocol as rw


def _packets_path(capture: Path) -> Path:
    return capture / capture_model.PACKETS_FILENAME if capture.is_dir() else capture


def decode_capture(path: Path):
    packets = _packets_path(path)
    decoder = rw.StreamDecoder()
    stats = capture_model.StreamStats()
    mag_records = []

    with packets.open("rb") as stream:
        while True:
            chunk = stream.read(4096)
            if not chunk:
                break
            for frame in decoder.feed(chunk):
                stats.consume(frame)
                if frame.record_type != rw.RECORD_MAG:
                    continue
                try:
                    record = rw.decode_record(frame)
                except rw.ProtocolError:
                    continue
                mag_records.append(record)

    return packets, tuple(mag_records), decoder, stats


def _fmt_optional(value, pattern):
    return "n/a" if value is None else pattern.format(value)


def _stream_clean(decoder, stats) -> bool:
    deltas = stats.health_deltas()
    return bool(
        decoder.frames_bad == 0
        and stats.semantic_errors == 0
        and stats.sequence_gaps == 0
        and deltas
        and all(int(value) == 0 for value in deltas.values())
    )


def _cadence_note(summary, config) -> str:
    rate = summary.observed_rate_hz
    if rate is None:
        return "UNKNOWN (insufficient samples)"
    if summary.overrun_count and rate < 0.75 * config.output_data_rate_hz:
        return (
            "WARN (producer records slower than sensor ODR; unread conversions are being overwritten)"
        )
    if summary.overrun_count:
        return "WARN (sensor reports output-register overwrite before some reads)"
    return "PASS"


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Inspect native LIS3MDL acquisition/configuration quality from a Rangeweave capture"
    )
    parser.add_argument("capture", help="capture directory or packets.bin")
    args = parser.parse_args()

    try:
        packets, raw_records, decoder, stats = decode_capture(Path(args.capture))
        config = magnetometer.config_from_stream_info(stats.last_info)
        samples = magnetometer.convert_samples(raw_records, config)
        summary = magnetometer.summarise(samples, config)
    except (OSError, ValueError, rw.ProtocolError, magnetometer.MagnetometerError) as exc:
        parser.error(str(exc))

    read_durations = [sample.read_duration_us for sample in samples]
    retries_from_status = None
    if stats.first_status is not None and stats.last_status is not None:
        retries_from_status = capture_model.counter_delta(
            stats.last_status.mag_retries,
            stats.first_status.mag_retries,
        )

    print("Rangeweave LIS3MDL magnetometer inspection")
    print(f"  capture:          {packets}")
    print(f"  MAG samples:      {summary.sample_count}")
    print(f"  stream health:    {'PASS' if _stream_clean(decoder, stats) else 'NOT CLEAN'}")
    print(f"  decoder bad:      {decoder.frames_bad}")
    print(f"  semantic errors:  {stats.semantic_errors}")
    print(f"  sequence gaps:    {stats.sequence_gaps}")
    print(
        "  health deltas:    "
        + (
            "all zero"
            if stats.health_deltas() and all(v == 0 for v in stats.health_deltas().values())
            else str(stats.health_deltas() or "unavailable")
        )
    )

    print()
    print("Recorded LIS3MDL configuration")
    print(f"  CTRL_REG1..5:     {config.ctrl_regs_1_to_5.hex(' ').upper()}")
    print(f"  full scale:       +/-{config.full_scale_gauss:g} gauss")
    print(f"  sensitivity:      {config.sensitivity_lsb_per_gauss:g} LSB/gauss")
    print(f"  sensor ODR:       {config.output_data_rate_hz:g} Hz")
    print(f"  XY performance:   {config.xy_performance_mode}")
    print(f"  Z performance:    {config.z_performance_mode}")
    print(f"  measurement mode: {config.measurement_mode}")
    print(f"  block update:     {'enabled' if config.block_data_update else 'disabled'}")

    print()
    print("Acquisition timing")
    print(f"  sample span:      {summary.span_s:.3f} s")
    print(
        f"  observed rate:    {_fmt_optional(summary.observed_rate_hz, '{:.3f} Hz')}"
    )
    print(
        f"  interval median:  {_fmt_optional(summary.median_interval_ms, '{:.3f} ms')}"
    )
    print(f"  interval p95:     {_fmt_optional(summary.p95_interval_ms, '{:.3f} ms')}")
    print(f"  interval max:     {_fmt_optional(summary.max_interval_ms, '{:.3f} ms')}")
    print(f"  read median:      {statistics.median(read_durations):.1f} us")
    print(f"  read max:         {max(read_durations)} us")
    print(f"  record retry flag:{summary.retry_count:>6}")
    if retries_from_status is not None:
        print(f"  STATUS retries:   {retries_from_status}")
    print(f"  not-ready records:{summary.not_ready_count:>6}")
    print(f"  overrun records:  {summary.overrun_count:>6}")
    print(f"  cadence status:   {_cadence_note(summary, config)}")

    print()
    print("Native mag_sensor field (uncalibrated)")
    print(
        "  X range:          {:+.2f} .. {:+.2f} uT   span {:.2f} uT".format(
            summary.x_range_ut[0], summary.x_range_ut[1], summary.axis_span_ut[0]
        )
    )
    print(
        "  Y range:          {:+.2f} .. {:+.2f} uT   span {:.2f} uT".format(
            summary.y_range_ut[0], summary.y_range_ut[1], summary.axis_span_ut[1]
        )
    )
    print(
        "  Z range:          {:+.2f} .. {:+.2f} uT   span {:.2f} uT".format(
            summary.z_range_ut[0], summary.z_range_ut[1], summary.axis_span_ut[2]
        )
    )
    print(
        "  peak FS use:      X {:.1f}%  Y {:.1f}%  Z {:.1f}%".format(
            *(100.0 * value for value in summary.max_axis_utilisation_fraction)
        )
    )
    print(
        "  |B|:              min {:.2f}, p05 {:.2f}, median {:.2f}, p95 {:.2f}, max {:.2f} uT".format(
            summary.norm_min_ut,
            summary.norm_p05_ut,
            summary.norm_median_ut,
            summary.norm_p95_ut,
            summary.norm_max_ut,
        )
    )

    print()
    print("Interpretation")
    print("  - values above are in the native mag_sensor frame; no body-axis mapping is assumed")
    print("  - stream health PASS covers protocol/producer counters, not LIS3MDL output-register overruns")
    print("  - an overrun means an unread conversion was overwritten; the recorded BDU-protected XYZ sample is still coherent")
    print("  - field magnitude is shown diagnostically, not accepted as an Earth-field/heading gate yet")
    print("  - hard-iron, soft-iron and environmental disturbance are not corrected")
    print("  - this inspector must not be used to claim magnetic heading accuracy")

    return 0 if _stream_clean(decoder, stats) else 1


if __name__ == "__main__":
    raise SystemExit(main())
