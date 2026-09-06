"""Rank diagnostic ``mag_sensor -> device_body`` axis mappings on one capture.

This tool intentionally searches only the 24 proper signed-permutation mappings.
It is a replay diagnostic to decide whether an existing motion capture contains
enough rotational evidence to distinguish the breakout-board mounting.  It does
not promote calibration and does not fuse magnetic heading.
"""

from __future__ import annotations

import argparse
from pathlib import Path

import rangeweave_capture as capture_model
import rangeweave_extrinsics as ext
import rangeweave_imu_quality as imu_quality
import rangeweave_magnetometer as magnetometer
import rangeweave_magnetometer_mapping as mapping
import rangeweave_orientation as orientation
import rangeweave_protocol as rw


def _packets_path(path: Path) -> Path:
    return path / capture_model.PACKETS_FILENAME if path.is_dir() else path


def _config_byte(info, tag):
    if info is None:
        return None
    value = info.first_value(tag)
    return value[0] if value and len(value) == 1 else None


def _decode(path: Path):
    packets = _packets_path(path)
    decoder = rw.StreamDecoder()
    stats = capture_model.StreamStats()
    imu_samples = []
    clock_syncs = []
    mag_records = []

    with packets.open("rb") as stream:
        while True:
            chunk = stream.read(4096)
            if not chunk:
                break
            for frame in decoder.feed(chunk):
                stats.consume(frame)
                try:
                    record = rw.decode_record(frame)
                except rw.ProtocolError:
                    continue
                if frame.record_type == rw.RECORD_IMU_BATCH:
                    imu_samples.extend(record.samples)
                elif frame.record_type == rw.RECORD_CLOCK_SYNC:
                    clock_syncs.append(record)
                elif frame.record_type == rw.RECORD_MAG:
                    mag_records.append(record)

    return (
        packets,
        tuple(imu_samples),
        tuple(clock_syncs),
        tuple(mag_records),
        decoder,
        stats,
    )


def _stream_clean(decoder, stats) -> bool:
    deltas = stats.health_deltas()
    return bool(
        decoder.frames_bad == 0
        and stats.semantic_errors == 0
        and stats.sequence_gaps == 0
        and deltas
        and all(int(value) == 0 for value in deltas.values())
    )


def _orientation_excursion_deg(run: orientation.OrientationRun) -> float:
    if not run.samples:
        return 0.0
    start_t = ext.transpose(run.samples[0].reference_from_body)
    return max(
        ext.rotation_angle_deg(ext.matrix_multiply(start_t, item.reference_from_body))
        for item in run.samples
    )


def _offsets(minimum: float, maximum: float, step: float) -> tuple[float, ...]:
    if step <= 0.0 or maximum < minimum:
        raise ValueError("timing scan requires step > 0 and max >= min")
    values = []
    value = float(minimum)
    while value <= float(maximum) + step * 1.0e-6:
        values.append(round(value, 9))
        value += step
    return tuple(values)


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Rank proper signed-permutation LIS3MDL body-axis mappings against six-axis orientation replay"
    )
    parser.add_argument("capture", help="capture directory or packets.bin")
    parser.add_argument("--offset-min-ms", type=float, default=-60.0)
    parser.add_argument("--offset-max-ms", type=float, default=0.0)
    parser.add_argument("--offset-step-ms", type=float, default=2.0)
    args = parser.parse_args()

    try:
        (
            packets,
            imu_samples,
            clock_syncs,
            raw_mag,
            decoder,
            stats,
        ) = _decode(Path(args.capture))
        ctrl1 = _config_byte(stats.last_info, rw.INFO_LSM_CTRL1_XL)
        ctrl2 = _config_byte(stats.last_info, rw.INFO_LSM_CTRL2_G)
        if ctrl1 is None or ctrl2 is None:
            raise mapping.MagnetometerMappingError(
                "capture is missing LSM CTRL1_XL/CTRL2_G STREAM_INFO metadata"
            )
        config = magnetometer.config_from_stream_info(stats.last_info)
        mag_samples = magnetometer.convert_samples(raw_mag, config)
        mag_summary = magnetometer.summarise(mag_samples, config)
        gyro_usage = imu_quality.analyse_gyro_range(imu_samples, ctrl2_g=ctrl2)
        orientation_run = orientation.estimate_orientation(
            imu_samples,
            clock_syncs,
            ctrl1_xl=ctrl1,
            ctrl2_g=ctrl2,
        )
        offsets = _offsets(args.offset_min_ms, args.offset_max_ms, args.offset_step_ms)
        scores = mapping.scan_signed_permutation_mappings(
            mag_samples,
            imu_samples,
            orientation_run,
            offsets,
        )
        distinct = mapping.best_distinct_mappings(scores)
    except (
        OSError,
        ValueError,
        magnetometer.MagnetometerError,
        mapping.MagnetometerMappingError,
        orientation.OrientationError,
        imu_quality.ImuQualityError,
    ) as exc:
        parser.error(str(exc))

    best = distinct[0]
    second = distinct[1] if len(distinct) > 1 else None
    overrun_fraction = mag_summary.overrun_count / max(1, mag_summary.sample_count)

    print("Rangeweave diagnostic magnetometer/body mapping search")
    print(f"  capture:          {packets}")
    print(f"  stream health:    {'PASS' if _stream_clean(decoder, stats) else 'NOT CLEAN'}")
    print(f"  IMU samples:      {len(imu_samples)}")
    print(f"  MAG samples:      {len(mag_samples)}")
    print(f"  CLOCK_SYNC:       {len(clock_syncs)}")
    print(f"  orientation excursion: {_orientation_excursion_deg(orientation_run):.3f} deg")
    print(
        "  gyro range:       {} (peak X {:.1f}% Y {:.1f}% Z {:.1f}%)".format(
            "PASS" if gyro_usage.pass_range else "FAIL",
            100.0 * gyro_usage.peak_fraction[0],
            100.0 * gyro_usage.peak_fraction[1],
            100.0 * gyro_usage.peak_fraction[2],
        )
    )

    print()
    print("Magnetic acquisition caveat")
    print(f"  sensor ODR:       {config.output_data_rate_hz:.3f} Hz")
    print(f"  recorded rate:    {mag_summary.observed_rate_hz:.3f} Hz")
    print(
        f"  overrun records:  {mag_summary.overrun_count} / {mag_summary.sample_count} ({100.0 * overrun_fraction:.1f}%)"
    )
    print(
        "  implication:      effective magnetic sample time precedes the MCU read; a constant offset is scanned diagnostically"
    )

    print()
    print("Signed-permutation mapping search")
    print(
        f"  timing scan:      {offsets[0]:+.1f} .. {offsets[-1]:+.1f} ms in {args.offset_step_ms:.1f} ms steps"
    )
    print("  rank  mapping                                             offset    RMS    p95    max   vecRMS  n")
    for index, score in enumerate(distinct[:5], start=1):
        print(
            "  {:>2d}.   {:<50s} {:+6.1f}  {:5.2f}  {:5.2f}  {:5.2f}  {:6.2f} {:3d}".format(
                index,
                score.mapping.name,
                score.time_offset_ms,
                score.direction_rms_deg,
                score.direction_p95_deg,
                score.direction_max_deg,
                score.vector_rms_ut,
                score.sample_count,
            )
        )

    print()
    print("Best candidate")
    print(f"  mapping:          {best.mapping.name}")
    print(f"  R_body_from_mag:")
    for row in best.mapping.rotation_body_from_mag:
        print("                   [{:+.0f} {:+.0f} {:+.0f}]".format(*row))
    print(f"  best offset:      {best.time_offset_ms:+.1f} ms")
    print(f"  direction RMS:    {best.direction_rms_deg:.3f} deg")
    print(f"  direction p95:    {best.direction_p95_deg:.3f} deg")
    print(f"  vector RMS:       {best.vector_rms_ut:.3f} uT")
    if second is not None:
        print(f"  second RMS:       {second.direction_rms_deg:.3f} deg")
        print(f"  RMS separation:   {second.direction_rms_deg - best.direction_rms_deg:.3f} deg")
        if best.direction_rms_deg > 1.0e-9:
            print(f"  second/best:      {second.direction_rms_deg / best.direction_rms_deg:.3f}x")

    print()
    print("Interpretation")
    print("  - this is a diagnostic ranking, not a promoted mag_sensor -> device_body calibration")
    print("  - the search assumes an orthogonal breakout mounting representable by a signed permutation")
    print("  - hard/soft-iron distortion is not corrected, so absolute residuals are not heading accuracy")
    print("  - the timing offset is exploratory because protocol v0.1 timestamps the MCU read bracket, not the LIS3MDL conversion instant")
    print("  - a clear winner can guide the purpose-made physical axis test; ambiguous rankings mean this capture lacks observability")

    return 0 if _stream_clean(decoder, stats) and gyro_usage.pass_range else 1


if __name__ == "__main__":
    raise SystemExit(main())
