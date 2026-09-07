"""Fit and inspect a diagnostic hard-iron-only sphere model from a capture."""

from __future__ import annotations

import argparse
from pathlib import Path

import rangeweave_capture as capture_model
import rangeweave_magnetometer as magnetometer
import rangeweave_magnetometer_calibration as calibration
import rangeweave_protocol as rw


def _packets_path(capture: Path) -> Path:
    return capture / capture_model.PACKETS_FILENAME if capture.is_dir() else capture


def _decode_capture(path: Path):
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


def _stream_clean(decoder, stats) -> bool:
    deltas = stats.health_deltas()
    return bool(
        decoder.frames_bad == 0
        and stats.semantic_errors == 0
        and stats.sequence_gaps == 0
        and deltas
        and all(int(value) == 0 for value in deltas.values())
    )


def main() -> int:
    parser = argparse.ArgumentParser(
        description=(
            "Fit a diagnostic hard-iron-only sphere in native mag_sensor coordinates; "
            "does not promote a calibration artifact"
        )
    )
    parser.add_argument("capture", help="capture directory or packets.bin")
    args = parser.parse_args()

    try:
        packets, raw_records, decoder, stats = _decode_capture(Path(args.capture))
        config = magnetometer.config_from_stream_info(stats.last_info)
        samples = magnetometer.convert_samples(raw_records, config)
        vectors = [(sample.x_ut, sample.y_ut, sample.z_ut) for sample in samples]
        fit = calibration.fit_hard_iron_sphere(vectors)
    except (
        OSError,
        ValueError,
        rw.ProtocolError,
        magnetometer.MagnetometerError,
        calibration.MagnetometerCalibrationError,
    ) as exc:
        parser.error(str(exc))

    reduction = fit.robust_span_reduction_fraction

    print("Rangeweave diagnostic hard-iron sphere fit")
    print(f"  capture:          {packets}")
    print(f"  MAG samples:      {fit.sample_count}")
    print(f"  stream health:    {'PASS' if _stream_clean(decoder, stats) else 'NOT CLEAN'}")
    print()
    print("Native mag_sensor hard-iron-only sphere")
    print(
        "  centre:           X {:+.3f}  Y {:+.3f}  Z {:+.3f} uT".format(
            *fit.center_ut
        )
    )
    print(f"  |centre|:         {fit.center_magnitude_ut:.3f} uT")
    print(f"  fitted radius:    {fit.radius_ut:.3f} uT")
    print()
    print("Field-magnitude spread")
    print(
        "  raw |B|:          p05 {:.3f}  median {:.3f}  p95 {:.3f} uT   span {:.3f} uT".format(
            fit.raw_norm_p05_ut,
            fit.raw_norm_median_ut,
            fit.raw_norm_p95_ut,
            fit.raw_robust_span_ut,
        )
    )
    print(
        "  centred |B|:      p05 {:.3f}  median {:.3f}  p95 {:.3f} uT   span {:.3f} uT".format(
            fit.corrected_norm_p05_ut,
            fit.corrected_norm_median_ut,
            fit.corrected_norm_p95_ut,
            fit.corrected_robust_span_ut,
        )
    )
    if reduction is None:
        print("  span reduction:   n/a")
    else:
        print(f"  span reduction:   {100.0 * reduction:.1f}%")
    print()
    print("Sphere residuals after centre subtraction")
    print(f"  radial RMS:       {fit.radial_residual_rms_ut:.3f} uT")
    print(f"  radial |err| p95: {fit.radial_residual_p95_abs_ut:.3f} uT")
    print(f"  radial |err| max: {fit.radial_residual_max_abs_ut:.3f} uT")
    print()
    print("Interpretation")
    print("  - this is a diagnostic native-frame hard-iron-only fit, not a promoted calibration")
    print("  - a large reduction in |B| spread means a fixed additive bias explains much of the variation")
    print("  - residual spread can still contain soft-iron distortion, poor orientation coverage, or environmental field changes")
    print("  - the fit assumes one approximately constant external field magnitude during the capture")
    print("  - do not use this result for heading until mapping, soft-iron behaviour, repeatability and disturbance confidence are validated")

    return 0 if _stream_clean(decoder, stats) else 1


if __name__ == "__main__":
    raise SystemExit(main())
