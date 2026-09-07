"""Compare raw and diagnostic hard-iron-centred magnetometer/body mapping scores.

This tool is intentionally an A/B replay diagnostic.  It fits a native
``mag_sensor`` hard-iron sphere from the same capture, subtracts only that fitted
centre, and reruns the existing 24-way signed-permutation/timing search.

The same-capture fit is not an independently validated calibration and must not be
promoted for heading use.  Its purpose is to test whether fixed additive magnetic
bias is what obscured the axis-mapping result.
"""

from __future__ import annotations

import argparse
from dataclasses import replace
import math
from pathlib import Path

import inspect_magnetometer_mapping as raw_inspector
import rangeweave_imu_quality as imu_quality
import rangeweave_magnetometer as magnetometer
import rangeweave_magnetometer_calibration as calibration
import rangeweave_magnetometer_mapping as mapping
import rangeweave_orientation as orientation
import rangeweave_protocol as rw


def centre_samples(
    samples: tuple[magnetometer.MagneticSample, ...],
    centre_ut: tuple[float, float, float],
) -> tuple[magnetometer.MagneticSample, ...]:
    """Return copies with a diagnostic native-frame additive bias removed."""

    if len(centre_ut) != 3:
        raise ValueError("hard-iron centre must contain exactly three components")

    corrected = []
    for sample in samples:
        x_ut = float(sample.x_ut) - float(centre_ut[0])
        y_ut = float(sample.y_ut) - float(centre_ut[1])
        z_ut = float(sample.z_ut) - float(centre_ut[2])
        corrected.append(
            replace(
                sample,
                x_ut=x_ut,
                y_ut=y_ut,
                z_ut=z_ut,
                norm_ut=math.sqrt(x_ut * x_ut + y_ut * y_ut + z_ut * z_ut),
            )
        )
    return tuple(corrected)


def _print_ranking(title: str, distinct: tuple[mapping.MappingScore, ...]) -> None:
    print(title)
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


def _second_ratio(distinct: tuple[mapping.MappingScore, ...]) -> float | None:
    if len(distinct) < 2 or distinct[0].direction_rms_deg <= 1.0e-12:
        return None
    return distinct[1].direction_rms_deg / distinct[0].direction_rms_deg


def main() -> int:
    parser = argparse.ArgumentParser(
        description=(
            "Compare raw versus same-capture hard-iron-centred LIS3MDL "
            "body-axis mapping rankings"
        )
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
        ) = raw_inspector._decode(Path(args.capture))
        ctrl1 = raw_inspector._config_byte(stats.last_info, rw.INFO_LSM_CTRL1_XL)
        ctrl2 = raw_inspector._config_byte(stats.last_info, rw.INFO_LSM_CTRL2_G)
        if ctrl1 is None or ctrl2 is None:
            raise mapping.MagnetometerMappingError(
                "capture is missing LSM CTRL1_XL/CTRL2_G STREAM_INFO metadata"
            )

        config = magnetometer.config_from_stream_info(stats.last_info)
        mag_samples = magnetometer.convert_samples(raw_mag, config)
        mag_summary = magnetometer.summarise(mag_samples, config)

        fit = calibration.fit_hard_iron_sphere(
            (sample.x_ut, sample.y_ut, sample.z_ut) for sample in mag_samples
        )
        centred_samples = centre_samples(mag_samples, fit.center_ut)

        gyro_usage = imu_quality.analyse_gyro_range(imu_samples, ctrl2_g=ctrl2)
        orientation_run = orientation.estimate_orientation(
            imu_samples,
            clock_syncs,
            ctrl1_xl=ctrl1,
            ctrl2_g=ctrl2,
        )
        offsets = raw_inspector._offsets(
            args.offset_min_ms,
            args.offset_max_ms,
            args.offset_step_ms,
        )

        raw_scores = mapping.scan_signed_permutation_mappings(
            mag_samples,
            imu_samples,
            orientation_run,
            offsets,
        )
        centred_scores = mapping.scan_signed_permutation_mappings(
            centred_samples,
            imu_samples,
            orientation_run,
            offsets,
        )
        raw_distinct = mapping.best_distinct_mappings(raw_scores)
        centred_distinct = mapping.best_distinct_mappings(centred_scores)
    except (
        OSError,
        ValueError,
        magnetometer.MagnetometerError,
        calibration.MagnetometerCalibrationError,
        mapping.MagnetometerMappingError,
        orientation.OrientationError,
        imu_quality.ImuQualityError,
    ) as exc:
        parser.error(str(exc))

    raw_best = raw_distinct[0]
    centred_best = centred_distinct[0]
    raw_ratio = _second_ratio(raw_distinct)
    centred_ratio = _second_ratio(centred_distinct)
    overrun_fraction = mag_summary.overrun_count / max(1, mag_summary.sample_count)
    reduction = fit.robust_span_reduction_fraction

    print("Rangeweave diagnostic raw vs hard-iron-centred mapping comparison")
    print(f"  capture:          {packets}")
    print(
        f"  stream health:    "
        f"{'PASS' if raw_inspector._stream_clean(decoder, stats) else 'NOT CLEAN'}"
    )
    print(f"  IMU samples:      {len(imu_samples)}")
    print(f"  MAG samples:      {len(mag_samples)}")
    print(f"  CLOCK_SYNC:       {len(clock_syncs)}")
    print(
        f"  orientation excursion: "
        f"{raw_inspector._orientation_excursion_deg(orientation_run):.3f} deg"
    )
    print(
        "  gyro range:       {} (peak X {:.1f}% Y {:.1f}% Z {:.1f}%)".format(
            "PASS" if not gyro_usage.rejected else "FAIL",
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
        f"  overrun records:  {mag_summary.overrun_count} / "
        f"{mag_summary.sample_count} ({100.0 * overrun_fraction:.1f}%)"
    )
    print(
        "  implication:      effective magnetic sample time precedes the MCU read; "
        "the same constant offset scan is used in both arms"
    )

    print()
    print("Same-capture diagnostic hard-iron sphere")
    print(
        "  centre:           X {:+.3f}  Y {:+.3f}  Z {:+.3f} uT".format(
            *fit.center_ut
        )
    )
    print(f"  |centre|:         {fit.center_magnitude_ut:.3f} uT")
    print(f"  fitted radius:    {fit.radius_ut:.3f} uT")
    print(
        f"  raw |B| span:     {fit.raw_robust_span_ut:.3f} uT "
        "(p05 to p95)"
    )
    print(
        f"  centred |B| span: {fit.corrected_robust_span_ut:.3f} uT "
        "(p05 to p95)"
    )
    print(
        "  span reduction:   "
        + ("n/a" if reduction is None else f"{100.0 * reduction:.1f}%")
    )
    print(f"  radial RMS:       {fit.radial_residual_rms_ut:.3f} uT")

    print()
    print(
        f"Timing scan: {offsets[0]:+.1f} .. {offsets[-1]:+.1f} ms "
        f"in {args.offset_step_ms:.1f} ms steps"
    )
    print()
    _print_ranking("Raw mapping ranking", raw_distinct)
    print()
    _print_ranking("Hard-iron-centred mapping ranking", centred_distinct)

    print()
    print("A/B summary")
    print(f"  raw winner:       {raw_best.mapping.name}")
    print(f"  raw RMS:          {raw_best.direction_rms_deg:.3f} deg")
    if raw_ratio is not None:
        print(f"  raw second/best:  {raw_ratio:.3f}x")
    print(f"  centred winner:   {centred_best.mapping.name}")
    print(f"  centred RMS:      {centred_best.direction_rms_deg:.3f} deg")
    if centred_ratio is not None:
        print(f"  centred 2nd/best: {centred_ratio:.3f}x")
    print(
        f"  same winner:      "
        f"{'YES' if raw_best.mapping.name == centred_best.mapping.name else 'NO'}"
    )
    if raw_best.direction_rms_deg > 1.0e-12:
        rms_drop = 1.0 - centred_best.direction_rms_deg / raw_best.direction_rms_deg
        print(f"  best RMS change:  {100.0 * rms_drop:+.1f}%")
    if raw_ratio is not None and centred_ratio is not None:
        print(f"  separation gain:  {centred_ratio / raw_ratio:.3f}x")

    boundary_hit = (
        abs(centred_best.time_offset_ms - offsets[0]) < 1.0e-9
        or abs(centred_best.time_offset_ms - offsets[-1]) < 1.0e-9
    )
    if boundary_hit:
        print(
            "  timing warning:   centred best offset is on the scan boundary; "
            "extend the range before interpreting timing"
        )

    print()
    print("Interpretation")
    print(
        "  - the hard-iron centre was fitted from this same capture; this is an "
        "A/B observability test, not independent calibration validation"
    )
    print(
        "  - a large centred RMS reduction and/or stronger winner separation "
        "supports fixed additive bias as the cause of the ambiguous raw mapping"
    )
    print(
        "  - same-capture fitting can overstate performance; repeatability must "
        "be tested on an independent multi-axis capture before promotion"
    )
    print(
        "  - soft-iron distortion remains unmodelled, and the timing offset "
        "remains exploratory under protocol v0.1"
    )
    print(
        "  - do not promote a mag_sensor -> device_body mapping or heading "
        "calibration from this comparison alone"
    )

    return (
        0
        if raw_inspector._stream_clean(decoder, stats) and not gyro_usage.rejected
        else 1
    )


if __name__ == "__main__":
    raise SystemExit(main())
