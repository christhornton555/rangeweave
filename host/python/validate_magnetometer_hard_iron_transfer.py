"""Validate hard-iron transfer and magnetometer/body mapping across two captures.

The first capture is treated as calibration evidence: fit a native ``mag_sensor``
hard-iron centre there and transfer that centre unchanged to the second capture.
The second capture is also self-fitted as a diagnostic upper bound only.

This tool does not promote a calibration or mapping artifact.  Its purpose is to
separate same-capture overfit from repeatable assembly-fixed magnetic behaviour.
"""

from __future__ import annotations

import argparse
import math
from pathlib import Path

import compare_magnetometer_mapping_hard_iron as ab
import inspect_magnetometer_mapping as raw_inspector
import rangeweave_imu_quality as imu_quality
import rangeweave_magnetometer as magnetometer
import rangeweave_magnetometer_calibration as calibration
import rangeweave_magnetometer_mapping as mapping
import rangeweave_orientation as orientation
import rangeweave_protocol as rw


def centre_delta_ut(
    calibration_centre: tuple[float, float, float],
    validation_centre: tuple[float, float, float],
) -> tuple[tuple[float, float, float], float]:
    if len(calibration_centre) != 3 or len(validation_centre) != 3:
        raise ValueError("hard-iron centres must contain exactly three components")
    delta = tuple(
        float(validation_centre[i]) - float(calibration_centre[i])
        for i in range(3)
    )
    magnitude = math.sqrt(sum(value * value for value in delta))
    return delta, magnitude


def _decode_and_prepare(path: Path):
    packets, imu_samples, clock_syncs, raw_mag, decoder, stats = raw_inspector._decode(path)
    ctrl1 = raw_inspector._config_byte(stats.last_info, rw.INFO_LSM_CTRL1_XL)
    ctrl2 = raw_inspector._config_byte(stats.last_info, rw.INFO_LSM_CTRL2_G)
    if ctrl1 is None or ctrl2 is None:
        raise mapping.MagnetometerMappingError(
            f"{packets}: capture is missing LSM CTRL1_XL/CTRL2_G STREAM_INFO metadata"
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
    return {
        "packets": packets,
        "imu_samples": imu_samples,
        "clock_syncs": clock_syncs,
        "decoder": decoder,
        "stats": stats,
        "config": config,
        "mag_samples": mag_samples,
        "mag_summary": mag_summary,
        "gyro_usage": gyro_usage,
        "orientation_run": orientation_run,
    }


def _distinct(samples, prepared, offsets):
    scores = mapping.scan_signed_permutation_mappings(
        samples,
        prepared["imu_samples"],
        prepared["orientation_run"],
        offsets,
    )
    return mapping.best_distinct_mappings(scores)


def _print_short_ranking(title: str, distinct: tuple[mapping.MappingScore, ...]) -> None:
    print(title)
    print("  rank  mapping                                             offset    RMS    p95    max   vecRMS  n")
    for index, score in enumerate(distinct[:3], start=1):
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


def _ratio(distinct: tuple[mapping.MappingScore, ...]) -> float | None:
    return ab._second_ratio(distinct)


def _span(summary: magnetometer.MagneticSummary) -> float:
    return summary.norm_p95_ut - summary.norm_p05_ut


def main() -> int:
    parser = argparse.ArgumentParser(
        description=(
            "Fit hard iron on one capture and validate transferred correction plus "
            "body-axis mapping on an independent capture"
        )
    )
    parser.add_argument("calibration_capture", help="M1 calibration capture directory or packets.bin")
    parser.add_argument("validation_capture", help="independent M2 validation capture directory or packets.bin")
    parser.add_argument("--offset-min-ms", type=float, default=-60.0)
    parser.add_argument("--offset-max-ms", type=float, default=0.0)
    parser.add_argument("--offset-step-ms", type=float, default=2.0)
    args = parser.parse_args()

    try:
        calibration_run = _decode_and_prepare(Path(args.calibration_capture))
        validation_run = _decode_and_prepare(Path(args.validation_capture))
        offsets = raw_inspector._offsets(
            args.offset_min_ms,
            args.offset_max_ms,
            args.offset_step_ms,
        )

        calibration_samples = calibration_run["mag_samples"]
        validation_samples = validation_run["mag_samples"]

        calibration_fit = calibration.fit_hard_iron_sphere(
            (sample.x_ut, sample.y_ut, sample.z_ut) for sample in calibration_samples
        )
        validation_fit = calibration.fit_hard_iron_sphere(
            (sample.x_ut, sample.y_ut, sample.z_ut) for sample in validation_samples
        )
        delta_components, delta_magnitude = centre_delta_ut(
            calibration_fit.center_ut,
            validation_fit.center_ut,
        )

        calibration_centred = ab.centre_samples(
            calibration_samples,
            calibration_fit.center_ut,
        )
        validation_cross_centred = ab.centre_samples(
            validation_samples,
            calibration_fit.center_ut,
        )
        validation_self_centred = ab.centre_samples(
            validation_samples,
            validation_fit.center_ut,
        )

        calibration_centred_summary = magnetometer.summarise(
            calibration_centred, calibration_run["config"]
        )
        validation_cross_summary = magnetometer.summarise(
            validation_cross_centred, validation_run["config"]
        )
        validation_self_summary = magnetometer.summarise(
            validation_self_centred, validation_run["config"]
        )

        calibration_mapping = _distinct(calibration_centred, calibration_run, offsets)
        validation_raw_mapping = _distinct(validation_samples, validation_run, offsets)
        validation_cross_mapping = _distinct(validation_cross_centred, validation_run, offsets)
        validation_self_mapping = _distinct(validation_self_centred, validation_run, offsets)
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

    cal_best = calibration_mapping[0]
    val_raw_best = validation_raw_mapping[0]
    val_cross_best = validation_cross_mapping[0]
    val_self_best = validation_self_mapping[0]
    cal_ratio = _ratio(calibration_mapping)
    cross_ratio = _ratio(validation_cross_mapping)
    self_ratio = _ratio(validation_self_mapping)

    calibration_clean = raw_inspector._stream_clean(
        calibration_run["decoder"], calibration_run["stats"]
    )
    validation_clean = raw_inspector._stream_clean(
        validation_run["decoder"], validation_run["stats"]
    )

    print("Rangeweave independent hard-iron transfer / mapping validation")
    print(f"  calibration:      {calibration_run['packets']}")
    print(f"  validation:       {validation_run['packets']}")
    print(f"  calibration health: {'PASS' if calibration_clean else 'NOT CLEAN'}")
    print(f"  validation health:  {'PASS' if validation_clean else 'NOT CLEAN'}")
    print(
        "  calibration motion: {:.3f} deg; MAG {} samples".format(
            raw_inspector._orientation_excursion_deg(calibration_run["orientation_run"]),
            len(calibration_samples),
        )
    )
    print(
        "  validation motion:  {:.3f} deg; MAG {} samples".format(
            raw_inspector._orientation_excursion_deg(validation_run["orientation_run"]),
            len(validation_samples),
        )
    )
    print(
        "  calibration gyro: {} (peak X {:.1f}% Y {:.1f}% Z {:.1f}%)".format(
            "PASS" if not calibration_run["gyro_usage"].rejected else "FAIL",
            100.0 * calibration_run["gyro_usage"].peak_fraction[0],
            100.0 * calibration_run["gyro_usage"].peak_fraction[1],
            100.0 * calibration_run["gyro_usage"].peak_fraction[2],
        )
    )
    print(
        "  validation gyro:  {} (peak X {:.1f}% Y {:.1f}% Z {:.1f}%)".format(
            "PASS" if not validation_run["gyro_usage"].rejected else "FAIL",
            100.0 * validation_run["gyro_usage"].peak_fraction[0],
            100.0 * validation_run["gyro_usage"].peak_fraction[1],
            100.0 * validation_run["gyro_usage"].peak_fraction[2],
        )
    )

    print()
    print("Independent hard-iron fit repeatability")
    print(
        "  calibration centre: X {:+.3f}  Y {:+.3f}  Z {:+.3f} uT".format(
            *calibration_fit.center_ut
        )
    )
    print(
        "  validation centre:  X {:+.3f}  Y {:+.3f}  Z {:+.3f} uT".format(
            *validation_fit.center_ut
        )
    )
    print(
        "  validation-cal delta:X {:+.3f}  Y {:+.3f}  Z {:+.3f} uT".format(
            *delta_components
        )
    )
    print(f"  |centre delta|:    {delta_magnitude:.3f} uT")
    if calibration_fit.radius_ut > 1.0e-12:
        print(
            f"  delta / field:     {100.0 * delta_magnitude / calibration_fit.radius_ut:.2f}%"
        )
    print(
        f"  fitted radii:      cal {calibration_fit.radius_ut:.3f} uT; "
        f"validation {validation_fit.radius_ut:.3f} uT; "
        f"delta {validation_fit.radius_ut - calibration_fit.radius_ut:+.3f} uT"
    )
    print(
        f"  radial RMS:        cal {calibration_fit.radial_residual_rms_ut:.3f} uT; "
        f"validation {validation_fit.radial_residual_rms_ut:.3f} uT"
    )

    print()
    print("Validation field-magnitude spread")
    print(
        f"  raw validation:    {_span(validation_run['mag_summary']):.3f} uT "
        "(p05 to p95)"
    )
    print(
        f"  M1-centre applied: {_span(validation_cross_summary):.3f} uT "
        "(independent transfer)"
    )
    print(
        f"  M2 self-fit:       {_span(validation_self_summary):.3f} uT "
        "(diagnostic upper bound)"
    )
    print(
        f"  calibration self:  {_span(calibration_centred_summary):.3f} uT"
    )

    print()
    print(
        f"Timing scan: {offsets[0]:+.1f} .. {offsets[-1]:+.1f} ms "
        f"in {args.offset_step_ms:.1f} ms steps"
    )
    print()
    _print_short_ranking("Calibration capture, self-centred", calibration_mapping)
    print()
    _print_short_ranking("Validation capture, raw", validation_raw_mapping)
    print()
    _print_short_ranking(
        "Validation capture, M1 centre transferred unchanged",
        validation_cross_mapping,
    )
    print()
    _print_short_ranking("Validation capture, M2 self-centred", validation_self_mapping)

    print()
    print("Cross-capture consistency summary")
    print(f"  calibration winner: {cal_best.mapping.name}")
    print(f"  transferred winner: {val_cross_best.mapping.name}")
    print(f"  self-fit winner:     {val_self_best.mapping.name}")
    print(
        "  cal == transferred: "
        + ("YES" if cal_best.mapping.name == val_cross_best.mapping.name else "NO")
    )
    print(
        "  transferred == self:"
        + (" YES" if val_cross_best.mapping.name == val_self_best.mapping.name else " NO")
    )
    print(
        f"  RMS cal/cross/self: {cal_best.direction_rms_deg:.3f} / "
        f"{val_cross_best.direction_rms_deg:.3f} / "
        f"{val_self_best.direction_rms_deg:.3f} deg"
    )
    if cal_ratio is not None and cross_ratio is not None and self_ratio is not None:
        print(
            f"  2nd/best ratios:    {cal_ratio:.3f}x / {cross_ratio:.3f}x / "
            f"{self_ratio:.3f}x"
        )
    print(
        f"  offsets cal/cross/self: {cal_best.time_offset_ms:+.1f} / "
        f"{val_cross_best.time_offset_ms:+.1f} / "
        f"{val_self_best.time_offset_ms:+.1f} ms"
    )
    print(
        f"  validation raw RMS: {val_raw_best.direction_rms_deg:.3f} deg"
    )

    print()
    print("Interpretation")
    print(
        "  - the transferred arm is the key independent test: its M1 hard-iron "
        "centre is applied to M2 without refitting"
    )
    print(
        "  - agreement between calibration and transferred mapping winners, with "
        "low RMS and strong candidate separation, supports a repeatable physical mapping"
    )
    print(
        "  - a small independent centre delta supports assembly-fixed hard iron; "
        "a large delta points to environment/setup change, fit instability, or non-fixed distortion"
    )
    print(
        "  - the M2 self-fit arm is diagnostic only; better self-fit performance "
        "than transferred performance quantifies how much capture-specific correction remains"
    )
    print(
        "  - soft-iron distortion and magnetic timing remain unpromoted; no heading "
        "or calibration artifact should be created from this tool alone"
    )

    return (
        0
        if calibration_clean
        and validation_clean
        and not calibration_run["gyro_usage"].rejected
        and not validation_run["gyro_usage"].rejected
        else 1
    )


if __name__ == "__main__":
    raise SystemExit(main())
