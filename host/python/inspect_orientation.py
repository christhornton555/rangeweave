"""Replay persistent six-axis orientation estimation on a Rangeweave capture."""

from __future__ import annotations

import argparse
from pathlib import Path
import statistics

import rangeweave_capture as cap
import rangeweave_extrinsics as ext
import rangeweave_imu_quality as imu_quality
import rangeweave_imu_relative as imu
import rangeweave_orientation as ori
import rangeweave_protocol as rw


def _resolve_packets_path(path: Path) -> Path:
    if path.is_dir():
        path = path / cap.PACKETS_FILENAME
    if not path.is_file():
        raise ori.OrientationError(f"packets file not found: {path}")
    return path


def _decode(path: Path):
    packets_path = _resolve_packets_path(path)
    decoder = rw.StreamDecoder()
    stats = cap.StreamStats()
    samples = []
    clock_syncs = []
    with packets_path.open("rb") as handle:
        while True:
            chunk = handle.read(4096)
            if not chunk:
                break
            for frame in decoder.feed(chunk):
                stats.consume(frame)
                try:
                    record = rw.decode_record(frame)
                except rw.ProtocolError:
                    continue
                if frame.record_type == rw.RECORD_IMU_BATCH:
                    samples.extend(record.samples)
                elif frame.record_type == rw.RECORD_CLOCK_SYNC:
                    clock_syncs.append(record)
    return packets_path, tuple(samples), tuple(clock_syncs), decoder, stats


def _config_byte(info, tag):
    if info is None:
        return None
    value = info.first_value(tag)
    if not value or len(value) != 1:
        return None
    return value[0]


def _fmt_vec(values, digits=4):
    fmt = "{:" + "+." + str(digits) + "f}"
    return "X {}  Y {}  Z {}".format(*(fmt.format(value) for value in values))


def _rotation_difference_deg(left: ext.Matrix3, right: ext.Matrix3) -> float:
    difference = ext.matrix_multiply(ext.transpose(left), right)
    return ext.rotation_angle_deg(difference)


def main() -> int:
    parser = argparse.ArgumentParser(
        description=(
            "Replay the Rangeweave Phase 3 gyro+gravity orientation estimator on "
            "one canonical capture"
        )
    )
    parser.add_argument("capture", help="capture directory or packets.bin")
    parser.add_argument(
        "--initialisation-seconds",
        type=float,
        default=ori.DEFAULT_INITIALISATION_SECONDS,
        help="stationary initial interval used for gravity/yaw-zero and fixed gyro bias",
    )
    parser.add_argument(
        "--gravity-gain",
        type=float,
        default=ori.DEFAULT_GRAVITY_GAIN_PER_S,
        help="proportional gravity-correction gain in 1/s",
    )
    parser.add_argument(
        "--compare-relative",
        action="store_true",
        help=(
            "also run the validated hold-move-hold relative-rotation estimator and "
            "report the angular difference; intended for boresight motion captures"
        ),
    )
    args = parser.parse_args()

    try:
        packets_path, samples, clock_syncs, decoder, stats = _decode(Path(args.capture))
        ctrl1 = _config_byte(stats.last_info, rw.INFO_LSM_CTRL1_XL)
        ctrl2 = _config_byte(stats.last_info, rw.INFO_LSM_CTRL2_G)
        if ctrl1 is None or ctrl2 is None:
            raise ori.OrientationError(
                "capture is missing LSM CTRL1_XL/CTRL2_G STREAM_INFO metadata"
            )
        range_usage = imu_quality.analyse_gyro_range(samples, ctrl2_g=ctrl2)
        result = ori.estimate_orientation(
            samples,
            clock_syncs,
            ctrl1_xl=ctrl1,
            ctrl2_g=ctrl2,
            initialisation_seconds=args.initialisation_seconds,
            gravity_gain_per_s=args.gravity_gain,
        )
        relative = None
        relative_difference = None
        if args.compare_relative:
            relative = imu.estimate_relative_body_rotation(
                samples,
                clock_syncs,
                ctrl1_xl=ctrl1,
                ctrl2_g=ctrl2,
            )
            persistent_relative = ext.matrix_multiply(
                ext.transpose(result.samples[0].reference_from_body),
                result.samples[-1].reference_from_body,
            )
            relative_difference = _rotation_difference_deg(
                relative.reference_from_body,
                persistent_relative,
            )
    except (
        OSError,
        ori.OrientationError,
        imu.RelativeRotationError,
        imu_quality.ImuQualityError,
    ) as exc:
        parser.error(str(exc))

    health_nonzero = {
        name: delta for name, delta in stats.health_deltas().items() if int(delta) != 0
    }
    weights = [sample.accel_weight for sample in result.samples]
    innovations = [sample.gravity_innovation_deg for sample in result.samples]
    final = result.samples[-1]

    print("Rangeweave persistent orientation replay")
    print(f"  capture:          {packets_path}")
    print(f"  IMU samples:      {len(result.samples)}")
    print(f"  CLOCK_SYNC:       {result.clock_fit.observation_count}")
    print(f"  decoder bad:      {decoder.frames_bad}")
    print(f"  sequence gaps:    {stats.sequence_gaps}")
    print(
        "  health deltas:    {}".format(
            "all zero"
            if stats.health_deltas() and not health_nonzero
            else (str(health_nonzero) if health_nonzero else "not available")
        )
    )
    print(f"  IMU mapping:      {result.imu_mapping_role}")
    print("  attitude:         local_reference_from_body")
    print("  quaternion:       scalar-first Hamilton (w, x, y, z)")
    print("  yaw reference:    local initial heading only; not globally observed")

    print()
    print("Gyro range utilisation")
    print(f"  configured range: +/-{range_usage.full_scale_dps:.0f} deg/s")
    print(
        "  peak utilisation: X {:.1%}  Y {:.1%}  Z {:.1%}".format(
            *range_usage.peak_fraction
        )
    )
    print("  status:           {}".format("REJECT" if range_usage.rejected else "PASS"))

    init = result.initialisation
    print()
    print("Initial stationary state")
    print(f"  samples:          {init.sample_count}")
    print(f"  duration:         {init.duration_s:.3f} s")
    print("  gyro bias dps:    " + _fmt_vec(init.gyro_bias_body_dps))
    print(f"  gyro spread:      {init.gyro_spread_dps:.4f} deg/s")
    print("  accel median g:   " + _fmt_vec(init.accel_median_body_g))
    print(f"  accel norm:       {init.accel_norm_g:.4f} g")
    print(f"  accel deviation:  {init.accel_median_deviation_g:.4f} g")

    print()
    print("Gravity correction diagnostics")
    print(f"  gain:             {result.gravity_gain_per_s:.3f} 1/s")
    print(f"  median weight:    {statistics.median(weights):.3f}")
    print(f"  zero-weight share:{sum(weight == 0.0 for weight in weights) / len(weights):8.1%}")
    print(f"  median innovation:{statistics.median(innovations):8.3f} deg")
    print(f"  final innovation: {final.gravity_innovation_deg:.3f} deg")

    print()
    print("Final attitude")
    q = final.quaternion_reference_from_body
    print("  q (w,x,y,z):      {:+.8f} {:+.8f} {:+.8f} {:+.8f}".format(*q))
    print("  matrix:")
    for row in final.reference_from_body:
        print("                   [{:+.8f} {:+.8f} {:+.8f}]".format(*row))

    if relative is not None and relative_difference is not None:
        print()
        print("Hold-move-hold regression comparison")
        print(f"  relative angle:   {relative.rotation_angle_deg:.3f} deg")
        print(f"  gravity closure:  {relative.gravity_closure_error_deg:.3f} deg")
        print(f"  estimator delta:  {relative_difference:.3f} deg")
        print(
            "  note: no Phase 3 acceptance threshold is frozen yet; this is regression evidence."
        )

    print()
    print(
        "Interpretation: this is a gravity-referenced local orientation replay. "
        "Yaw is propagated by gyro but is not absolutely observed."
    )

    failed = bool(decoder.frames_bad or stats.sequence_gaps or health_nonzero)
    failed = failed or range_usage.rejected
    return 1 if failed else 0


if __name__ == "__main__":
    raise SystemExit(main())
