"""Validate persistent Rangeweave orientation against a static wall normal."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
import statistics

import rangeweave_capture as cap
import rangeweave_extrinsics as ext
import rangeweave_geometry as geometry
import rangeweave_imu_quality as imu_quality
import rangeweave_orientation as ori
import rangeweave_orientation_wall as wall
import rangeweave_protocol as rw


def _resolve_packets_path(path: Path) -> Path:
    if path.is_dir():
        path = path / cap.PACKETS_FILENAME
    if not path.is_file():
        raise wall.WallOrientationError(f"packets file not found: {path}")
    return path


def _decode(path: Path):
    packets_path = _resolve_packets_path(path)
    decoder = rw.StreamDecoder()
    stats = cap.StreamStats()
    samples = []
    clock_syncs = []
    tof_grids = []
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
                elif frame.record_type == rw.RECORD_TOF_GRID:
                    tof_grids.append(record)
    return (
        packets_path,
        tuple(samples),
        tuple(clock_syncs),
        tuple(tof_grids),
        decoder,
        stats,
    )


def _config_byte(info, tag):
    if info is None:
        return None
    value = info.first_value(tag)
    if not value or len(value) != 1:
        return None
    return value[0]


def _load_boresight(path: Path) -> ext.TofBodyRotation:
    try:
        document = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as exc:
        raise wall.WallOrientationError(f"could not read boresight artifact {path}: {exc}") from exc
    if not isinstance(document, dict):
        raise wall.WallOrientationError("boresight artifact root must be an object")
    try:
        return ext.TofBodyRotation.from_dict(document)
    except ext.ExtrinsicError as exc:
        raise wall.WallOrientationError(f"invalid boresight artifact: {exc}") from exc


def _fmt_vec(values, digits=5):
    fmt = "{:" + "+." + str(digits) + "f}"
    return "X {}  Y {}  Z {}".format(*(fmt.format(value) for value in values))


def _offset_values(min_ms: float, max_ms: float, step_ms: float):
    if step_ms <= 0.0:
        raise wall.WallOrientationError("--scan-offset-step-ms must be positive")
    if max_ms < min_ms:
        raise wall.WallOrientationError("--scan-offset-max-ms must be >= --scan-offset-min-ms")
    values = []
    value = min_ms
    while value <= max_ms + 1.0e-9:
        values.append(value)
        value += step_ms
    return tuple(values)


def main() -> int:
    parser = argparse.ArgumentParser(
        description=(
            "Replay persistent orientation and test whether a fixed physical wall "
            "keeps a stable normal in local_reference"
        )
    )
    parser.add_argument("capture", help="capture directory or packets.bin")
    parser.add_argument(
        "--boresight-artifact",
        required=True,
        help="per-device rangeweave.tof-body-rotation JSON artifact",
    )
    parser.add_argument(
        "--initialisation-seconds",
        type=float,
        default=ori.DEFAULT_INITIALISATION_SECONDS,
        help="stationary initial interval used by the orientation estimator",
    )
    parser.add_argument(
        "--gravity-gain",
        type=float,
        default=ori.DEFAULT_GRAVITY_GAIN_PER_S,
        help="proportional gravity-correction gain in 1/s",
    )
    parser.add_argument(
        "--tof-time-offset-ms",
        type=float,
        default=0.0,
        help=(
            "explicit offset added to protocol mcu_ready_us before attitude interpolation; "
            "default 0.0 ms"
        ),
    )
    parser.add_argument(
        "--start-end-window-seconds",
        type=float,
        default=3.0,
        help="duration of first/last usable wall-normal windows (default: 3.0)",
    )
    parser.add_argument(
        "--min-valid-zones",
        type=int,
        default=48,
        help="minimum valid zones for each fitted wall frame (default: 48)",
    )
    parser.add_argument(
        "--max-plane-rms-mm",
        type=float,
        default=10.0,
        help="per-frame plane RMS quality limit (default: 10 mm)",
    )
    parser.add_argument(
        "--max-plane-residual-mm",
        type=float,
        default=30.0,
        help="per-frame maximum plane residual quality limit (default: 30 mm)",
    )
    parser.add_argument(
        "--scan-time-offset",
        action="store_true",
        help=(
            "exploratorily scan explicit ToF timing offsets and print the best five by "
            "wall-normal RMS; diagnostic only, not an automatic calibration"
        ),
    )
    parser.add_argument("--scan-offset-min-ms", type=float, default=-80.0)
    parser.add_argument("--scan-offset-max-ms", type=float, default=20.0)
    parser.add_argument("--scan-offset-step-ms", type=float, default=2.0)
    args = parser.parse_args()

    try:
        (
            packets_path,
            samples,
            clock_syncs,
            tof_grids,
            decoder,
            stats,
        ) = _decode(Path(args.capture))
        ctrl1 = _config_byte(stats.last_info, rw.INFO_LSM_CTRL1_XL)
        ctrl2 = _config_byte(stats.last_info, rw.INFO_LSM_CTRL2_G)
        if ctrl1 is None or ctrl2 is None:
            raise wall.WallOrientationError(
                "capture is missing LSM CTRL1_XL/CTRL2_G STREAM_INFO metadata"
            )
        range_usage = imu_quality.analyse_gyro_range(samples, ctrl2_g=ctrl2)
        orientation_run = ori.estimate_orientation(
            samples,
            clock_syncs,
            ctrl1_xl=ctrl1,
            ctrl2_g=ctrl2,
            initialisation_seconds=args.initialisation_seconds,
            gravity_gain_per_s=args.gravity_gain,
        )
        boresight_path = Path(args.boresight_artifact)
        boresight = _load_boresight(boresight_path)
        result = wall.evaluate_wall_stability(
            tof_grids,
            samples,
            orientation_run,
            boresight.rotation_body_from_tof,
            tof_time_offset_ms=args.tof_time_offset_ms,
            min_valid_zones=args.min_valid_zones,
            max_plane_rms_mm=args.max_plane_rms_mm,
            max_plane_residual_mm=args.max_plane_residual_mm,
            start_end_window_s=args.start_end_window_seconds,
        )
        offset_scan = ()
        if args.scan_time_offset:
            offset_scan = wall.scan_time_offsets(
                tof_grids,
                samples,
                orientation_run,
                boresight.rotation_body_from_tof,
                _offset_values(
                    args.scan_offset_min_ms,
                    args.scan_offset_max_ms,
                    args.scan_offset_step_ms,
                ),
                min_valid_zones=args.min_valid_zones,
                max_plane_rms_mm=args.max_plane_rms_mm,
                max_plane_residual_mm=args.max_plane_residual_mm,
                start_end_window_s=args.start_end_window_seconds,
            )
    except (
        OSError,
        ori.OrientationError,
        wall.WallOrientationError,
        imu_quality.ImuQualityError,
    ) as exc:
        parser.error(str(exc))

    health_nonzero = {
        name: delta for name, delta in stats.health_deltas().items() if int(delta) != 0
    }
    weights = [sample.accel_weight for sample in orientation_run.samples]
    innovations = [sample.gravity_innovation_deg for sample in orientation_run.samples]
    plane_rms = [item.plane_rms_mm for item in result.observations]
    plane_max = [item.plane_max_abs_mm for item in result.observations]
    bracket_gaps_ms = [item.orientation_bracket_gap_s * 1000.0 for item in result.observations]

    print("Rangeweave continuous wall orientation validation")
    print(f"  capture:          {packets_path}")
    print(f"  IMU samples:      {len(orientation_run.samples)}")
    print(f"  ToF grids:        {len(tof_grids)}")
    print(f"  CLOCK_SYNC:       {orientation_run.clock_fit.observation_count}")
    print(f"  decoder bad:      {decoder.frames_bad}")
    print(f"  sequence gaps:    {stats.sequence_gaps}")
    print(
        "  health deltas:    {}".format(
            "all zero"
            if stats.health_deltas() and not health_nonzero
            else (str(health_nonzero) if health_nonzero else "not available")
        )
    )

    print()
    print("Calibration / frame contract")
    print(f"  boresight:        {boresight_path}")
    print(f"  artifact role:    {boresight.role}")
    print("  transform:        tof_optical -> device_body -> local_reference")
    print(f"  ToF geometry:     {geometry.GEOMETRY_PROFILE_ROLE} ({geometry.GEOMETRY_MODEL})")
    print("  ToF timestamp:    protocol mcu_ready_us")
    print(f"  explicit offset:  {result.tof_time_offset_ms:+.3f} ms")
    print(
        "  timing note:      exact VL53L5CX internal ranging instant is not exposed by v0.1"
    )

    print()
    print("Gyro / orientation diagnostics")
    print(f"  configured range: +/-{range_usage.full_scale_dps:.0f} deg/s")
    print(
        "  peak utilisation: X {:.1%}  Y {:.1%}  Z {:.1%}".format(
            *range_usage.peak_fraction
        )
    )
    print("  gyro status:      {}".format("REJECT" if range_usage.rejected else "PASS"))
    init = orientation_run.initialisation
    print(f"  init gyro spread: {init.gyro_spread_dps:.4f} deg/s")
    print(f"  init accel dev:   {init.accel_median_deviation_g:.4f} g")
    print(f"  gravity gain:     {orientation_run.gravity_gain_per_s:.3f} 1/s")
    print(f"  median weight:    {statistics.median(weights):.3f}")
    print(f"  zero-weight share:{sum(weight == 0.0 for weight in weights) / len(weights):8.1%}")
    print(f"  median innovation:{statistics.median(innovations):8.3f} deg")
    print(f"  max excursion:    {result.orientation_excursion_deg:.3f} deg from initial attitude")

    print()
    print("Usable wall observations")
    print(f"  accepted:         {len(result.observations)} / {result.total_tof_grids}")
    print(f"  missing distance: {result.rejected_missing_distance}")
    print(f"  wrong grid shape: {result.rejected_geometry_shape}")
    print(f"  plane quality:    {result.rejected_plane_quality}")
    print(f"  outside attitude: {result.rejected_outside_orientation}")
    print(f"  plane RMS median: {statistics.median(plane_rms):.3f} mm")
    print(f"  plane RMS max:    {max(plane_rms):.3f} mm")
    print(f"  plane resid max:  {max(plane_max):.3f} mm")
    print(f"  IMU bracket p95:  {wall.percentile(bracket_gaps_ms, 0.95):.3f} ms")

    print()
    print("Static-wall normal in local_reference")
    print("  mean normal:      " + _fmt_vec(result.reference_normal))
    print(f"  residual median:  {result.residual_median_deg:.3f} deg")
    print(f"  residual RMS:     {result.residual_rms_deg:.3f} deg")
    print(f"  residual p95:     {result.residual_p95_deg:.3f} deg")
    print(f"  residual max:     {result.residual_max_deg:.3f} deg")
    print(
        "  start window:     {} frames, {}".format(
            result.start_count, _fmt_vec(result.start_normal)
        )
    )
    print(
        "  end window:       {} frames, {}".format(
            result.end_count, _fmt_vec(result.end_normal)
        )
    )
    print(f"  start/end delta:  {result.start_end_error_deg:.3f} deg")

    if offset_scan:
        print()
        print("Exploratory ToF timing-offset scan")
        print("  diagnostic only; do not promote an offset from one capture")
        for candidate in offset_scan[:5]:
            print(
                "  offset {:+7.2f} ms -> RMS {:6.3f} deg, p95 {:6.3f}, max {:6.3f} (n={})".format(
                    candidate.offset_ms,
                    candidate.residual_rms_deg,
                    candidate.residual_p95_deg,
                    candidate.residual_max_deg,
                    candidate.observation_count,
                )
            )

    print()
    print(
        "Interpretation: no Phase 3 wall-normal acceptance threshold is frozen yet. "
        "Use this result as physical evidence and inspect timing/motion correlations before promotion."
    )

    failed = bool(decoder.frames_bad or stats.sequence_gaps or health_nonzero)
    failed = failed or range_usage.rejected
    return 1 if failed else 0


if __name__ == "__main__":
    raise SystemExit(main())
