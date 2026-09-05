"""Apply the frozen Phase 3 reference wall gate to one calibrated capture."""

from __future__ import annotations

import argparse
from pathlib import Path

import inspect_orientation_wall as inspect_wall
import rangeweave_imu_quality as imu_quality
import rangeweave_orientation as ori
import rangeweave_orientation_wall as wall
import rangeweave_phase3_gate as gate
import rangeweave_protocol as rw
import rangeweave_tof_timing as tof_timing


def _status(ok: bool) -> str:
    return "PASS" if ok else "FAIL"


def main() -> int:
    parser = argparse.ArgumentParser(
        description=(
            "Run the calibrated Rangeweave Phase 3 rotation-in-place wall acceptance gate"
        )
    )
    parser.add_argument("capture", help="capture directory or packets.bin")
    parser.add_argument("--boresight-artifact", required=True)
    parser.add_argument("--timing-artifact", required=True)
    parser.add_argument("--timing-assembly-id", required=True)
    args = parser.parse_args()

    try:
        (
            packets_path,
            samples,
            clock_syncs,
            tof_grids,
            decoder,
            stats,
        ) = inspect_wall._decode(Path(args.capture))
        ctrl1 = inspect_wall._config_byte(stats.last_info, rw.INFO_LSM_CTRL1_XL)
        ctrl2 = inspect_wall._config_byte(stats.last_info, rw.INFO_LSM_CTRL2_G)
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
        )
        boresight = inspect_wall._load_boresight(Path(args.boresight_artifact))
        if boresight.role != "calibrated":
            raise wall.WallOrientationError(
                "Phase 3 reference gate requires a calibrated ToF/body boresight artifact"
            )

        timing_artifact = tof_timing.load_artifact(Path(args.timing_artifact))
        timing = tof_timing.resolve_tof_timing(
            stats.last_info,
            tuple(stats.protocol_versions),
            mode=tof_timing.MODE_CALIBRATED,
            assembly_id=args.timing_assembly_id,
            artifact=timing_artifact,
        )
        if not timing.calibrated:
            raise wall.WallOrientationError(
                "Phase 3 reference gate requires calibrated ToF timing resolution"
            )

        result = wall.evaluate_wall_stability(
            tof_grids,
            samples,
            orientation_run,
            boresight.rotation_body_from_tof,
            tof_time_offset_ms=timing.effective_offset_ms,
        )
        assessment = gate.assess_wall_stability(result)
    except (
        OSError,
        ori.OrientationError,
        wall.WallOrientationError,
        imu_quality.ImuQualityError,
        tof_timing.TofTimingError,
        ValueError,
    ) as exc:
        parser.error(str(exc))

    health_nonzero = {
        name: delta for name, delta in stats.health_deltas().items() if int(delta) != 0
    }
    stream_ok = not decoder.frames_bad and not stats.sequence_gaps and not health_nonzero
    gyro_ok = not range_usage.rejected

    usable_ok = assessment.usable_fraction >= gate.MIN_USABLE_FRACTION
    excursion_ok = result.orientation_excursion_deg >= gate.MIN_ORIENTATION_EXCURSION_DEG
    rms_ok = result.residual_rms_deg <= gate.MAX_RESIDUAL_RMS_DEG
    p95_ok = result.residual_p95_deg <= gate.MAX_RESIDUAL_P95_DEG
    max_ok = result.residual_max_deg <= gate.MAX_RESIDUAL_MAX_DEG
    start_end_ok = result.start_end_error_deg <= gate.MAX_START_END_ERROR_DEG

    print("Rangeweave Phase 3 calibrated wall gate")
    print(f"  capture:          {packets_path}")
    print(f"  boresight role:   {boresight.role}")
    print(f"  timing role:      {timing.role}")
    print(f"  timing profile:   {timing.artifact_name}")
    print(f"  assembly id:      {args.timing_assembly_id}")
    print(f"  timing offset:    {timing.effective_offset_ms:+.3f} ms")
    print()
    print("Gate inputs")
    print(f"  stream health:    {_status(stream_ok)}")
    print(f"  gyro range:       {_status(gyro_ok)}")
    print(
        "  usable frames:    {:.1%} >= {:.1%}  {}".format(
            assessment.usable_fraction, gate.MIN_USABLE_FRACTION, _status(usable_ok)
        )
    )
    print(
        "  excursion:        {:.3f} >= {:.3f} deg  {}".format(
            result.orientation_excursion_deg,
            gate.MIN_ORIENTATION_EXCURSION_DEG,
            _status(excursion_ok),
        )
    )
    print(
        "  residual RMS:     {:.3f} <= {:.3f} deg  {}".format(
            result.residual_rms_deg, gate.MAX_RESIDUAL_RMS_DEG, _status(rms_ok)
        )
    )
    print(
        "  residual p95:     {:.3f} <= {:.3f} deg  {}".format(
            result.residual_p95_deg, gate.MAX_RESIDUAL_P95_DEG, _status(p95_ok)
        )
    )
    print(
        "  residual max:     {:.3f} <= {:.3f} deg  {}".format(
            result.residual_max_deg, gate.MAX_RESIDUAL_MAX_DEG, _status(max_ok)
        )
    )
    print(
        "  start/end delta:  {:.3f} <= {:.3f} deg  {}".format(
            result.start_end_error_deg,
            gate.MAX_START_END_ERROR_DEG,
            _status(start_end_ok),
        )
    )

    passed = stream_ok and gyro_ok and assessment.passed
    print()
    print("PHASE 3 WALL GATE: {}".format(_status(passed)))
    print(
        "Scope: empirical calibrated reference-path acceptance gate; not a universal "
        "sensor-model or cross-unit performance specification."
    )
    if assessment.failures:
        for failure in assessment.failures:
            print(f"  failure: {failure}")

    return 0 if passed else 1


if __name__ == "__main__":
    raise SystemExit(main())
