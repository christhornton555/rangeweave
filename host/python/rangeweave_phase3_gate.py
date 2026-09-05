"""Empirical Phase 3 rotation-in-place wall acceptance gate.

These bounds are a project validation contract derived from the September 2026
reference-rig evidence. They are deliberately not presented as universal
VL53L5CX/IMU specifications or guaranteed cross-unit performance.
"""

from __future__ import annotations

from dataclasses import dataclass


MIN_USABLE_FRACTION = 0.95
MIN_ORIENTATION_EXCURSION_DEG = 20.0
MAX_RESIDUAL_RMS_DEG = 1.0
MAX_RESIDUAL_P95_DEG = 2.0
MAX_RESIDUAL_MAX_DEG = 5.0
MAX_START_END_ERROR_DEG = 1.0


@dataclass(frozen=True)
class Phase3WallGateAssessment:
    passed: bool
    usable_fraction: float
    failures: tuple[str, ...]


def assess_wall_stability(result: object) -> Phase3WallGateAssessment:
    """Assess one calibrated wall-normal run against the Phase 3 reference gate.

    The caller remains responsible for acquisition-health, gyro-range and
    calibration-role checks. This function evaluates only the geometric/motion
    evidence contained in a WallNormalStability-like result.
    """

    total = int(getattr(result, "total_tof_grids"))
    observations = tuple(getattr(result, "observations"))
    if total <= 0:
        raise ValueError("total_tof_grids must be positive")

    usable_fraction = len(observations) / total
    failures: list[str] = []

    checks = (
        (
            usable_fraction >= MIN_USABLE_FRACTION,
            "usable wall observations {:.1%} < {:.1%}".format(
                usable_fraction, MIN_USABLE_FRACTION
            ),
        ),
        (
            float(getattr(result, "orientation_excursion_deg"))
            >= MIN_ORIENTATION_EXCURSION_DEG,
            "orientation excursion {:.3f} deg < {:.3f} deg".format(
                float(getattr(result, "orientation_excursion_deg")),
                MIN_ORIENTATION_EXCURSION_DEG,
            ),
        ),
        (
            float(getattr(result, "residual_rms_deg")) <= MAX_RESIDUAL_RMS_DEG,
            "wall-normal RMS {:.3f} deg > {:.3f} deg".format(
                float(getattr(result, "residual_rms_deg")), MAX_RESIDUAL_RMS_DEG
            ),
        ),
        (
            float(getattr(result, "residual_p95_deg")) <= MAX_RESIDUAL_P95_DEG,
            "wall-normal p95 {:.3f} deg > {:.3f} deg".format(
                float(getattr(result, "residual_p95_deg")), MAX_RESIDUAL_P95_DEG
            ),
        ),
        (
            float(getattr(result, "residual_max_deg")) <= MAX_RESIDUAL_MAX_DEG,
            "wall-normal max {:.3f} deg > {:.3f} deg".format(
                float(getattr(result, "residual_max_deg")), MAX_RESIDUAL_MAX_DEG
            ),
        ),
        (
            float(getattr(result, "start_end_error_deg")) <= MAX_START_END_ERROR_DEG,
            "start/end wall-normal delta {:.3f} deg > {:.3f} deg".format(
                float(getattr(result, "start_end_error_deg")), MAX_START_END_ERROR_DEG
            ),
        ),
    )

    for passed, message in checks:
        if not passed:
            failures.append(message)

    return Phase3WallGateAssessment(
        passed=not failures,
        usable_fraction=usable_fraction,
        failures=tuple(failures),
    )
