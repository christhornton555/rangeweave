"""Held-out validation for fixed-plane ToF/body boresight fits.

The training fit uses only the supplied training ToF observations.  A held-out
pose contributes its independently measured ``reference_from_body`` rotation to
predict what fixed-wall normal the ToF should observe; its actual ToF normal is
used only for the final prediction-error calculation and the optional all-pose
refit.
"""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Sequence

import rangeweave_extrinsics as ext


@dataclass(frozen=True)
class BoresightHoldoutEvaluation:
    training_fit: ext.BoresightFit
    predicted_holdout_tof_normal: ext.Vector3
    observed_holdout_tof_normal: ext.Vector3
    holdout_normal_error_deg: float
    all_pose_fit: ext.BoresightFit
    fit_parameter_delta_deg: ext.Vector3
    fit_rotation_change_deg: float


def _normalise(vector: Sequence[float]) -> ext.Vector3:
    values = tuple(float(value) for value in vector)
    if len(values) != 3 or not all(math.isfinite(value) for value in values):
        raise ext.ExtrinsicError("normal must contain three finite values")
    length = math.sqrt(sum(value * value for value in values))
    if length <= 0.0:
        raise ext.ExtrinsicError("normal must be non-zero")
    return tuple(value / length for value in values)  # type: ignore[return-value]


def normal_angle_deg(a: Sequence[float], b: Sequence[float]) -> float:
    """Return the directed-normal angular separation in degrees."""

    left = _normalise(a)
    right = _normalise(b)
    dot = max(-1.0, min(1.0, sum(left[i] * right[i] for i in range(3))))
    return math.degrees(math.acos(dot))


def predict_holdout_tof_normal(
    *,
    reference_plane_normal: Sequence[float],
    rotation_body_from_tof: ext.Matrix3,
    reference_from_body: ext.Matrix3,
) -> ext.Vector3:
    """Predict the ToF-frame fixed-wall normal for one independently placed body pose.

    The boresight model is

        n_ref = R_reference_from_body * R_body_from_tof * n_tof

    so the held-out ToF normal is predicted by applying the inverse rotations to
    the common reference-frame wall normal.  No held-out ToF measurement enters
    this prediction.
    """

    normal_ref = _normalise(reference_plane_normal)
    body_normal = ext.matrix_vector(ext.transpose(reference_from_body), normal_ref)
    tof_normal = ext.matrix_vector(ext.transpose(rotation_body_from_tof), body_normal)
    return _normalise(tof_normal)


def rotation_difference_deg(left: ext.Matrix3, right: ext.Matrix3) -> float:
    """Return the geodesic angular difference between two rotation matrices."""

    relative = ext.matrix_multiply(left, ext.transpose(right))
    return ext.rotation_angle_deg(relative)


def evaluate_holdout(
    training_observations: Sequence[ext.FixedPlaneBoresightObservation],
    holdout_observation: ext.FixedPlaneBoresightObservation,
    *,
    max_abs_angle_deg: float = 20.0,
) -> BoresightHoldoutEvaluation:
    """Fit training observations, predict one held-out ToF normal, then refit all.

    The held-out observation's body orientation is an independent IMU-derived
    input.  Its ToF normal is excluded from ``training_fit`` and is revealed only
    when computing ``holdout_normal_error_deg`` and ``all_pose_fit``.
    """

    training = tuple(training_observations)
    if len(training) < 4:
        raise ext.ExtrinsicError("held-out validation requires at least four training poses")

    training_fit = ext.solve_fixed_plane_boresight(
        training,
        max_abs_angle_deg=max_abs_angle_deg,
    )
    predicted = predict_holdout_tof_normal(
        reference_plane_normal=training_fit.reference_plane_normal,
        rotation_body_from_tof=training_fit.extrinsic.rotation_body_from_tof,
        reference_from_body=holdout_observation.reference_from_body,
    )
    observed = _normalise(holdout_observation.tof_plane_normal)
    holdout_error = normal_angle_deg(predicted, observed)

    all_pose_fit = ext.solve_fixed_plane_boresight(
        training + (holdout_observation,),
        max_abs_angle_deg=max_abs_angle_deg,
    )
    parameter_delta = (
        all_pose_fit.rotation_x_deg - training_fit.rotation_x_deg,
        all_pose_fit.rotation_y_deg - training_fit.rotation_y_deg,
        all_pose_fit.rotation_z_deg - training_fit.rotation_z_deg,
    )
    rotation_change = rotation_difference_deg(
        all_pose_fit.extrinsic.rotation_body_from_tof,
        training_fit.extrinsic.rotation_body_from_tof,
    )

    return BoresightHoldoutEvaluation(
        training_fit=training_fit,
        predicted_holdout_tof_normal=predicted,
        observed_holdout_tof_normal=observed,
        holdout_normal_error_deg=holdout_error,
        all_pose_fit=all_pose_fit,
        fit_parameter_delta_deg=parameter_delta,
        fit_rotation_change_deg=rotation_change,
    )
