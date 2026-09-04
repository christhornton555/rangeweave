"""Rotational extrinsics between Rangeweave ``tof_optical`` and ``device_body``.

The v1 artifact deliberately contains rotation only. Translation between sensor
origins is a separate later calibration problem.

The fitted boresight path is designed for a fixed physical plane observed from
multiple stationary device poses. Relative body rotations come from a separate
source (eventually the validated IMU/body mapping); the plane's absolute room
orientation need not be known.
"""

from __future__ import annotations

from dataclasses import dataclass, field
import math
from typing import Any, Mapping, Sequence


EXTRINSIC_SCHEMA = "rangeweave.tof-body-rotation"
EXTRINSIC_SCHEMA_VERSION = 1
BORESIGHT_METHOD = "fixed-plane-relative-body-rotations-coordinate-search-v1"

Matrix3 = tuple[
    tuple[float, float, float],
    tuple[float, float, float],
    tuple[float, float, float],
]
Vector3 = tuple[float, float, float]


class ExtrinsicError(ValueError):
    """Raised when a rotational extrinsic or boresight fit is invalid."""


def identity_matrix() -> Matrix3:
    return (
        (1.0, 0.0, 0.0),
        (0.0, 1.0, 0.0),
        (0.0, 0.0, 1.0),
    )


def matrix_multiply(left: Matrix3, right: Matrix3) -> Matrix3:
    return tuple(
        tuple(
            sum(left[row][index] * right[index][col] for index in range(3))
            for col in range(3)
        )
        for row in range(3)
    )  # type: ignore[return-value]


def matrix_vector(matrix: Matrix3, vector: Vector3) -> Vector3:
    return tuple(
        sum(matrix[row][col] * vector[col] for col in range(3))
        for row in range(3)
    )  # type: ignore[return-value]


def transpose(matrix: Matrix3) -> Matrix3:
    return tuple(
        tuple(matrix[col][row] for col in range(3))
        for row in range(3)
    )  # type: ignore[return-value]


def _vector3(vector: Sequence[float]) -> Vector3:
    values = tuple(float(value) for value in vector)
    if len(values) != 3 or not all(math.isfinite(value) for value in values):
        raise ExtrinsicError("vector must contain three finite values")
    return values  # type: ignore[return-value]


def _normalise(vector: Sequence[float]) -> Vector3:
    values = _vector3(vector)
    length = math.sqrt(sum(value * value for value in values))
    if length <= 0.0:
        raise ExtrinsicError("vector must be non-zero")
    return tuple(value / length for value in values)  # type: ignore[return-value]


def _determinant(matrix: Matrix3) -> float:
    return (
        matrix[0][0] * (matrix[1][1] * matrix[2][2] - matrix[1][2] * matrix[2][1])
        - matrix[0][1] * (matrix[1][0] * matrix[2][2] - matrix[1][2] * matrix[2][0])
        + matrix[0][2] * (matrix[1][0] * matrix[2][1] - matrix[1][1] * matrix[2][0])
    )


def _validated_rotation(matrix: Sequence[Sequence[float]]) -> Matrix3:
    if len(matrix) != 3 or any(len(row) != 3 for row in matrix):
        raise ExtrinsicError("rotation must be a 3x3 matrix")
    result: Matrix3 = tuple(
        tuple(float(value) for value in row)
        for row in matrix
    )  # type: ignore[assignment]
    if not all(math.isfinite(value) for row in result for value in row):
        raise ExtrinsicError("rotation matrix entries must be finite")

    product = matrix_multiply(result, transpose(result))
    max_error = max(
        abs(product[row][col] - (1.0 if row == col else 0.0))
        for row in range(3)
        for col in range(3)
    )
    if max_error > 1.0e-6 or abs(_determinant(result) - 1.0) > 1.0e-6:
        raise ExtrinsicError("rotation matrix must be orthonormal with determinant +1")
    return result


def rotation_x_deg(angle_deg: float) -> Matrix3:
    angle = math.radians(float(angle_deg))
    c, s = math.cos(angle), math.sin(angle)
    return ((1.0, 0.0, 0.0), (0.0, c, -s), (0.0, s, c))


def rotation_y_deg(angle_deg: float) -> Matrix3:
    angle = math.radians(float(angle_deg))
    c, s = math.cos(angle), math.sin(angle)
    return ((c, 0.0, s), (0.0, 1.0, 0.0), (-s, 0.0, c))


def rotation_z_deg(angle_deg: float) -> Matrix3:
    angle = math.radians(float(angle_deg))
    c, s = math.cos(angle), math.sin(angle)
    return ((c, -s, 0.0), (s, c, 0.0), (0.0, 0.0, 1.0))


def rotation_xyz_deg(rx_deg: float, ry_deg: float, rz_deg: float) -> Matrix3:
    """Active fixed-axis rotations applied X, then Y, then Z.

    For a column vector this is ``Rz * Ry * Rx * v``.
    """

    return matrix_multiply(
        rotation_z_deg(rz_deg),
        matrix_multiply(rotation_y_deg(ry_deg), rotation_x_deg(rx_deg)),
    )


def rotation_angle_deg(matrix: Matrix3) -> float:
    value = (matrix[0][0] + matrix[1][1] + matrix[2][2] - 1.0) / 2.0
    return math.degrees(math.acos(max(-1.0, min(1.0, value))))


@dataclass(frozen=True)
class TofBodyRotation:
    name: str
    role: str
    rotation_body_from_tof: Matrix3
    metadata: Mapping[str, Any] = field(default_factory=dict)

    def __post_init__(self) -> None:
        if not self.name or not self.role:
            raise ExtrinsicError("extrinsic name and role must be non-empty")
        object.__setattr__(
            self,
            "rotation_body_from_tof",
            _validated_rotation(self.rotation_body_from_tof),
        )
        object.__setattr__(self, "metadata", dict(self.metadata))

    def tof_vector_to_body(self, vector: Sequence[float]) -> Vector3:
        return matrix_vector(self.rotation_body_from_tof, _vector3(vector))

    def body_vector_to_tof(self, vector: Sequence[float]) -> Vector3:
        return matrix_vector(transpose(self.rotation_body_from_tof), _vector3(vector))

    def to_dict(self) -> dict[str, Any]:
        return {
            "schema": EXTRINSIC_SCHEMA,
            "schema_version": EXTRINSIC_SCHEMA_VERSION,
            "name": self.name,
            "role": self.role,
            "from_frame": "tof_optical",
            "to_frame": "device_body",
            "rotation_body_from_tof": [
                list(row) for row in self.rotation_body_from_tof
            ],
            "metadata": dict(self.metadata),
        }

    @classmethod
    def from_dict(cls, document: Mapping[str, Any]) -> "TofBodyRotation":
        if document.get("schema") != EXTRINSIC_SCHEMA:
            raise ExtrinsicError("unexpected ToF/body extrinsic schema")
        if document.get("schema_version") != EXTRINSIC_SCHEMA_VERSION:
            raise ExtrinsicError("unsupported ToF/body extrinsic schema version")
        if document.get("from_frame") != "tof_optical":
            raise ExtrinsicError("extrinsic from_frame must be 'tof_optical'")
        if document.get("to_frame") != "device_body":
            raise ExtrinsicError("extrinsic to_frame must be 'device_body'")
        metadata = document.get("metadata", {})
        if not isinstance(metadata, Mapping):
            raise ExtrinsicError("extrinsic metadata must be an object")
        try:
            return cls(
                name=str(document["name"]),
                role=str(document["role"]),
                rotation_body_from_tof=document["rotation_body_from_tof"],
                metadata=dict(metadata),
            )
        except (KeyError, TypeError, ValueError) as exc:
            raise ExtrinsicError("invalid ToF/body extrinsic document") from exc


NOMINAL_TOF_BODY_ROTATION = TofBodyRotation(
    name="nominal-tof-body-alignment",
    role="nominal-fallback",
    rotation_body_from_tof=identity_matrix(),
    metadata={
        "note": (
            "Nominal device_body axes are parallel to tof_optical. "
            "Assembly/package boresight error is not assumed to be zero after calibration."
        )
    },
)


@dataclass(frozen=True)
class FixedPlaneBoresightObservation:
    """One stationary fixed-plane observation.

    ``reference_from_body`` maps vectors expressed in the current ``device_body``
    frame into an arbitrary reference-body frame. The first pose is a natural
    reference, but the solver does not require its matrix to be identity.
    """

    tof_plane_normal: Vector3
    reference_from_body: Matrix3

    def __post_init__(self) -> None:
        object.__setattr__(self, "tof_plane_normal", _normalise(self.tof_plane_normal))
        object.__setattr__(
            self, "reference_from_body", _validated_rotation(self.reference_from_body)
        )


@dataclass(frozen=True)
class BoresightFit:
    extrinsic: TofBodyRotation
    reference_plane_normal: Vector3
    rotation_x_deg: float
    rotation_y_deg: float
    rotation_z_deg: float
    rms_normal_error_deg: float
    max_normal_error_deg: float
    observability_cost_increase_at_1deg: Vector3


def _reference_normals(
    observations: Sequence[FixedPlaneBoresightObservation],
    rotation_body_from_tof: Matrix3,
) -> tuple[Vector3, ...]:
    return tuple(
        _normalise(
            matrix_vector(
                observation.reference_from_body,
                matrix_vector(rotation_body_from_tof, observation.tof_plane_normal),
            )
        )
        for observation in observations
    )


def _mean_direction(vectors: Sequence[Vector3]) -> Vector3:
    return _normalise(
        tuple(sum(vector[index] for vector in vectors) for index in range(3))
    )


def _dispersion_cost(
    observations: Sequence[FixedPlaneBoresightObservation],
    rx_deg: float,
    ry_deg: float,
    rz_deg: float,
) -> float:
    reference_normals = _reference_normals(
        observations, rotation_xyz_deg(rx_deg, ry_deg, rz_deg)
    )
    mean = _mean_direction(reference_normals)
    return sum(
        sum((vector[index] - mean[index]) ** 2 for index in range(3))
        for vector in reference_normals
    ) / len(reference_normals)


def solve_fixed_plane_boresight(
    observations: Sequence[FixedPlaneBoresightObservation],
    *,
    max_abs_angle_deg: float = 20.0,
) -> BoresightFit:
    """Fit a small ToF->body boresight rotation from a fixed plane and body motion.

    The plane's absolute orientation is an unknown nuisance parameter. A candidate
    boresight rotation is good when every observed ToF plane normal, after mapping
    through the candidate extrinsic and the measured relative body rotation, lands
    on one common normal in the reference frame.

    This v1 solver intentionally searches only a modest assembly-error envelope.
    It is portable, deterministic and dependency-free rather than optimized for
    speed.
    """

    observations = tuple(observations)
    if len(observations) < 4:
        raise ExtrinsicError("at least four fixed-plane poses are required")
    bound = float(max_abs_angle_deg)
    if not math.isfinite(bound) or bound <= 0.0 or bound > 45.0:
        raise ExtrinsicError("max_abs_angle_deg must be in (0, 45]")

    params = [0.0, 0.0, 0.0]
    for step in (5.0, 2.0, 1.0, 0.5, 0.2, 0.1, 0.05, 0.02, 0.01):
        improved = True
        while improved:
            improved = False
            best_cost = _dispersion_cost(observations, *params)
            best_params = params[:]
            for index in range(3):
                for direction in (-1.0, 1.0):
                    candidate = params[:]
                    candidate[index] += direction * step
                    if abs(candidate[index]) > bound:
                        continue
                    candidate_cost = _dispersion_cost(observations, *candidate)
                    if candidate_cost + 1.0e-15 < best_cost:
                        best_cost = candidate_cost
                        best_params = candidate
                        improved = True
            params = best_params

    best_cost = _dispersion_cost(observations, *params)
    observability = []
    for index in range(3):
        costs = []
        for direction in (-1.0, 1.0):
            candidate = params[:]
            candidate[index] += direction
            if abs(candidate[index]) > bound:
                costs.append(best_cost)
            else:
                costs.append(_dispersion_cost(observations, *candidate))
        observability.append(max(0.0, (sum(costs) / 2.0) - best_cost))

    if min(observability) < 1.0e-9:
        raise ExtrinsicError(
            "body-motion set does not constrain all three boresight rotation axes"
        )

    rotation = rotation_xyz_deg(*params)
    reference_normals = _reference_normals(observations, rotation)
    mean = _mean_direction(reference_normals)
    angular_errors = []
    for vector in reference_normals:
        dot = max(-1.0, min(1.0, sum(vector[i] * mean[i] for i in range(3))))
        angular_errors.append(math.degrees(math.acos(dot)))

    rms = math.sqrt(
        sum(error * error for error in angular_errors) / len(angular_errors)
    )
    max_error = max(angular_errors)

    extrinsic = TofBodyRotation(
        name="fixed-plane-boresight",
        role="calibrated",
        rotation_body_from_tof=rotation,
        metadata={
            "calibration": {
                "method": BORESIGHT_METHOD,
                "observation_count": len(observations),
                "rotation_parameter_convention": (
                    "active fixed-axis X then Y then Z; R = Rz*Ry*Rx"
                ),
                "rotation_xyz_deg": list(params),
                "rms_normal_error_deg": rms,
                "max_normal_error_deg": max_error,
                "observability_cost_increase_at_1deg": observability,
            }
        },
    )
    return BoresightFit(
        extrinsic=extrinsic,
        reference_plane_normal=mean,
        rotation_x_deg=params[0],
        rotation_y_deg=params[1],
        rotation_z_deg=params[2],
        rms_normal_error_deg=rms,
        max_normal_error_deg=max_error,
        observability_cost_increase_at_1deg=tuple(observability),
    )
