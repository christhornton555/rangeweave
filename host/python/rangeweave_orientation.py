"""Persistent six-axis orientation estimation for Rangeweave captures.

Project-owned attitude convention:

    q_reference_from_body = (w, x, y, z)
    v_reference = R(q_reference_from_body) * v_body

Quaternions use the Hamilton product and active right-handed rotations. Gyroscope
samples are mapped into ``device_body`` and are body-frame angular velocity, so
propagation right-multiplies the current attitude by the body-frame increment.

This first Phase 3 estimator is deliberately six-axis: gyro + accelerometer only.
Accelerometer correction constrains gravity/pitch/roll when specific force is
credible; yaw about gravity is not observable and is not claimed to be globally
referenced. Magnetometer integration is a later layer.
"""

from __future__ import annotations

from dataclasses import dataclass
import math
import statistics
from typing import Sequence

import rangeweave_extrinsics as ext
import rangeweave_imu_relative as imu


Vector3 = ext.Vector3
Matrix3 = ext.Matrix3
Quaternion = tuple[float, float, float, float]  # scalar-first (w, x, y, z)

UP_REFERENCE: Vector3 = (0.0, -1.0, 0.0)
DOWN_REFERENCE: Vector3 = (0.0, 1.0, 0.0)

DEFAULT_INITIALISATION_SECONDS = 1.0
DEFAULT_GRAVITY_GAIN_PER_S = 2.0
DEFAULT_ACCEL_FULL_WEIGHT_DEVIATION_G = 0.05
DEFAULT_ACCEL_ZERO_WEIGHT_DEVIATION_G = 0.20
DEFAULT_MAX_INITIAL_GYRO_SPREAD_DPS = 1.5
DEFAULT_MAX_INITIAL_ACCEL_DEVIATION_G = 0.04


class OrientationError(ValueError):
    """Raised when persistent orientation estimation cannot proceed safely."""


@dataclass(frozen=True)
class InitialisationDiagnostics:
    sample_count: int
    duration_s: float
    gyro_bias_body_dps: Vector3
    gyro_mad_body_dps: Vector3
    gyro_spread_dps: float
    accel_median_body_g: Vector3
    accel_norm_g: float
    accel_median_deviation_g: float


@dataclass(frozen=True)
class OrientationSample:
    time_s: float
    quaternion_reference_from_body: Quaternion
    reference_from_body: Matrix3
    accel_weight: float
    gravity_innovation_deg: float


@dataclass(frozen=True)
class OrientationRun:
    samples: tuple[OrientationSample, ...]
    initialisation: InitialisationDiagnostics
    clock_fit: imu.ClockFit
    imu_mapping_role: str
    gravity_gain_per_s: float


@dataclass(frozen=True)
class BodyImuSeries:
    times_s: tuple[float, ...]
    gyros_body_dps: tuple[Vector3, ...]
    accels_body_g: tuple[Vector3, ...]
    clock_fit: imu.ClockFit


def _vector3(values: Sequence[float]) -> Vector3:
    result = tuple(float(value) for value in values)
    if len(result) != 3 or not all(math.isfinite(value) for value in result):
        raise OrientationError("vector must contain three finite values")
    return result  # type: ignore[return-value]


def _norm(values: Sequence[float]) -> float:
    return math.sqrt(sum(float(value) * float(value) for value in values))


def _normalise_vector(values: Sequence[float]) -> Vector3:
    result = _vector3(values)
    length = _norm(result)
    if length <= 1.0e-12:
        raise OrientationError("cannot normalise a zero vector")
    return tuple(value / length for value in result)  # type: ignore[return-value]


def _dot(a: Sequence[float], b: Sequence[float]) -> float:
    return sum(float(a[index]) * float(b[index]) for index in range(3))


def _cross(a: Sequence[float], b: Sequence[float]) -> Vector3:
    ax, ay, az = _vector3(a)
    bx, by, bz = _vector3(b)
    return (
        ay * bz - az * by,
        az * bx - ax * bz,
        ax * by - ay * bx,
    )


def _angle_deg(a: Sequence[float], b: Sequence[float]) -> float:
    na = _normalise_vector(a)
    nb = _normalise_vector(b)
    cosine = max(-1.0, min(1.0, _dot(na, nb)))
    return math.degrees(math.acos(cosine))


def quaternion_normalise(quaternion: Sequence[float]) -> Quaternion:
    values = tuple(float(value) for value in quaternion)
    if len(values) != 4 or not all(math.isfinite(value) for value in values):
        raise OrientationError("quaternion must contain four finite values")
    length = math.sqrt(sum(value * value for value in values))
    if length <= 1.0e-15:
        raise OrientationError("quaternion norm must be non-zero")
    return tuple(value / length for value in values)  # type: ignore[return-value]


def quaternion_conjugate(quaternion: Sequence[float]) -> Quaternion:
    w, x, y, z = quaternion_normalise(quaternion)
    return (w, -x, -y, -z)


def quaternion_multiply(left: Sequence[float], right: Sequence[float]) -> Quaternion:
    """Hamilton product ``left ⊗ right`` for active scalar-first quaternions."""

    lw, lx, ly, lz = tuple(float(value) for value in left)
    rw, rx, ry, rz = tuple(float(value) for value in right)
    return (
        lw * rw - lx * rx - ly * ry - lz * rz,
        lw * rx + lx * rw + ly * rz - lz * ry,
        lw * ry - lx * rz + ly * rw + lz * rx,
        lw * rz + lx * ry - ly * rx + lz * rw,
    )


def quaternion_from_rotation_vector(rotation_vector_rad: Sequence[float]) -> Quaternion:
    x, y, z = _vector3(rotation_vector_rad)
    angle = math.sqrt(x * x + y * y + z * z)
    if angle < 1.0e-12:
        # First-order vector part; normalisation removes the tiny norm error.
        return quaternion_normalise((1.0, x / 2.0, y / 2.0, z / 2.0))
    half = angle / 2.0
    scale = math.sin(half) / angle
    return (math.cos(half), x * scale, y * scale, z * scale)


def quaternion_to_matrix(quaternion: Sequence[float]) -> Matrix3:
    w, x, y, z = quaternion_normalise(quaternion)
    xx, yy, zz = x * x, y * y, z * z
    xy, xz, yz = x * y, x * z, y * z
    wx, wy, wz = w * x, w * y, w * z
    return (
        (1.0 - 2.0 * (yy + zz), 2.0 * (xy - wz), 2.0 * (xz + wy)),
        (2.0 * (xy + wz), 1.0 - 2.0 * (xx + zz), 2.0 * (yz - wx)),
        (2.0 * (xz - wy), 2.0 * (yz + wx), 1.0 - 2.0 * (xx + yy)),
    )


def matrix_to_quaternion(matrix: Matrix3) -> Quaternion:
    """Convert a proper active rotation matrix to scalar-first quaternion form."""

    m = matrix
    trace = m[0][0] + m[1][1] + m[2][2]
    if trace > 0.0:
        s = math.sqrt(trace + 1.0) * 2.0
        q = (
            0.25 * s,
            (m[2][1] - m[1][2]) / s,
            (m[0][2] - m[2][0]) / s,
            (m[1][0] - m[0][1]) / s,
        )
    elif m[0][0] > m[1][1] and m[0][0] > m[2][2]:
        s = math.sqrt(1.0 + m[0][0] - m[1][1] - m[2][2]) * 2.0
        q = (
            (m[2][1] - m[1][2]) / s,
            0.25 * s,
            (m[0][1] + m[1][0]) / s,
            (m[0][2] + m[2][0]) / s,
        )
    elif m[1][1] > m[2][2]:
        s = math.sqrt(1.0 + m[1][1] - m[0][0] - m[2][2]) * 2.0
        q = (
            (m[0][2] - m[2][0]) / s,
            (m[0][1] + m[1][0]) / s,
            0.25 * s,
            (m[1][2] + m[2][1]) / s,
        )
    else:
        s = math.sqrt(1.0 + m[2][2] - m[0][0] - m[1][1]) * 2.0
        q = (
            (m[1][0] - m[0][1]) / s,
            (m[0][2] + m[2][0]) / s,
            (m[1][2] + m[2][1]) / s,
            0.25 * s,
        )
    qn = quaternion_normalise(q)
    # Canonical sign is useful for deterministic text/regression output.
    return tuple(-value for value in qn) if qn[0] < 0.0 else qn  # type: ignore[return-value]


def initial_reference_from_body(accel_body_g: Sequence[float]) -> Matrix3:
    """Create ``R_reference_from_body`` from stationary accelerometer direction.

    Accelerometer specific force defines local up/down. Initial yaw is chosen so the
    horizontal projection of body +Z is local +Z. This is a local yaw zero only.
    """

    up_body = _normalise_vector(accel_body_g)
    down_body = tuple(-value for value in up_body)

    body_forward = (0.0, 0.0, 1.0)
    forward_projection = tuple(
        body_forward[index] - _dot(body_forward, down_body) * down_body[index]
        for index in range(3)
    )
    if _norm(forward_projection) >= 1.0e-6:
        z_ref_in_body = _normalise_vector(forward_projection)
        x_ref_in_body = _normalise_vector(_cross(down_body, z_ref_in_body))
    else:
        body_right = (1.0, 0.0, 0.0)
        right_projection = tuple(
            body_right[index] - _dot(body_right, down_body) * down_body[index]
            for index in range(3)
        )
        if _norm(right_projection) < 1.0e-6:
            raise OrientationError("initial body axes are degenerate relative to gravity")
        x_ref_in_body = _normalise_vector(right_projection)
        z_ref_in_body = _normalise_vector(_cross(x_ref_in_body, down_body))

    # Re-orthogonalise in the chosen handedness. Rows are reference basis vectors
    # expressed in body coordinates, so row dot v_body gives v_reference.
    z_ref_in_body = _normalise_vector(_cross(x_ref_in_body, down_body))
    x_ref_in_body = _normalise_vector(_cross(down_body, z_ref_in_body))
    return (x_ref_in_body, down_body, z_ref_in_body)


def accel_confidence(
    accel_body_g: Sequence[float],
    *,
    full_weight_deviation_g: float = DEFAULT_ACCEL_FULL_WEIGHT_DEVIATION_G,
    zero_weight_deviation_g: float = DEFAULT_ACCEL_ZERO_WEIGHT_DEVIATION_G,
) -> float:
    """Return [0, 1] gravity-correction confidence from specific-force magnitude."""

    full = float(full_weight_deviation_g)
    zero = float(zero_weight_deviation_g)
    if not (0.0 <= full < zero):
        raise OrientationError("accelerometer confidence thresholds must satisfy 0 <= full < zero")
    deviation = abs(_norm(_vector3(accel_body_g)) - 1.0)
    if deviation <= full:
        return 1.0
    if deviation >= zero:
        return 0.0
    return 1.0 - (deviation - full) / (zero - full)


def body_imu_series(
    samples: Sequence[object],
    clock_syncs: Sequence[object],
    *,
    ctrl1_xl: int,
    ctrl2_g: int,
) -> BodyImuSeries:
    samples = tuple(samples)
    if len(samples) < 2:
        raise OrientationError("at least two IMU samples are required")
    clock_fit = imu.fit_lsm_clock(clock_syncs)
    ticks = [int(getattr(sample, "lsm_tick")) for sample in samples]
    if any(b <= a for a, b in zip(ticks, ticks[1:])):
        raise OrientationError("IMU sample timestamps must be strictly increasing")

    first_us = clock_fit.tick_to_us(ticks[0])
    times_s = tuple((clock_fit.tick_to_us(tick) - first_us) / 1_000_000.0 for tick in ticks)
    accel_scale = imu.accel_g_per_lsb(ctrl1_xl)
    gyro_scale = imu.gyro_dps_per_lsb(ctrl2_g)
    accels = []
    gyros = []
    for sample in samples:
        accel_sensor = (
            float(getattr(sample, "accel_x")) * accel_scale,
            float(getattr(sample, "accel_y")) * accel_scale,
            float(getattr(sample, "accel_z")) * accel_scale,
        )
        gyro_sensor = (
            float(getattr(sample, "gyro_x")) * gyro_scale,
            float(getattr(sample, "gyro_y")) * gyro_scale,
            float(getattr(sample, "gyro_z")) * gyro_scale,
        )
        accels.append(imu.imu_vector_to_body(accel_sensor))
        gyros.append(imu.imu_vector_to_body(gyro_sensor))
    return BodyImuSeries(times_s, tuple(gyros), tuple(accels), clock_fit)


def _median_vector(vectors: Sequence[Vector3]) -> Vector3:
    if not vectors:
        raise OrientationError("cannot take median of an empty vector sequence")
    return tuple(
        float(statistics.median(vector[axis] for vector in vectors))
        for axis in range(3)
    )  # type: ignore[return-value]


def initialise_from_series(
    series: BodyImuSeries,
    *,
    initialisation_seconds: float = DEFAULT_INITIALISATION_SECONDS,
) -> tuple[Quaternion, InitialisationDiagnostics]:
    duration = float(initialisation_seconds)
    if not (0.25 <= duration <= 5.0):
        raise OrientationError("initialisation_seconds must be in [0.25, 5.0]")
    indices = [index for index, time_s in enumerate(series.times_s) if time_s <= duration]
    if len(indices) < 20:
        raise OrientationError("initialisation interval contains fewer than 20 IMU samples")
    count = indices[-1] + 1
    gyros = series.gyros_body_dps[:count]
    accels = series.accels_body_g[:count]
    gyro_median = _median_vector(gyros)
    gyro_mad = tuple(
        float(statistics.median(abs(vector[axis] - gyro_median[axis]) for vector in gyros))
        for axis in range(3)
    )
    gyro_spread = _norm(gyro_mad)
    accel_median = _median_vector(accels)
    accel_deviation = float(
        statistics.median(
            _norm(tuple(vector[axis] - accel_median[axis] for axis in range(3)))
            for vector in accels
        )
    )
    accel_norm = _norm(accel_median)
    if not (0.75 <= accel_norm <= 1.25):
        raise OrientationError(
            f"initialisation accelerometer norm is implausible ({accel_norm:.3f} g)"
        )
    if gyro_spread > DEFAULT_MAX_INITIAL_GYRO_SPREAD_DPS:
        raise OrientationError(
            "initialisation interval is not stationary enough: "
            f"gyro robust spread {gyro_spread:.3f} deg/s"
        )
    if accel_deviation > DEFAULT_MAX_INITIAL_ACCEL_DEVIATION_G:
        raise OrientationError(
            "initialisation interval is not stationary enough: "
            f"accelerometer median deviation {accel_deviation:.4f} g"
        )

    matrix = initial_reference_from_body(accel_median)
    diagnostics = InitialisationDiagnostics(
        sample_count=count,
        duration_s=series.times_s[count - 1] - series.times_s[0],
        gyro_bias_body_dps=gyro_median,
        gyro_mad_body_dps=gyro_mad,  # type: ignore[arg-type]
        gyro_spread_dps=gyro_spread,
        accel_median_body_g=accel_median,
        accel_norm_g=accel_norm,
        accel_median_deviation_g=accel_deviation,
    )
    return matrix_to_quaternion(matrix), diagnostics


class SixAxisOrientationFilter:
    """Deterministic gyro propagation with confidence-gated gravity correction."""

    def __init__(
        self,
        *,
        quaternion_reference_from_body: Sequence[float],
        gyro_bias_body_dps: Sequence[float] = (0.0, 0.0, 0.0),
        gravity_gain_per_s: float = DEFAULT_GRAVITY_GAIN_PER_S,
        accel_full_weight_deviation_g: float = DEFAULT_ACCEL_FULL_WEIGHT_DEVIATION_G,
        accel_zero_weight_deviation_g: float = DEFAULT_ACCEL_ZERO_WEIGHT_DEVIATION_G,
    ) -> None:
        gain = float(gravity_gain_per_s)
        if not math.isfinite(gain) or gain < 0.0:
            raise OrientationError("gravity_gain_per_s must be finite and non-negative")
        self._q = quaternion_normalise(quaternion_reference_from_body)
        self._bias_dps = _vector3(gyro_bias_body_dps)
        self._gain = gain
        self._full_dev = float(accel_full_weight_deviation_g)
        self._zero_dev = float(accel_zero_weight_deviation_g)
        self._last_time_s: float | None = None
        self._last_rate_body_rad_s: Vector3 | None = None

    @property
    def quaternion_reference_from_body(self) -> Quaternion:
        return self._q

    @property
    def reference_from_body(self) -> Matrix3:
        return quaternion_to_matrix(self._q)

    @property
    def gyro_bias_body_dps(self) -> Vector3:
        return self._bias_dps

    def observe(
        self,
        time_s: float,
        gyro_body_dps: Sequence[float],
        accel_body_g: Sequence[float],
    ) -> OrientationSample:
        """Consume one timestamped body-frame IMU sample and return current attitude."""

        time_value = float(time_s)
        if not math.isfinite(time_value):
            raise OrientationError("sample time must be finite")
        gyro = _vector3(gyro_body_dps)
        accel = _vector3(accel_body_g)
        weight = accel_confidence(
            accel,
            full_weight_deviation_g=self._full_dev,
            zero_weight_deviation_g=self._zero_dev,
        )

        rotation = quaternion_to_matrix(self._q)
        predicted_up_body = ext.matrix_vector(ext.transpose(rotation), UP_REFERENCE)
        measured_up_body = _normalise_vector(accel)
        innovation_deg = _angle_deg(predicted_up_body, measured_up_body)
        correction_axis = _cross(measured_up_body, predicted_up_body)

        corrected_rate_rad_s = tuple(
            math.radians(gyro[axis] - self._bias_dps[axis])
            + self._gain * weight * correction_axis[axis]
            for axis in range(3)
        )

        if self._last_time_s is not None:
            dt = time_value - self._last_time_s
            if dt <= 0.0 or dt > 0.1:
                raise OrientationError(f"invalid IMU timestep {dt:.6f} s")
            previous = self._last_rate_body_rad_s
            if previous is None:
                raise OrientationError("internal orientation-rate state is inconsistent")
            average_rate = tuple(
                (previous[axis] + corrected_rate_rad_s[axis]) / 2.0
                for axis in range(3)
            )
            dq = quaternion_from_rotation_vector(tuple(value * dt for value in average_rate))
            self._q = quaternion_normalise(quaternion_multiply(self._q, dq))

            # Report innovation after propagation as the useful current residual.
            rotation = quaternion_to_matrix(self._q)
            predicted_up_body = ext.matrix_vector(ext.transpose(rotation), UP_REFERENCE)
            innovation_deg = _angle_deg(predicted_up_body, measured_up_body)

        self._last_time_s = time_value
        self._last_rate_body_rad_s = corrected_rate_rad_s
        return OrientationSample(
            time_s=time_value,
            quaternion_reference_from_body=self._q,
            reference_from_body=quaternion_to_matrix(self._q),
            accel_weight=weight,
            gravity_innovation_deg=innovation_deg,
        )


def estimate_orientation(
    samples: Sequence[object],
    clock_syncs: Sequence[object],
    *,
    ctrl1_xl: int,
    ctrl2_g: int,
    initialisation_seconds: float = DEFAULT_INITIALISATION_SECONDS,
    gravity_gain_per_s: float = DEFAULT_GRAVITY_GAIN_PER_S,
) -> OrientationRun:
    """Estimate persistent ``local_reference_from_body`` over a recorded IMU stream."""

    series = body_imu_series(
        samples,
        clock_syncs,
        ctrl1_xl=ctrl1_xl,
        ctrl2_g=ctrl2_g,
    )
    initial_q, initialisation = initialise_from_series(
        series,
        initialisation_seconds=initialisation_seconds,
    )
    filter_ = SixAxisOrientationFilter(
        quaternion_reference_from_body=initial_q,
        gyro_bias_body_dps=initialisation.gyro_bias_body_dps,
        gravity_gain_per_s=gravity_gain_per_s,
    )
    output = tuple(
        filter_.observe(time_s, gyro, accel)
        for time_s, gyro, accel in zip(
            series.times_s,
            series.gyros_body_dps,
            series.accels_body_g,
        )
    )
    return OrientationRun(
        samples=output,
        initialisation=initialisation,
        clock_fit=series.clock_fit,
        imu_mapping_role=imu.REFERENCE_IMU_MAPPING_ROLE,
        gravity_gain_per_s=float(gravity_gain_per_s),
    )
