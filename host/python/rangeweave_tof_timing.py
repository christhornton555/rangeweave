"""Resolve Rangeweave ToF observation-time alignment for quick-start/calibrated use.

The protocol v0.1 ToF timestamp is ``mcu_ready_us``: the time at which the
producer observes data-ready, not a guaranteed sensor-internal measurement
instant.  This module therefore treats effective ToF/IMU time alignment as a
versioned, configuration-scoped calibration quantity rather than as a
VL53L5CX model constant.

Two user-facing modes are deliberately supported:

``quick-start``
    No physical calibration is required.  A matching project-supplied nominal
    profile is used when available; otherwise the conservative zero-offset
    fallback is returned with an explicit ``uncalibrated`` role.

``calibrated``
    A per-build timing artifact is required and must match both its capture
    configuration fingerprint and its explicit assembly id.  A material
    mismatch is an error rather than a silent fallback.

An explicit numeric override has highest precedence in either mode.  It is
reported as an override, never mislabelled as calibration.
"""

from __future__ import annotations

from dataclasses import dataclass, field
import json
import math
from pathlib import Path
from typing import Any, Mapping, Sequence

import rangeweave_protocol as rw


TIMING_SCHEMA = "rangeweave.tof-time-alignment"
TIMING_SCHEMA_VERSION = 1
TIMESTAMP_FIELD = "mcu_ready_us"
OFFSET_SIGN_CONVENTION = "offset_added_to_mcu_ready_us_before_attitude_interpolation"

ROLE_CALIBRATED = "calibrated"
ROLE_NOMINAL = "nominal-fallback"
ROLE_UNCALIBRATED = "uncalibrated"
ROLE_OVERRIDE = "explicit-override"

MODE_QUICK_START = "quick-start"
MODE_CALIBRATED = "calibrated"
MODES = (MODE_QUICK_START, MODE_CALIBRATED)


class TofTimingError(ValueError):
    """Raised when a timing artifact or resolver request is invalid."""


def _decode_text(value: bytes | None) -> str | None:
    if value is None:
        return None
    return value.decode("utf-8", "replace")


def _single_byte(info: rw.StreamInfo | None, tag: int) -> int | None:
    if info is None:
        return None
    value = info.first_value(tag)
    if value is None or len(value) != 1:
        return None
    return value[0]


def _field_mask(info: rw.StreamInfo | None) -> int | None:
    if info is None:
        return None
    value = info.first_value(rw.INFO_TOF_DEFAULT_FIELD_MASK)
    if value is None or len(value) != 2:
        return None
    return int.from_bytes(value, "little")


def _tof_grid(info: rw.StreamInfo | None) -> dict[str, int] | None:
    if info is None:
        return None
    value = info.first_value(rw.INFO_TOF_GRID_CONFIG)
    if value is None or len(value) < 3:
        return None
    return {"rows": value[0], "cols": value[1], "hz": value[2]}


def _protocol_dict(protocol_versions: Sequence[tuple[int, int]]) -> dict[str, int] | None:
    versions = tuple(sorted(set(protocol_versions)))
    if len(versions) != 1:
        return None
    major, minor = versions[0]
    return {"major": int(major), "minor": int(minor)}


def capture_fingerprint(
    info: rw.StreamInfo | None,
    protocol_versions: Sequence[tuple[int, int]],
    *,
    assembly_id: str | None = None,
) -> dict[str, Any]:
    """Return the timing-relevant capture/configuration fingerprint.

    ``assembly_id`` is deliberately external to protocol v0.1 because the
    current reference stream does not expose a stable physical sensor serial.
    Calibrated artifacts use this user/deployment-managed identity to prevent a
    calibration for one physical build being silently copied to another.
    """

    stream_info: dict[str, Any] = {}
    if info is not None:
        stream_info["info_revision"] = int(info.info_revision)
        firmware = _decode_text(info.first_value(rw.INFO_FIRMWARE_LABEL))
        source_profile = _decode_text(info.first_value(rw.INFO_SOURCE_PROFILE))
        if firmware is not None:
            stream_info["firmware_label"] = firmware
        if source_profile is not None:
            stream_info["source_profile"] = source_profile
        i2c_address = _single_byte(info, rw.INFO_TOF_I2C_ADDRESS)
        if i2c_address is not None:
            stream_info["tof_i2c_address"] = i2c_address
        grid = _tof_grid(info)
        if grid is not None:
            stream_info["tof_grid"] = grid
        field_mask = _field_mask(info)
        if field_mask is not None:
            stream_info["tof_default_field_mask"] = field_mask

    result: dict[str, Any] = {
        "protocol": _protocol_dict(protocol_versions),
        "stream_info": stream_info,
    }
    if assembly_id:
        result["assembly_id"] = str(assembly_id)
    return result


def _mapping(value: Any, name: str) -> Mapping[str, Any]:
    if not isinstance(value, Mapping):
        raise TofTimingError(f"{name} must be an object")
    return value


def _finite_offset(value: Any) -> float:
    try:
        result = float(value)
    except (TypeError, ValueError) as exc:
        raise TofTimingError("effective_offset_ms must be numeric") from exc
    if not math.isfinite(result):
        raise TofTimingError("effective_offset_ms must be finite")
    if abs(result) > 1000.0:
        raise TofTimingError("effective_offset_ms magnitude above 1000 ms is implausible")
    return result


def _compare_expected(expected: Any, actual: Any, path: str, mismatches: list[str]) -> None:
    if isinstance(expected, Mapping):
        if not isinstance(actual, Mapping):
            mismatches.append(f"{path}: expected object, capture has {actual!r}")
            return
        for key, value in expected.items():
            child = f"{path}.{key}" if path else str(key)
            if key not in actual:
                mismatches.append(f"{child}: missing from capture fingerprint")
            else:
                _compare_expected(value, actual[key], child, mismatches)
        return
    if actual != expected:
        mismatches.append(f"{path}: artifact {expected!r} != capture {actual!r}")


@dataclass(frozen=True)
class TofTimingArtifact:
    name: str
    role: str
    effective_offset_ms: float
    applies_to: Mapping[str, Any]
    evidence: Mapping[str, Any] = field(default_factory=dict)
    metadata: Mapping[str, Any] = field(default_factory=dict)

    def __post_init__(self) -> None:
        if not self.name:
            raise TofTimingError("timing artifact name must be non-empty")
        if self.role not in (ROLE_CALIBRATED, ROLE_NOMINAL):
            raise TofTimingError("timing artifact role must be calibrated or nominal-fallback")
        object.__setattr__(self, "effective_offset_ms", _finite_offset(self.effective_offset_ms))
        applies_to = dict(_mapping(self.applies_to, "applies_to"))
        if not applies_to:
            raise TofTimingError("timing artifact applies_to fingerprint must not be empty")
        if self.role == ROLE_CALIBRATED and not applies_to.get("assembly_id"):
            raise TofTimingError("calibrated timing artifacts require applies_to.assembly_id")
        object.__setattr__(self, "applies_to", applies_to)
        object.__setattr__(self, "evidence", dict(_mapping(self.evidence, "evidence")))
        object.__setattr__(self, "metadata", dict(_mapping(self.metadata, "metadata")))

    def mismatches(self, fingerprint: Mapping[str, Any]) -> tuple[str, ...]:
        mismatches: list[str] = []
        _compare_expected(self.applies_to, fingerprint, "", mismatches)
        return tuple(mismatches)

    def to_dict(self) -> dict[str, Any]:
        return {
            "schema": TIMING_SCHEMA,
            "schema_version": TIMING_SCHEMA_VERSION,
            "name": self.name,
            "role": self.role,
            "timestamp_field": TIMESTAMP_FIELD,
            "offset_sign_convention": OFFSET_SIGN_CONVENTION,
            "effective_offset_ms": self.effective_offset_ms,
            "applies_to": dict(self.applies_to),
            "evidence": dict(self.evidence),
            "metadata": dict(self.metadata),
        }

    @classmethod
    def from_dict(cls, document: Mapping[str, Any]) -> "TofTimingArtifact":
        if document.get("schema") != TIMING_SCHEMA:
            raise TofTimingError("unexpected ToF timing artifact schema")
        if document.get("schema_version") != TIMING_SCHEMA_VERSION:
            raise TofTimingError("unsupported ToF timing artifact schema version")
        if document.get("timestamp_field") != TIMESTAMP_FIELD:
            raise TofTimingError("timing artifact timestamp_field must be mcu_ready_us")
        if document.get("offset_sign_convention") != OFFSET_SIGN_CONVENTION:
            raise TofTimingError("unsupported ToF timing offset sign convention")
        try:
            return cls(
                name=str(document["name"]),
                role=str(document["role"]),
                effective_offset_ms=document["effective_offset_ms"],
                applies_to=_mapping(document["applies_to"], "applies_to"),
                evidence=_mapping(document.get("evidence", {}), "evidence"),
                metadata=_mapping(document.get("metadata", {}), "metadata"),
            )
        except KeyError as exc:
            raise TofTimingError(f"missing timing artifact field: {exc.args[0]}") from exc


def load_artifact(path: Path | str) -> TofTimingArtifact:
    path = Path(path)
    try:
        document = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as exc:
        raise TofTimingError(f"could not read timing artifact {path}: {exc}") from exc
    if not isinstance(document, Mapping):
        raise TofTimingError("timing artifact root must be an object")
    return TofTimingArtifact.from_dict(document)


@dataclass(frozen=True)
class TimingResolution:
    mode: str
    role: str
    effective_offset_ms: float
    source: str
    artifact_name: str | None
    fingerprint: Mapping[str, Any]

    @property
    def calibrated(self) -> bool:
        return self.role == ROLE_CALIBRATED


# The reference stack is intentionally recognised in quick-start mode, but its
# conservative nominal timing remains 0 ms until cross-device evidence exists.
# The per-build reference-rig calibration (-24 ms) lives in a separate artifact.
REFERENCE_QUICK_START_PROFILE = TofTimingArtifact(
    name="pico2w-vl53l5cx-8x8-15hz-conservative-timing",
    role=ROLE_NOMINAL,
    effective_offset_ms=0.0,
    applies_to={
        "protocol": {"major": 0, "minor": 1},
        "stream_info": {
            "firmware_label": "rangeweave-pico2w-acq-0.1",
            "source_profile": "pico2w-lsm6dsox-lis3mdl-vl53l5cx-8x8-15hz",
            "tof_i2c_address": 41,
            "tof_grid": {"rows": 8, "cols": 8, "hz": 15},
            "tof_default_field_mask": 3,
        },
    },
    evidence={
        "status": "conservative-quick-start",
        "note": (
            "Zero offset is retained as the cross-device nominal until multiple physical "
            "builds establish a safe population-level timing profile."
        ),
    },
)

NOMINAL_PROFILES = (REFERENCE_QUICK_START_PROFILE,)


def resolve_tof_timing(
    info: rw.StreamInfo | None,
    protocol_versions: Sequence[tuple[int, int]],
    *,
    mode: str = MODE_QUICK_START,
    assembly_id: str | None = None,
    artifact: TofTimingArtifact | None = None,
    explicit_offset_ms: float | None = None,
    nominal_profiles: Sequence[TofTimingArtifact] = NOMINAL_PROFILES,
) -> TimingResolution:
    if mode not in MODES:
        raise TofTimingError(f"timing mode must be one of: {', '.join(MODES)}")
    fingerprint = capture_fingerprint(
        info,
        protocol_versions,
        assembly_id=assembly_id,
    )

    if explicit_offset_ms is not None:
        offset = _finite_offset(explicit_offset_ms)
        return TimingResolution(
            mode=mode,
            role=ROLE_OVERRIDE,
            effective_offset_ms=offset,
            source="explicit --tof-time-offset-ms override",
            artifact_name=None,
            fingerprint=fingerprint,
        )

    if mode == MODE_CALIBRATED:
        if artifact is None:
            raise TofTimingError("calibrated timing mode requires --timing-artifact")
        if artifact.role != ROLE_CALIBRATED:
            raise TofTimingError("calibrated timing mode requires a calibrated artifact")
        mismatches = artifact.mismatches(fingerprint)
        if mismatches:
            raise TofTimingError(
                "timing artifact does not apply to this build/configuration: "
                + "; ".join(mismatches)
            )
        return TimingResolution(
            mode=mode,
            role=ROLE_CALIBRATED,
            effective_offset_ms=artifact.effective_offset_ms,
            source="matched per-build timing artifact",
            artifact_name=artifact.name,
            fingerprint=fingerprint,
        )

    # Quick-start mode deliberately does not consume a calibrated per-build
    # artifact.  A supplied nominal artifact is allowed for deployment-specific
    # profile registries, provided it matches the capture fingerprint.
    if artifact is not None:
        if artifact.role != ROLE_NOMINAL:
            raise TofTimingError(
                "quick-start timing mode cannot use a calibrated artifact; "
                "select --timing-mode calibrated"
            )
        mismatches = artifact.mismatches(fingerprint)
        if mismatches:
            raise TofTimingError(
                "nominal timing artifact does not match this configuration: "
                + "; ".join(mismatches)
            )
        return TimingResolution(
            mode=mode,
            role=ROLE_NOMINAL,
            effective_offset_ms=artifact.effective_offset_ms,
            source="matched supplied nominal timing profile",
            artifact_name=artifact.name,
            fingerprint=fingerprint,
        )

    for profile in nominal_profiles:
        if profile.role != ROLE_NOMINAL:
            continue
        if not profile.mismatches(fingerprint):
            return TimingResolution(
                mode=mode,
                role=ROLE_NOMINAL,
                effective_offset_ms=profile.effective_offset_ms,
                source="matched built-in quick-start timing profile",
                artifact_name=profile.name,
                fingerprint=fingerprint,
            )

    return TimingResolution(
        mode=mode,
        role=ROLE_UNCALIBRATED,
        effective_offset_ms=0.0,
        source="no matching nominal profile; conservative zero-offset fallback",
        artifact_name=None,
        fingerprint=fingerprint,
    )
