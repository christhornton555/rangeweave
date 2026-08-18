"""Dependency-light reference implementation of Rangeweave wire protocol v0.1.

This module is intentionally standard-library only. It is the reference decoder used to
validate byte-level fixtures; it is not intended to define Python-specific protocol
semantics.
"""

from __future__ import annotations

from dataclasses import dataclass
import struct
from typing import Optional, Union

MAGIC = b"RW"
PROTOCOL_MAJOR = 0
PROTOCOL_MINOR = 1
MAX_DECODED_FRAME = 1024

RECORD_IMU_BATCH = 0x01
RECORD_MAG = 0x02
RECORD_TOF_GRID = 0x03
RECORD_CLOCK_SYNC = 0x04
RECORD_STATUS = 0x05

TOF_FIELD_DISTANCE_MM = 0x0001
TOF_FIELD_REFLECTANCE_PERCENT = 0x0002
TOF_FIELD_TARGET_STATUS = 0x0004
TOF_FIELD_RANGE_SIGMA_MM = 0x0008  # reserved; not defined in protocol v0.1
TOF_FIELD_NB_TARGET_DETECTED = 0x0010  # reserved; not defined in protocol v0.1
TOF_KNOWN_FIELD_MASK = (
    TOF_FIELD_DISTANCE_MM
    | TOF_FIELD_REFLECTANCE_PERCENT
    | TOF_FIELD_TARGET_STATUS
)

MAG_FLAG_RETRY_USED = 0x01

STATUS_FLAG_ACQUISITION_READY = 0x0001
STATUS_FLAG_CLOCK_SYNC_ACTIVE = 0x0002
STATUS_FLAG_FIFO_OVERRUN_LATCHED = 0x0004
STATUS_FLAG_TRANSPORT_BACKPRESSURE = 0x0008

_HEADER = struct.Struct("<2sBBBBHI")
_CRC = struct.Struct("<H")
_IMU_SAMPLE = struct.Struct("<Qhhhhhh")
_MAG = struct.Struct("<QQhhhBB")
_TOF_PREFIX = struct.Struct("<QQBBBBH")
_CLOCK_SYNC = struct.Struct("<QQQ")
_STATUS = struct.Struct("<QIIIIIIIIIHH")


class ProtocolError(ValueError):
    """Base class for malformed/unsupported protocol data."""


class CobsError(ProtocolError):
    pass


class FrameError(ProtocolError):
    pass


class CrcError(FrameError):
    pass


class UnsupportedRecordError(ProtocolError):
    pass


@dataclass(frozen=True)
class Frame:
    protocol_major: int
    protocol_minor: int
    record_type: int
    flags: int
    sequence: int
    payload: bytes


@dataclass(frozen=True)
class ImuSample:
    lsm_tick: int
    accel_x: int
    accel_y: int
    accel_z: int
    gyro_x: int
    gyro_y: int
    gyro_z: int


@dataclass(frozen=True)
class ImuBatch:
    samples: tuple[ImuSample, ...]


@dataclass(frozen=True)
class MagSample:
    mcu_before_us: int
    mcu_after_us: int
    mag_x: int
    mag_y: int
    mag_z: int
    status_reg: int
    read_flags: int


@dataclass(frozen=True)
class TofGrid:
    mcu_ready_us: int
    mcu_read_complete_us: int
    rows: int
    cols: int
    targets_per_zone: int
    layout_id: int
    field_mask: int
    distance_mm: Optional[tuple[int, ...]]
    reflectance_percent: Optional[tuple[int, ...]]
    target_status: Optional[tuple[int, ...]]


@dataclass(frozen=True)
class ClockSync:
    mcu_before_us: int
    lsm_tick: int
    mcu_after_us: int


@dataclass(frozen=True)
class Status:
    mcu_time_us: int
    frames_created: int
    frames_dropped: int
    imu_samples_dropped: int
    fifo_overruns: int
    fifo_structural_errors: int
    mag_retries: int
    mag_errors: int
    tof_errors: int
    clock_sync_errors: int
    queue_high_water: int
    status_flags: int


DecodedRecord = Union[ImuBatch, MagSample, TofGrid, ClockSync, Status]


def crc16_ccitt_false(data: bytes) -> int:
    """CRC-16/CCITT-FALSE: poly=0x1021, init=0xFFFF, refin=false, xorout=0."""
    crc = 0xFFFF
    for byte in data:
        crc ^= byte << 8
        for _ in range(8):
            if crc & 0x8000:
                crc = ((crc << 1) ^ 0x1021) & 0xFFFF
            else:
                crc = (crc << 1) & 0xFFFF
    return crc


def cobs_encode(data: bytes) -> bytes:
    out = bytearray([0])
    code_index = 0
    code = 1

    for byte in data:
        if byte == 0:
            out[code_index] = code
            code_index = len(out)
            out.append(0)
            code = 1
        else:
            out.append(byte)
            code += 1
            if code == 0xFF:
                out[code_index] = code
                code_index = len(out)
                out.append(0)
                code = 1

    out[code_index] = code
    return bytes(out)


def cobs_decode(data: bytes) -> bytes:
    if not data:
        raise CobsError("empty COBS frame")

    out = bytearray()
    index = 0

    while index < len(data):
        code = data[index]
        if code == 0:
            raise CobsError("zero byte inside COBS frame")
        index += 1

        end = index + code - 1
        if end > len(data):
            raise CobsError("COBS block extends beyond frame")

        out.extend(data[index:end])
        index = end

        if code != 0xFF and index < len(data):
            out.append(0)

    return bytes(out)


def encode_frame(record_type: int, sequence: int, payload: bytes, flags: int = 0) -> bytes:
    """Return one complete wire frame, including trailing 0x00 delimiter."""
    if not 0 <= record_type <= 0xFF:
        raise ValueError("record_type must fit uint8")
    if not 0 <= flags <= 0xFF:
        raise ValueError("flags must fit uint8")
    if not 0 <= sequence <= 0xFFFFFFFF:
        raise ValueError("sequence must fit uint32")
    if len(payload) > 0xFFFF:
        raise ValueError("payload too large")

    header = _HEADER.pack(
        MAGIC,
        PROTOCOL_MAJOR,
        PROTOCOL_MINOR,
        record_type,
        flags,
        len(payload),
        sequence,
    )
    body = header + payload
    decoded = body + _CRC.pack(crc16_ccitt_false(body))
    if len(decoded) > MAX_DECODED_FRAME:
        raise ValueError("decoded frame exceeds protocol v0.1 maximum")
    return cobs_encode(decoded) + b"\x00"


def decode_wire_frame(wire_without_delimiter: bytes) -> Frame:
    decoded = cobs_decode(wire_without_delimiter)
    if len(decoded) < _HEADER.size + _CRC.size:
        raise FrameError("decoded frame too short")
    if len(decoded) > MAX_DECODED_FRAME:
        raise FrameError("decoded frame exceeds protocol maximum")

    body = decoded[:-_CRC.size]
    received_crc = _CRC.unpack(decoded[-_CRC.size:])[0]
    calculated_crc = crc16_ccitt_false(body)
    if received_crc != calculated_crc:
        raise CrcError(
            "CRC mismatch: received 0x{:04X}, calculated 0x{:04X}".format(
                received_crc, calculated_crc
            )
        )

    magic, major, minor, record_type, flags, payload_len, sequence = _HEADER.unpack(
        body[:_HEADER.size]
    )
    if magic != MAGIC:
        raise FrameError("wrong magic")
    if major != PROTOCOL_MAJOR:
        raise FrameError(
            "unsupported protocol major {} (expected {})".format(
                major, PROTOCOL_MAJOR
            )
        )

    payload = body[_HEADER.size:]
    if len(payload) != payload_len:
        raise FrameError(
            "payload length mismatch: header {}, actual {}".format(
                payload_len, len(payload)
            )
        )

    return Frame(major, minor, record_type, flags, sequence, payload)


def decode_record(frame: Frame) -> DecodedRecord:
    payload = frame.payload

    if frame.record_type == RECORD_IMU_BATCH:
        if not payload:
            raise FrameError("IMU_BATCH payload is empty")
        count = payload[0]
        if not 1 <= count <= 16:
            raise FrameError("IMU_BATCH sample count must be 1..16")
        expected = 1 + count * _IMU_SAMPLE.size
        if len(payload) != expected:
            raise FrameError("IMU_BATCH payload length mismatch")
        samples = []
        offset = 1
        for _ in range(count):
            values = _IMU_SAMPLE.unpack_from(payload, offset)
            samples.append(ImuSample(*values))
            offset += _IMU_SAMPLE.size
        return ImuBatch(tuple(samples))

    if frame.record_type == RECORD_MAG:
        if len(payload) != _MAG.size:
            raise FrameError("MAG payload length mismatch")
        return MagSample(*_MAG.unpack(payload))

    if frame.record_type == RECORD_TOF_GRID:
        if len(payload) < _TOF_PREFIX.size:
            raise FrameError("TOF_GRID payload too short")

        (
            ready_us,
            complete_us,
            rows,
            cols,
            targets_per_zone,
            layout_id,
            field_mask,
        ) = _TOF_PREFIX.unpack_from(payload, 0)

        if rows == 0 or cols == 0 or rows * cols > 64:
            raise FrameError("TOF_GRID rows*cols must be 1..64 in v0.1")
        if targets_per_zone != 1:
            raise FrameError("TOF_GRID v0.1 requires targets_per_zone=1")
        if field_mask & ~TOF_KNOWN_FIELD_MASK:
            raise UnsupportedRecordError("TOF_GRID contains unknown field-mask bits")

        zones = rows * cols
        target_values = zones * targets_per_zone
        offset = _TOF_PREFIX.size

        def read_u16(count: int) -> tuple[int, ...]:
            nonlocal offset
            size = count * 2
            if offset + size > len(payload):
                raise FrameError("TOF_GRID truncated uint16 array")
            values = struct.unpack_from("<{}H".format(count), payload, offset)
            offset += size
            return tuple(values)

        def read_u8(count: int) -> tuple[int, ...]:
            nonlocal offset
            if offset + count > len(payload):
                raise FrameError("TOF_GRID truncated uint8 array")
            values = tuple(payload[offset:offset + count])
            offset += count
            return values

        distance = (
            read_u16(target_values)
            if field_mask & TOF_FIELD_DISTANCE_MM
            else None
        )
        reflectance = (
            read_u8(target_values)
            if field_mask & TOF_FIELD_REFLECTANCE_PERCENT
            else None
        )
        target_status = (
            read_u8(target_values)
            if field_mask & TOF_FIELD_TARGET_STATUS
            else None
        )
        if offset != len(payload):
            raise FrameError("TOF_GRID has trailing bytes")

        return TofGrid(
            ready_us,
            complete_us,
            rows,
            cols,
            targets_per_zone,
            layout_id,
            field_mask,
            distance,
            reflectance,
            target_status,
        )

    if frame.record_type == RECORD_CLOCK_SYNC:
        if len(payload) != _CLOCK_SYNC.size:
            raise FrameError("CLOCK_SYNC payload length mismatch")
        return ClockSync(*_CLOCK_SYNC.unpack(payload))

    if frame.record_type == RECORD_STATUS:
        if len(payload) != _STATUS.size:
            raise FrameError("STATUS payload length mismatch")
        return Status(*_STATUS.unpack(payload))

    raise UnsupportedRecordError(
        "unsupported record type 0x{:02X}".format(frame.record_type)
    )


class StreamDecoder:
    """Incremental 0x00-delimited decoder with corruption recovery."""

    def __init__(self, max_encoded_frame: int = MAX_DECODED_FRAME + 8):
        self._buffer = bytearray()
        self.max_encoded_frame = max_encoded_frame
        self.frames_ok = 0
        self.frames_bad = 0
        self.empty_delimiters = 0
        self._discarding = False

    def feed(self, data: bytes) -> list[Frame]:
        frames: list[Frame] = []
        for byte in data:
            if byte == 0:
                if self._discarding:
                    self._discarding = False
                    self._buffer.clear()
                    continue
                if not self._buffer:
                    self.empty_delimiters += 1
                    continue
                try:
                    frame = decode_wire_frame(bytes(self._buffer))
                except ProtocolError:
                    self.frames_bad += 1
                else:
                    self.frames_ok += 1
                    frames.append(frame)
                self._buffer.clear()
                continue

            if self._discarding:
                continue

            if len(self._buffer) >= self.max_encoded_frame:
                # Drop the oversize partial frame and ignore bytes until delimiter.
                self._buffer.clear()
                self._discarding = True
                self.frames_bad += 1
                continue
            self._buffer.append(byte)

        return frames
