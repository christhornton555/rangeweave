"""MicroPython packet encoder for Rangeweave wire protocol v0.1.

This module intentionally contains no USB or sensor logic. It translates semantic
records into the transport-independent byte frames defined by protocol/spec-v0.1.md.
"""

import struct

MAGIC = b"RW"
PROTOCOL_MAJOR = 0
PROTOCOL_MINOR = 1
MAX_DECODED_FRAME = 1024

RECORD_IMU_BATCH = 0x01
RECORD_MAG = 0x02
RECORD_TOF_GRID = 0x03
RECORD_CLOCK_SYNC = 0x04
RECORD_STATUS = 0x05
RECORD_STREAM_INFO = 0x06

TOF_FIELD_DISTANCE_MM = 0x0001
TOF_FIELD_REFLECTANCE_PERCENT = 0x0002
TOF_FIELD_TARGET_STATUS = 0x0004

INFO_FIRMWARE_LABEL = 0x01
INFO_SOURCE_PROFILE = 0x02
INFO_LSM_WHOAMI = 0x10
INFO_MAG_WHOAMI = 0x11
INFO_TOF_I2C_ADDRESS = 0x12
INFO_LSM_FREQ_FINE = 0x13
INFO_LSM_CTRL1_XL = 0x14
INFO_LSM_CTRL2_G = 0x15
INFO_LSM_FIFO_CTRL3 = 0x16
INFO_LSM_FIFO_CTRL4 = 0x17
INFO_MAG_CTRL_REGS_1_TO_5 = 0x20
INFO_TOF_GRID_CONFIG = 0x30
INFO_TOF_DEFAULT_FIELD_MASK = 0x31

MAG_FLAG_RETRY_USED = 0x01

STATUS_FLAG_ACQUISITION_READY = 0x0001
STATUS_FLAG_CLOCK_SYNC_ACTIVE = 0x0002
STATUS_FLAG_FIFO_OVERRUN_LATCHED = 0x0004
STATUS_FLAG_TRANSPORT_BACKPRESSURE = 0x0008

# 4-bit lookup table for CRC-16/CCITT-FALSE (poly 0x1021). A 16-entry
# table keeps MCU memory cost small while reducing the hot path from eight
# Python bit-iterations per byte to two nibble steps. Wire bytes are unchanged.
_CRC16_NIBBLE = (
    0x0000, 0x1021, 0x2042, 0x3063,
    0x4084, 0x50A5, 0x60C6, 0x70E7,
    0x8108, 0x9129, 0xA14A, 0xB16B,
    0xC18C, 0xD1AD, 0xE1CE, 0xF1EF,
)


def crc16_ccitt_false(data):
    crc = 0xFFFF
    for byte in data:
        crc ^= byte << 8
        crc = ((crc << 4) & 0xFFFF) ^ _CRC16_NIBBLE[(crc >> 12) & 0x0F]
        crc = ((crc << 4) & 0xFFFF) ^ _CRC16_NIBBLE[(crc >> 12) & 0x0F]
    return crc


def cobs_encode(data):
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


def encode_frame(record_type, sequence, payload, flags=0):
    if len(payload) > 0xFFFF:
        raise ValueError("payload too large")

    header = struct.pack(
        "<2sBBBBHI",
        MAGIC,
        PROTOCOL_MAJOR,
        PROTOCOL_MINOR,
        record_type,
        flags,
        len(payload),
        sequence & 0xFFFFFFFF,
    )
    body = header + payload
    decoded = body + struct.pack("<H", crc16_ccitt_false(body))

    if len(decoded) > MAX_DECODED_FRAME:
        raise ValueError("decoded frame exceeds protocol v0.1 maximum")

    return cobs_encode(decoded) + b"\x00"


def pack_imu_batch(samples):
    count = len(samples)
    if count < 1 or count > 16:
        raise ValueError("IMU batch must contain 1..16 samples")

    payload = bytearray(1 + count * 20)
    payload[0] = count
    offset = 1

    for sample in samples:
        struct.pack_into(
            "<Qhhhhhh",
            payload,
            offset,
            sample[0],
            sample[1],
            sample[2],
            sample[3],
            sample[4],
            sample[5],
            sample[6],
        )
        offset += 20

    return bytes(payload)


def pack_mag(mcu_before_us, mcu_after_us, x, y, z, status_reg, read_flags):
    return struct.pack(
        "<QQhhhBB",
        mcu_before_us,
        mcu_after_us,
        x,
        y,
        z,
        status_reg,
        read_flags,
    )


def pack_tof_grid(
    mcu_ready_us,
    mcu_read_complete_us,
    rows,
    cols,
    distances=None,
    reflectance=None,
    target_status=None,
    layout_id=0,
):
    zones = rows * cols
    if zones < 1 or zones > 64:
        raise ValueError("TOF grid must contain 1..64 zones")

    field_mask = 0
    if distances is not None:
        if len(distances) != zones:
            raise ValueError("distance array length mismatch")
        field_mask |= TOF_FIELD_DISTANCE_MM
    if reflectance is not None:
        if len(reflectance) != zones:
            raise ValueError("reflectance array length mismatch")
        field_mask |= TOF_FIELD_REFLECTANCE_PERCENT
    if target_status is not None:
        if len(target_status) != zones:
            raise ValueError("target-status array length mismatch")
        field_mask |= TOF_FIELD_TARGET_STATUS

    payload_len = 22
    if distances is not None:
        payload_len += zones * 2
    if reflectance is not None:
        payload_len += zones
    if target_status is not None:
        payload_len += zones

    payload = bytearray(payload_len)
    struct.pack_into(
        "<QQBBBBH",
        payload,
        0,
        mcu_ready_us,
        mcu_read_complete_us,
        rows,
        cols,
        1,
        layout_id,
        field_mask,
    )
    offset = 22

    if distances is not None:
        for value in distances:
            value = int(value)
            if value < 0 or value > 0xFFFF:
                raise ValueError("distance value outside uint16 range")
            struct.pack_into("<H", payload, offset, value)
            offset += 2

    if reflectance is not None:
        for value in reflectance:
            value = int(value)
            if value < 0 or value > 0xFF:
                raise ValueError("reflectance value outside uint8 range")
            payload[offset] = value
            offset += 1

    if target_status is not None:
        for value in target_status:
            value = int(value)
            if value < 0 or value > 0xFF:
                raise ValueError("target-status value outside uint8 range")
            payload[offset] = value
            offset += 1

    return bytes(payload)


def pack_clock_sync(mcu_before_us, lsm_tick, mcu_after_us):
    return struct.pack("<QQQ", mcu_before_us, lsm_tick, mcu_after_us)


def pack_status(
    mcu_time_us,
    frames_created,
    frames_dropped,
    imu_samples_dropped,
    fifo_overruns,
    fifo_structural_errors,
    mag_retries,
    mag_errors,
    tof_errors,
    clock_sync_errors,
    queue_high_water,
    status_flags,
):
    return struct.pack(
        "<QIIIIIIIIIHH",
        mcu_time_us,
        frames_created & 0xFFFFFFFF,
        frames_dropped & 0xFFFFFFFF,
        imu_samples_dropped & 0xFFFFFFFF,
        fifo_overruns & 0xFFFFFFFF,
        fifo_structural_errors & 0xFFFFFFFF,
        mag_retries & 0xFFFFFFFF,
        mag_errors & 0xFFFFFFFF,
        tof_errors & 0xFFFFFFFF,
        clock_sync_errors & 0xFFFFFFFF,
        queue_high_water & 0xFFFF,
        status_flags & 0xFFFF,
    )


def tlv(tag, value):
    if len(value) > 255:
        raise ValueError("STREAM_INFO TLV value too long")
    return bytes([tag & 0xFF, len(value)]) + bytes(value)


def pack_stream_info(session_id, info_revision, items):
    payload = bytearray(struct.pack("<QH", session_id, info_revision & 0xFFFF))
    for tag, value in items:
        payload.extend(tlv(tag, value))
    return bytes(payload)
