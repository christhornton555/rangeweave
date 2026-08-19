"""Rangeweave Pico 2 W acquisition firmware, experimental v0.1.

Successful operation produces binary Rangeweave protocol frames only. USB is merely the
first transport adapter; sensor acquisition and packet semantics are kept independent.
"""

import os
import struct
import time

import rw_protocol as rw
from rw_sensors import (
    SensorStack,
    LSM_TAG_GYRO,
    LSM_TAG_ACCEL,
    LSM_TAG_TIMESTAMP,
    LSM_FIFO_BDR_CODE,
    TOF_ADDR,
    TOF_ROWS,
    TOF_COLS,
    TOF_HZ,
)
from rw_transport_usb import FrameQueue, UsbCdcTransport
from rw_timing import LsmTickExtender

FIRMWARE_LABEL = b"rangeweave-pico2w-acq-0.1-exp1"
SOURCE_PROFILE = b"pico2w-lsm6dsox-lis3mdl-vl53l5cx-8x8-15hz"

MAG_PERIOD_US = 100_000
MAG_INITIAL_OFFSET_US = 5_000
FIFO_SERVICE_PERIOD_US = 10_000
FIFO_MAX_RECORDS_PER_SERVICE = 192
TOF_POLL_US = 5_000
CLOCK_SYNC_PERIOD_US = 1_000_000
CLOCK_SYNC_INITIAL_US = 50_000
STATUS_PERIOD_US = 1_000_000
STREAM_INFO_PERIOD_US = 10_000_000

FIFO_MAG_GUARD_US = 3_000
TOF_MAG_GUARD_US = 25_000
CLOCK_SYNC_MAG_GUARD_US = 3_000

IMU_BATCH_SAMPLES = 4
IMU_BATCH_MAX_LATENCY_US = 50_000
FRAME_QUEUE_CAPACITY = 32
USB_WRITE_CHUNK = 64

SLOT_GYRO = 0x01
SLOT_ACCEL = 0x02
SLOT_TIMESTAMP = 0x04
SLOT_COMPLETE = SLOT_GYRO | SLOT_ACCEL | SLOT_TIMESTAMP


class Counters:
    def __init__(self):
        self.frames_created = 0
        self.frames_dropped = 0
        self.imu_samples_dropped = 0
        self.fifo_overruns = 0
        self.fifo_structural_errors = 0
        self.mag_retries = 0
        self.mag_errors = 0
        self.tof_errors = 0
        self.clock_sync_errors = 0
        self.fifo_overrun_latched = False
        self.clock_sync_active = False


class FifoSlotAssembler:
    def __init__(self, tick_extender, counters):
        self.tick_extender = tick_extender
        self.counters = counters
        self.slot_cnt = None
        self.mask = 0
        self.slot_groups_seen = 0
        self.emitted = False
        self.accel = None
        self.gyro = None
        self.timestamp = None

    def _new_slot(self, slot_cnt):
        self.slot_cnt = slot_cnt
        self.mask = 0
        self.emitted = False
        self.accel = None
        self.gyro = None
        self.timestamp = None

    def _finish_previous(self, new_slot_cnt):
        jump = (new_slot_cnt - self.slot_cnt) & 0x03
        if jump != 1:
            self.counters.fifo_structural_errors += 1
            if jump > 1:
                self.counters.imu_samples_dropped += jump - 1

        if self.slot_groups_seen > 0 and not self.emitted:
            if self.mask != 0:
                self.counters.fifo_structural_errors += 1
                self.counters.imu_samples_dropped += 1

        self.slot_groups_seen += 1

    def accept(self, tag, slot_cnt, raw):
        if self.slot_cnt is None:
            self._new_slot(slot_cnt)
        elif slot_cnt != self.slot_cnt:
            self._finish_previous(slot_cnt)
            self._new_slot(slot_cnt)

        if tag == LSM_TAG_GYRO:
            if self.mask & SLOT_GYRO:
                self.counters.fifo_structural_errors += 1
            self.gyro = struct.unpack("<hhh", raw)
            self.mask |= SLOT_GYRO

        elif tag == LSM_TAG_ACCEL:
            if self.mask & SLOT_ACCEL:
                self.counters.fifo_structural_errors += 1
            self.accel = struct.unpack("<hhh", raw)
            self.mask |= SLOT_ACCEL

        elif tag == LSM_TAG_TIMESTAMP:
            if self.mask & SLOT_TIMESTAMP:
                self.counters.fifo_structural_errors += 1

            raw_tick = raw[0] | (raw[1] << 8) | (raw[2] << 16) | (raw[3] << 24)
            bdr_shub = raw[4] & 0x0F
            bdr_xl = raw[5] & 0x0F
            bdr_gy = (raw[5] >> 4) & 0x0F
            if not (
                bdr_shub == 0
                and bdr_xl == LSM_FIFO_BDR_CODE
                and bdr_gy == LSM_FIFO_BDR_CODE
            ):
                self.counters.fifo_structural_errors += 1
                self.timestamp = None
            else:
                self.timestamp = self.tick_extender.extend(raw_tick)
            self.mask |= SLOT_TIMESTAMP

        else:
            self.counters.fifo_structural_errors += 1
            return None

        if self.mask == SLOT_COMPLETE and not self.emitted:
            self.emitted = True
            if self.timestamp is None or self.accel is None or self.gyro is None:
                self.counters.imu_samples_dropped += 1
                return None
            return (
                self.timestamp,
                self.accel[0],
                self.accel[1],
                self.accel[2],
                self.gyro[0],
                self.gyro[1],
                self.gyro[2],
            )

        return None


class Acquisition:
    def __init__(self):
        self.stack = SensorStack()
        self.stack.initialise()

        self.counters = Counters()
        self.tick_extender = LsmTickExtender()
        self.assembler = FifoSlotAssembler(self.tick_extender, self.counters)
        self.queue = FrameQueue(FRAME_QUEUE_CAPACITY)
        self.transport = UsbCdcTransport(USB_WRITE_CHUNK)

        self.sequence = 0
        self.info_revision = 0
        self.session_id = self._make_session_id()
        self.imu_batch = []
        self.imu_batch_started_us = None

        self.config = self.stack.config_snapshot()
        self._fifo_overrun_active = False

    def _make_session_id(self):
        try:
            random_bytes = os.urandom(8)
            return struct.unpack("<Q", random_bytes)[0]
        except Exception:
            # Ephemeral fallback only; deliberately do not use machine.unique_id().
            now = self.stack.clock.now_us() & 0xFFFFFFFF
            return ((now << 32) ^ (time.ticks_us() & 0xFFFFFFFF)) & 0xFFFFFFFFFFFFFFFF

    def _reserve_sequence(self):
        sequence = self.sequence
        self.sequence = (self.sequence + 1) & 0xFFFFFFFF
        self.counters.frames_created = (self.counters.frames_created + 1) & 0xFFFFFFFF
        return sequence

    def enqueue_payload(self, record_type, payload, imu_samples=0):
        sequence = self._reserve_sequence()
        if self.queue.full():
            self.counters.frames_dropped = (self.counters.frames_dropped + 1) & 0xFFFFFFFF
            if imu_samples:
                self.counters.imu_samples_dropped = (
                    self.counters.imu_samples_dropped + imu_samples
                ) & 0xFFFFFFFF
            return False

        frame = rw.encode_frame(record_type, sequence, payload)
        if not self.queue.push(frame):
            self.counters.frames_dropped = (self.counters.frames_dropped + 1) & 0xFFFFFFFF
            if imu_samples:
                self.counters.imu_samples_dropped = (
                    self.counters.imu_samples_dropped + imu_samples
                ) & 0xFFFFFFFF
            return False
        return True

    def emit_stream_info(self):
        ctrl1, ctrl2, fifo3, fifo4, mag_regs = self.config
        items = [
            (rw.INFO_FIRMWARE_LABEL, FIRMWARE_LABEL),
            (rw.INFO_SOURCE_PROFILE, SOURCE_PROFILE),
            (rw.INFO_LSM_WHOAMI, bytes([self.stack.lsm_whoami])),
            (rw.INFO_MAG_WHOAMI, bytes([self.stack.mag_whoami])),
            (rw.INFO_TOF_I2C_ADDRESS, bytes([TOF_ADDR])),
            (rw.INFO_LSM_FREQ_FINE, struct.pack("<b", self.stack.freq_fine)),
            (rw.INFO_LSM_CTRL1_XL, bytes([ctrl1])),
            (rw.INFO_LSM_CTRL2_G, bytes([ctrl2])),
            (rw.INFO_LSM_FIFO_CTRL3, bytes([fifo3])),
            (rw.INFO_LSM_FIFO_CTRL4, bytes([fifo4])),
            (rw.INFO_MAG_CTRL_REGS_1_TO_5, mag_regs),
            (rw.INFO_TOF_GRID_CONFIG, bytes([TOF_ROWS, TOF_COLS, TOF_HZ])),
            (
                rw.INFO_TOF_DEFAULT_FIELD_MASK,
                struct.pack(
                    "<H",
                    rw.TOF_FIELD_DISTANCE_MM | rw.TOF_FIELD_REFLECTANCE_PERCENT,
                ),
            ),
        ]
        payload = rw.pack_stream_info(self.session_id, self.info_revision, items)
        self.enqueue_payload(rw.RECORD_STREAM_INFO, payload)

    def emit_status(self, now_us):
        flags = rw.STATUS_FLAG_ACQUISITION_READY
        if self.counters.clock_sync_active:
            flags |= rw.STATUS_FLAG_CLOCK_SYNC_ACTIVE
        if self.counters.fifo_overrun_latched:
            flags |= rw.STATUS_FLAG_FIFO_OVERRUN_LATCHED

        backpressure = self.queue.full() or len(self.queue) >= (FRAME_QUEUE_CAPACITY * 3 // 4)
        if backpressure:
            flags |= rw.STATUS_FLAG_TRANSPORT_BACKPRESSURE

        payload = rw.pack_status(
            now_us,
            self.counters.frames_created,
            self.counters.frames_dropped,
            self.counters.imu_samples_dropped,
            self.counters.fifo_overruns,
            self.counters.fifo_structural_errors,
            self.counters.mag_retries,
            self.counters.mag_errors,
            self.counters.tof_errors,
            self.counters.clock_sync_errors,
            self.queue.high_water,
            flags,
        )
        self.enqueue_payload(rw.RECORD_STATUS, payload)

    def add_imu_sample(self, sample, now_us):
        if not self.imu_batch:
            self.imu_batch_started_us = now_us
        self.imu_batch.append(sample)
        if len(self.imu_batch) >= IMU_BATCH_SAMPLES:
            self.flush_imu_batch()

    def flush_imu_batch(self):
        if not self.imu_batch:
            return
        samples = self.imu_batch
        self.imu_batch = []
        self.imu_batch_started_us = None
        payload = rw.pack_imu_batch(samples)
        self.enqueue_payload(rw.RECORD_IMU_BATCH, payload, len(samples))

    def service_fifo(self):
        try:
            level, status2 = self.stack.fifo_status()
        except OSError:
            self.counters.fifo_structural_errors += 1
            return

        overrun_now = bool(status2 & 0x48)
        if overrun_now and not self._fifo_overrun_active:
            self.counters.fifo_overruns = (
                self.counters.fifo_overruns + 1
            ) & 0xFFFFFFFF
            self.counters.fifo_overrun_latched = True
        self._fifo_overrun_active = overrun_now

        records_to_read = level
        if records_to_read > FIFO_MAX_RECORDS_PER_SERVICE:
            records_to_read = FIFO_MAX_RECORDS_PER_SERVICE

        try:
            for _ in range(records_to_read):
                tag, slot_cnt, raw = self.stack.read_fifo_record()
                sample = self.assembler.accept(tag, slot_cnt, raw)
                if sample is not None:
                    self.add_imu_sample(sample, self.stack.clock.now_us())
        except OSError:
            self.counters.fifo_structural_errors += 1

    def service_mag(self):
        state, sample, retried = self.stack.read_mag()
        if retried:
            self.counters.mag_retries = (
                self.counters.mag_retries + 1
            ) & 0xFFFFFFFF
        if state == "ok":
            before_us, after_us, x, y, z, status, flags = sample
            payload = rw.pack_mag(before_us, after_us, x, y, z, status, flags)
            self.enqueue_payload(rw.RECORD_MAG, payload)
        elif state == "error":
            self.counters.mag_errors = (self.counters.mag_errors + 1) & 0xFFFFFFFF

    def service_tof(self):
        try:
            sample = self.stack.read_tof()
        except (OSError, RuntimeError):
            self.counters.tof_errors = (self.counters.tof_errors + 1) & 0xFFFFFFFF
            return
        if sample is None:
            return

        ready_us, complete_us, distances, reflectance = sample
        try:
            payload = rw.pack_tof_grid(
                ready_us,
                complete_us,
                TOF_ROWS,
                TOF_COLS,
                distances=distances,
                reflectance=reflectance,
            )
        except ValueError:
            self.counters.tof_errors = (self.counters.tof_errors + 1) & 0xFFFFFFFF
            return
        self.enqueue_payload(rw.RECORD_TOF_GRID, payload)

    def service_clock_sync(self):
        try:
            before_us, lsm_tick, after_us = self.stack.read_clock_sync(self.tick_extender)
        except OSError:
            self.counters.clock_sync_errors = (
                self.counters.clock_sync_errors + 1
            ) & 0xFFFFFFFF
            return

        self.counters.clock_sync_active = True
        payload = rw.pack_clock_sync(before_us, lsm_tick, after_us)
        self.enqueue_payload(rw.RECORD_CLOCK_SYNC, payload)

    def run(self):
        now_us = self.stack.clock.now_us()
        next_mag_us = now_us + MAG_INITIAL_OFFSET_US
        next_fifo_us = now_us
        next_tof_poll_us = now_us
        next_sync_us = now_us + CLOCK_SYNC_INITIAL_US
        next_status_us = now_us + STATUS_PERIOD_US
        next_info_us = now_us + STREAM_INFO_PERIOD_US

        self.emit_stream_info()

        while True:
            now_us = self.stack.clock.now_us()

            if now_us >= next_mag_us:
                self.service_mag()
                next_mag_us += MAG_PERIOD_US
                now_after = self.stack.clock.now_us()
                if now_after - next_mag_us > MAG_PERIOD_US:
                    next_mag_us = now_after + MAG_PERIOD_US

            now_us = self.stack.clock.now_us()
            if now_us >= next_fifo_us:
                time_to_mag = next_mag_us - now_us
                time_since_mag = now_us - self.stack.last_mag_bus_activity_us
                if time_to_mag > FIFO_MAG_GUARD_US and time_since_mag > FIFO_MAG_GUARD_US:
                    self.service_fifo()
                    next_fifo_us += FIFO_SERVICE_PERIOD_US
                    now_after = self.stack.clock.now_us()
                    if now_after - next_fifo_us > FIFO_SERVICE_PERIOD_US:
                        next_fifo_us = now_after + FIFO_SERVICE_PERIOD_US

            now_us = self.stack.clock.now_us()
            if now_us >= next_tof_poll_us:
                next_tof_poll_us = now_us + TOF_POLL_US
                if next_mag_us - now_us > TOF_MAG_GUARD_US:
                    self.service_tof()

            now_us = self.stack.clock.now_us()
            if now_us >= next_mag_us:
                self.service_mag()
                next_mag_us += MAG_PERIOD_US

            now_us = self.stack.clock.now_us()
            if now_us >= next_sync_us:
                time_to_mag = next_mag_us - now_us
                time_since_mag = now_us - self.stack.last_mag_bus_activity_us
                if (
                    time_to_mag > CLOCK_SYNC_MAG_GUARD_US
                    and time_since_mag > CLOCK_SYNC_MAG_GUARD_US
                ):
                    self.service_clock_sync()
                    next_sync_us += CLOCK_SYNC_PERIOD_US

            now_us = self.stack.clock.now_us()
            if now_us >= next_fifo_us:
                time_to_mag = next_mag_us - now_us
                time_since_mag = now_us - self.stack.last_mag_bus_activity_us
                if time_to_mag > FIFO_MAG_GUARD_US and time_since_mag > FIFO_MAG_GUARD_US:
                    self.service_fifo()
                    next_fifo_us = now_us + FIFO_SERVICE_PERIOD_US

            now_us = self.stack.clock.now_us()
            if self.imu_batch and self.imu_batch_started_us is not None:
                if now_us - self.imu_batch_started_us >= IMU_BATCH_MAX_LATENCY_US:
                    self.flush_imu_batch()

            if now_us >= next_status_us:
                self.emit_status(now_us)
                next_status_us += STATUS_PERIOD_US

            if now_us >= next_info_us:
                self.emit_stream_info()
                next_info_us += STREAM_INFO_PERIOD_US

            # One bounded transport service per loop. Sensor scheduling remains able to
            # run while USB is temporarily not writable; queue overflow becomes an
            # explicit sequence gap + STATUS counter instead of hidden timing damage.
            self.transport.service(self.queue)

            time.sleep_ms(1)


def main():
    acquisition = Acquisition()
    acquisition.run()


if __name__ == "__main__":
    main()
