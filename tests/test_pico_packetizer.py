"""Conformance of the MicroPython-side encoder with shared protocol v0.1 fixtures."""

import importlib.util
import json
from pathlib import Path
import unittest

HERE = Path(__file__).resolve().parent
REPO_ROOT = HERE.parent
MODULE_PATH = REPO_ROOT / "firmware" / "pico2w" / "acquisition" / "rw_protocol.py"
FIXTURE_PATH = REPO_ROOT / "protocol" / "test-vectors" / "v0.1.json"

spec = importlib.util.spec_from_file_location("pico_rw_protocol", MODULE_PATH)
rw = importlib.util.module_from_spec(spec)
spec.loader.exec_module(rw)


class PicoPacketizerTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.fixture = json.loads(FIXTURE_PATH.read_text(encoding="utf-8"))

    def _payload_for(self, vector):
        record = vector["expected_record"]
        record_type = vector["record_type"]

        if record_type == rw.RECORD_IMU_BATCH:
            samples = []
            for item in record["samples"]:
                samples.append((
                    item["lsm_tick"],
                    item["accel_x"], item["accel_y"], item["accel_z"],
                    item["gyro_x"], item["gyro_y"], item["gyro_z"],
                ))
            return rw.pack_imu_batch(samples)

        if record_type == rw.RECORD_MAG:
            return rw.pack_mag(
                record["mcu_before_us"], record["mcu_after_us"],
                record["mag_x"], record["mag_y"], record["mag_z"],
                record["status_reg"], record["read_flags"],
            )

        if record_type == rw.RECORD_TOF_GRID:
            return rw.pack_tof_grid(
                record["mcu_ready_us"], record["mcu_read_complete_us"],
                record["rows"], record["cols"],
                record["distance_mm"],
                record["reflectance_percent"],
                record["target_status"],
                record["layout_id"],
            )

        if record_type == rw.RECORD_CLOCK_SYNC:
            return rw.pack_clock_sync(
                record["mcu_before_us"], record["lsm_tick"], record["mcu_after_us"]
            )

        if record_type == rw.RECORD_STATUS:
            return rw.pack_status(
                record["mcu_time_us"], record["frames_created"],
                record["frames_dropped"], record["imu_samples_dropped"],
                record["fifo_overruns"], record["fifo_structural_errors"],
                record["mag_retries"], record["mag_errors"],
                record["tof_errors"], record["clock_sync_errors"],
                record["queue_high_water"], record["status_flags"],
            )

        if record_type == rw.RECORD_STREAM_INFO:
            items = []
            for item in record["tlvs"]:
                items.append((item["tag"], bytes.fromhex(item["value"]["hex"])))
            return rw.pack_stream_info(
                record["session_id"], record["info_revision"], items
            )

        self.fail("fixture contains unsupported record type {}".format(record_type))

    def test_all_golden_frames(self):
        for vector in self.fixture["vectors"]:
            with self.subTest(vector=vector["name"]):
                payload = self._payload_for(vector)
                wire = rw.encode_frame(vector["record_type"], vector["sequence"], payload)
                self.assertEqual(wire.hex(), vector["wire_hex"])

    def test_crc_check_value(self):
        self.assertEqual(rw.crc16_ccitt_false(b"123456789"), 0x29B1)


if __name__ == "__main__":
    unittest.main()
