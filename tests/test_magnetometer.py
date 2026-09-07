"""Tests for LIS3MDL configuration decoding and raw-field diagnostics."""

from pathlib import Path
import sys
import unittest

HERE = Path(__file__).resolve().parent
REPO_ROOT = HERE.parent
HOST_PYTHON = REPO_ROOT / "host" / "python"
if str(HOST_PYTHON) not in sys.path:
    sys.path.insert(0, str(HOST_PYTHON))

import rangeweave_magnetometer as mag
import rangeweave_protocol as rw


class MagnetometerConfigTests(unittest.TestCase):
    def test_reference_config_decodes_without_hard_coded_runtime_assumption(self):
        config = mag.decode_lis3mdl_config(bytes.fromhex("74 00 00 0C 40"))
        self.assertEqual(config.full_scale_gauss, 4.0)
        self.assertEqual(config.sensitivity_lsb_per_gauss, 6842.0)
        self.assertEqual(config.output_data_rate_hz, 20.0)
        self.assertEqual(config.xy_performance_mode, "ultra-high-performance")
        self.assertEqual(config.z_performance_mode, "ultra-high-performance")
        self.assertEqual(config.measurement_mode, "continuous-conversion")
        self.assertFalse(config.low_power)
        self.assertTrue(config.block_data_update)

    def test_full_scale_codes_select_documented_sensitivities(self):
        expected = {
            0: (4.0, 6842.0),
            1: (8.0, 3421.0),
            2: (12.0, 2281.0),
            3: (16.0, 1711.0),
        }
        for code, pair in expected.items():
            ctrl2 = code << 5
            config = mag.decode_lis3mdl_config(bytes([0x10, ctrl2, 0x00, 0x00, 0x00]))
            self.assertEqual((config.full_scale_gauss, config.sensitivity_lsb_per_gauss), pair)

    def test_snapshot_must_contain_exactly_five_registers(self):
        with self.assertRaises(mag.MagnetometerError):
            mag.decode_lis3mdl_config(b"\x00\x00")


class MagnetometerSampleTests(unittest.TestCase):
    def setUp(self):
        self.config = mag.decode_lis3mdl_config(bytes.fromhex("74 00 00 0C 40"))

    def test_raw_counts_convert_to_microtesla(self):
        self.assertAlmostEqual(mag.raw_to_microtesla(6842, self.config), 100.0)
        self.assertAlmostEqual(mag.raw_to_microtesla(-3421, self.config), -50.0)

    def test_sample_uses_midpoint_of_read_bracket(self):
        raw = rw.MagSample(
            mcu_before_us=1000,
            mcu_after_us=1040,
            mag_x=6842,
            mag_y=0,
            mag_z=0,
            status_reg=0x08,
            read_flags=rw.MAG_FLAG_RETRY_USED,
        )
        converted = mag.convert_sample(raw, self.config)
        self.assertEqual(converted.time_us, 1020.0)
        self.assertEqual(converted.read_duration_us, 40)
        self.assertAlmostEqual(converted.x_ut, 100.0)
        self.assertAlmostEqual(converted.norm_ut, 100.0)
        self.assertTrue(converted.retry_used)
        self.assertTrue(converted.data_ready)
        self.assertFalse(converted.overrun)

    def test_negative_read_bracket_is_rejected(self):
        raw = rw.MagSample(200, 100, 0, 0, 0, 0x08, 0)
        with self.assertRaises(mag.MagnetometerError):
            mag.convert_sample(raw, self.config)

    def test_summary_reports_timing_ranges_and_flags(self):
        raw_samples = (
            rw.MagSample(0, 20, -1000, 0, 1000, 0x08, 0),
            rw.MagSample(100000, 100020, 0, 1000, 0, 0x88, rw.MAG_FLAG_RETRY_USED),
            rw.MagSample(200000, 200020, 1000, 0, -1000, 0x00, 0),
        )
        samples = mag.convert_samples(raw_samples, self.config)
        summary = mag.summarise(samples, self.config)
        self.assertEqual(summary.sample_count, 3)
        self.assertAlmostEqual(summary.span_s, 0.2)
        self.assertAlmostEqual(summary.observed_rate_hz, 10.0)
        self.assertAlmostEqual(summary.median_interval_ms, 100.0)
        self.assertEqual(summary.retry_count, 1)
        self.assertEqual(summary.not_ready_count, 1)
        self.assertEqual(summary.overrun_count, 1)
        self.assertGreater(summary.axis_span_ut[0], 0.0)
        self.assertGreater(summary.norm_max_ut, 0.0)


if __name__ == "__main__":
    unittest.main()
