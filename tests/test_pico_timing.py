"""Tests for acquisition-side LSM timestamp extension."""

import importlib.util
from pathlib import Path
import unittest

HERE = Path(__file__).resolve().parent
REPO_ROOT = HERE.parent
MODULE_PATH = REPO_ROOT / "firmware" / "pico2w" / "acquisition" / "rw_timing.py"

spec = importlib.util.spec_from_file_location("pico_timing", MODULE_PATH)
timing = importlib.util.module_from_spec(spec)
spec.loader.exec_module(timing)


class LsmTickExtenderTests(unittest.TestCase):
    def test_wrap_and_older_fifo_observation(self):
        ext = timing.LsmTickExtender()
        self.assertEqual(ext.extend(0xFFFFFF00), 0xFFFFFF00)
        self.assertEqual(ext.extend(0x00000100), 0x100000100)
        self.assertEqual(ext.extend(0xFFFFFF80), 0xFFFFFF80)
        self.assertEqual(ext.anchor_extended, 0x100000100)
        self.assertEqual(ext.extend(0x00000180), 0x100000180)
        self.assertEqual(ext.anchor_extended, 0x100000180)

    def test_simple_monotonic_progress(self):
        ext = timing.LsmTickExtender()
        self.assertEqual(ext.extend(1000), 1000)
        self.assertEqual(ext.extend(1384), 1384)
        self.assertEqual(ext.extend(1768), 1768)


if __name__ == "__main__":
    unittest.main()
