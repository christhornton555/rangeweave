"""Host-side regression tests for the Pico USB transport queue/drain policy."""

import importlib.util
from pathlib import Path
import unittest

HERE = Path(__file__).resolve().parent
REPO_ROOT = HERE.parent
MODULE_PATH = (
    REPO_ROOT / "firmware" / "pico2w" / "acquisition" / "rw_transport_usb.py"
)

spec = importlib.util.spec_from_file_location("pico_rw_transport_usb", MODULE_PATH)
transport_mod = importlib.util.module_from_spec(spec)
spec.loader.exec_module(transport_mod)


class FakeStream:
    def __init__(self):
        self.data = bytearray()
        self.write_calls = 0

    def write(self, data):
        chunk = bytes(data)
        self.write_calls += 1
        self.data.extend(chunk)
        return len(chunk)


class PicoTransportTests(unittest.TestCase):
    def test_frame_queue_is_fixed_capacity_fifo(self):
        queue = transport_mod.FrameQueue(2)
        self.assertTrue(queue.push(b"a"))
        self.assertTrue(queue.push(b"b"))
        self.assertFalse(queue.push(b"c"))
        self.assertEqual(queue.high_water, 2)
        self.assertEqual(queue.pop(), b"a")
        self.assertEqual(queue.pop(), b"b")
        self.assertIsNone(queue.pop())

    def test_service_drains_multiple_chunks_but_respects_budget(self):
        stream = FakeStream()
        queue = transport_mod.FrameQueue(4)
        queue.push(b"abcdef")
        queue.push(b"ghijkl")

        transport = transport_mod.UsbCdcTransport(
            max_write_chunk=4,
            max_chunks_per_service=3,
            stream=stream,
            use_poll=False,
        )

        # First service: sync delimiter + two chunks of first frame.
        self.assertTrue(transport.service(queue))
        self.assertEqual(stream.write_calls, 3)
        self.assertEqual(bytes(stream.data), b"\x00abcdef")
        self.assertEqual(len(queue), 1)

        # Second service can drain the second frame within the same bounded burst.
        self.assertTrue(transport.service(queue))
        self.assertEqual(bytes(stream.data), b"\x00abcdefghijkl")
        self.assertEqual(len(queue), 0)
        self.assertIsNone(transport.current)

    def test_service_stops_when_queue_empty(self):
        stream = FakeStream()
        queue = transport_mod.FrameQueue(2)
        transport = transport_mod.UsbCdcTransport(
            max_write_chunk=64,
            max_chunks_per_service=4,
            stream=stream,
            use_poll=False,
        )

        self.assertTrue(transport.service(queue))  # initial sync delimiter only
        calls_after_sync = stream.write_calls
        self.assertFalse(transport.service(queue))
        self.assertEqual(stream.write_calls, calls_after_sync)


if __name__ == "__main__":
    unittest.main()
