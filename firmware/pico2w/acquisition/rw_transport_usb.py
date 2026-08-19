"""USB-CDC transport adapter for Rangeweave Pico acquisition firmware.

The queue and packet bytes are transport-neutral. This module is the only place that
knows the current prototype uses MicroPython's built-in USB CDC stdout stream.
"""

import sys

try:
    import select
except ImportError:
    select = None


class FrameQueue:
    def __init__(self, capacity):
        self.capacity = capacity
        self._items = [None] * capacity
        self._head = 0
        self._tail = 0
        self._count = 0
        self.high_water = 0

    def __len__(self):
        return self._count

    def full(self):
        return self._count >= self.capacity

    def push(self, frame):
        if self.full():
            return False
        self._items[self._tail] = frame
        self._tail = (self._tail + 1) % self.capacity
        self._count += 1
        if self._count > self.high_water:
            self.high_water = self._count
        return True

    def pop(self):
        if self._count == 0:
            return None
        frame = self._items[self._head]
        self._items[self._head] = None
        self._head = (self._head + 1) % self.capacity
        self._count -= 1
        return frame


class UsbCdcTransport:
    """Incremental, polling-first writer for the built-in USB CDC stream."""

    def __init__(self, max_write_chunk=64):
        try:
            self.stream = sys.stdout.buffer
        except AttributeError:
            raise RuntimeError("MicroPython sys.stdout.buffer is required")

        self.max_write_chunk = max_write_chunk
        self.current = None
        self.offset = 0
        self.write_errors = 0
        self.bytes_written = 0
        self._sync_pending = True

        self.poller = None
        if select is not None:
            try:
                poller = select.poll()
                poller.register(self.stream, select.POLLOUT)
                self.poller = poller
            except Exception:
                self.poller = None

    def has_pending(self):
        return self._sync_pending or self.current is not None

    def writable(self):
        if self.poller is None:
            return True
        try:
            events = self.poller.poll(0)
        except Exception:
            self.poller = None
            return True
        for event in events:
            if event[1] & select.POLLOUT:
                return True
        return False

    def _write_some(self, data, start):
        if not self.writable():
            return start

        remaining = len(data) - start
        if remaining <= 0:
            return start

        count = remaining
        if count > self.max_write_chunk:
            count = self.max_write_chunk

        try:
            written = self.stream.write(memoryview(data)[start:start + count])
        except (OSError, IOError):
            self.write_errors += 1
            return start

        if written is None or written <= 0:
            return start

        self.bytes_written += written
        return start + written

    def service(self, queue):
        # A leading zero delimiter discards any textual soft-reset/REPL bytes that
        # may have preceded main.py, so the first Rangeweave frame can start cleanly.
        if self._sync_pending:
            if not self.writable():
                return False
            try:
                written = self.stream.write(b"\x00")
            except (OSError, IOError):
                self.write_errors += 1
                return False
            if written:
                self.bytes_written += written
                self._sync_pending = False
            return bool(written)

        if self.current is None:
            self.current = queue.pop()
            self.offset = 0
            if self.current is None:
                return False

        new_offset = self._write_some(self.current, self.offset)
        progressed = new_offset != self.offset
        self.offset = new_offset

        if self.offset >= len(self.current):
            self.current = None
            self.offset = 0

        return progressed
