"""Pure timing helpers shared by Rangeweave acquisition code.

Kept hardware-independent so wrap/out-of-order behaviour can be regression-tested under
CPython as well as used by MicroPython firmware.
"""


def signed_mod_delta32(new_value, reference_value):
    delta = (new_value - reference_value) & 0xFFFFFFFF
    if delta & 0x80000000:
        delta -= 0x100000000
    return delta


class LsmTickExtender:
    """Map nearby 32-bit LSM timestamp observations into one 64-bit epoch.

    A direct register read may be newer than FIFO timestamps still waiting in backlog.
    Older observations are mapped relative to the current anchor without moving it
    backwards. This assumes no two observations being related differ by >= 2^31 ticks,
    which is vastly larger than the expected acquisition backlog.
    """

    def __init__(self):
        self.anchor_raw = None
        self.anchor_extended = None

    def extend(self, raw_tick):
        raw_tick &= 0xFFFFFFFF
        if self.anchor_raw is None:
            self.anchor_raw = raw_tick
            self.anchor_extended = raw_tick
            return self.anchor_extended

        delta = signed_mod_delta32(raw_tick, self.anchor_raw)
        extended = self.anchor_extended + delta

        if delta >= 0:
            self.anchor_raw = raw_tick
            self.anchor_extended = extended

        if extended < 0:
            # Only plausible for a contrived very-early out-of-order startup read.
            # Preserve uint64 wire semantics; normal validated scheduling never hits it.
            return raw_tick

        return extended
