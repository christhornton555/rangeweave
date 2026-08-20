"""Temporary exp4 hardware-validation harness for Rangeweave Pico acquisition.

This reuses the canonical exp2 scheduler/transport and changes only the active
MicroPython protocol implementation, whose CRC hot path has been replaced with a
16-entry nibble lookup. The wire protocol and packet bytes remain unchanged.

Run manually from Thonny after interrupting the auto-started canonical main.py.
Do not rename this file to main.py: it imports main.py as the exp2 implementation base.
"""

import main as exp2


FIRMWARE_LABEL = b"rangeweave-pico2w-acq-0.1-exp4"


class Acquisition(exp2.Acquisition):
    def __init__(self):
        exp2.FIRMWARE_LABEL = FIRMWARE_LABEL
        super().__init__()


def main():
    acquisition = Acquisition()
    acquisition.run()


if __name__ == "__main__":
    main()
