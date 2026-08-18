#!/usr/bin/env python3
"""Fetch the VL53L5CX firmware blob used by the Pimoroni MicroPython driver.

The project intentionally does not vendor this third-party binary. By default this
script fetches the same upstream branch/path used during prototype bring-up and
prints its SHA-256. For a tagged project release, pass a project-pinned expected
SHA-256 with --sha256 (and ideally update the URL to an immutable upstream commit).
"""

from __future__ import annotations

import argparse
import hashlib
from pathlib import Path
import sys
import urllib.request

DEFAULT_URL = (
    "https://raw.githubusercontent.com/ST-mirror/VL53L5CX_ULD_driver/"
    "no-fw/lite/en/vl53l5cx_firmware.bin"
)


def digest(data: bytes) -> str:
    return hashlib.sha256(data).hexdigest()


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--url", default=DEFAULT_URL, help="Upstream firmware URL")
    parser.add_argument("--output", default="vl53l5cx_firmware.bin")
    parser.add_argument(
        "--sha256",
        help="Expected SHA-256. If supplied, mismatch aborts without writing the file.",
    )
    args = parser.parse_args()

    print("Fetching:", args.url)
    with urllib.request.urlopen(args.url, timeout=30) as response:
        data = response.read()

    actual = digest(data)
    print("Bytes:", len(data))
    print("SHA-256:", actual)

    if args.sha256 and actual.lower() != args.sha256.lower():
        print("ERROR: SHA-256 mismatch; file not written.", file=sys.stderr)
        return 2

    output = Path(args.output)
    output.write_bytes(data)
    print("Wrote:", output)

    if not args.sha256:
        print(
            "WARNING: this fetch was not pinned by hash. Record/pin the checksum "
            "before relying on it for a tagged reproducibility release."
        )

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
