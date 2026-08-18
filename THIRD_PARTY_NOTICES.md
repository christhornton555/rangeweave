# Third-party notices / dependencies

The initial repository scaffold intentionally **does not bundle third-party firmware binaries**.

## Pimoroni RP2350 MicroPython build and libraries

The reference Pico 2 W runtime uses Pimoroni's RP2350/Pico 2 W MicroPython distribution, including VL53L5CX support.

- Repository: https://github.com/pimoroni/pimoroni-pico-rp2350
- Releases: https://github.com/pimoroni/pimoroni-pico-rp2350/releases
- Libraries: https://github.com/pimoroni/pimoroni-pico

Users obtain the appropriate UF2 upstream. Do not copy a local `.uf2` into this repository merely for convenience; record exact build provenance in validation reports.

## VL53L5CX firmware blob

The Pimoroni driver used during prototype bring-up expects `vl53l5cx_firmware.bin` on the Pico filesystem. The source path used during development was:

https://github.com/ST-mirror/VL53L5CX_ULD_driver/blob/no-fw/lite/en/vl53l5cx_firmware.bin

This repository does not redistribute the blob. `tools/fetch_vl53l5cx_firmware.py` is provided as fetch/checksum tooling. Before a tagged release, pin an immutable upstream source/checksum and re-review the applicable upstream licensing/notice requirements.

## Vendor documentation

The build/diagnostic code was developed with reference to STMicroelectronics datasheets/application notes and Adafruit/Pimoroni hardware documentation linked from `docs/build-guide.md`. Documentation links are references, not bundled project assets.

## Contributor rule

If third-party source, model weights, firmware, CAD, media or other assets are later vendored, update this file and preserve all required notices/licenses at the time they are added.
