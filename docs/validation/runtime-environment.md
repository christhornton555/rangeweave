# Reference Pico runtime environment

The validated reference unit reported:

```text
MicroPython pico2_w_2025_09_19, on 2025-09-22
MicroPython 1.27.0 preview
board/build: rpi_pico2_w
```

The local pre-Git development folder also contained a Pico 2 W UF2 named:

```text
rpi_pico2_w-v1.26.1-micropython.uf2
```

Pimoroni's official RP2350 release page labels `v1.26.1` as the Pico 2/Pico 2 W distribution release and its changelog identifies the MicroPython build lineage ending at `pico2_w_2025_09_19`.

Upstream release page:

<https://github.com/pimoroni/pimoroni-pico-rp2350/releases/tag/v1.26.1>

For reproducibility, treat the **runtime fingerprint printed by the board** as the authoritative record, not a filename sitting on a development PC. A later compatible Pimoroni build may work but should be reported explicitly rather than described only as “latest”.

The UF2 is not redistributed in this repository.
