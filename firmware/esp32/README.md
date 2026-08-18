# ESP32-class portability work

**Status: PLANNED.**

The Pico 2 W is a prototype controller, not a protocol dependency. An early portability spike will use a small ESP32-class board (or comparable MCU) to emit protocol-conforming synthetic records before porting all sensor drivers.

Success criterion: the PC and Android parsers should consume the synthetic ESP32 stream unchanged except for metadata describing the source controller/transport.

Do not port by copying Pico scheduling assumptions. Preserve these conceptual services instead:

- sensor drivers returning raw samples/native timestamps;
- a monotonic MCU clock;
- sensor <-> MCU clock-correlation observations;
- packetizer/queue independent of transport;
- transport adapter (USB/UART/BLE/Wi-Fi/local storage as appropriate).

See [`docs/hardware-porting.md`](../../docs/hardware-porting.md) and ADR-0001.
