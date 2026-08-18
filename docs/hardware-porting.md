# Hardware / MCU porting notes

The Raspberry Pi Pico 2 W is the current reference controller, not an architectural requirement.

## Firmware services that must survive an MCU port

1. **Sensor drivers** - configure devices and return raw measurements plus native timestamps.
2. **Clock service** - MCU monotonic time and explicit sensor<->MCU clock correlation.
3. **Packetizer/queue** - versioned records unaware of transport.
4. **Transport adapter** - USB initially; BLE/Wi-Fi/UART/local storage later.

## Things that may not leak upward

- Pico GPIO numbers;
- MicroPython object types;
- exact scheduler loop timing;
- the reference IMU's measured ~101-Hz real rate;
- USB-specific framing assumptions.

## Early ESP32-class portability spike

Do this before the mapping stack becomes large:

1. implement only MCU monotonic time + packetizer + a synthetic IMU/TOF generator on a small ESP32-class board;
2. emit the same protocol records over an available transport;
3. require the existing PC parser, recorder and Android parser to consume the stream unchanged except for source metadata;
4. only then port physical sensor drivers if the target MCU still looks suitable.

This tests the architecture boundary cheaply before a hardware redesign.
