# Pico 2 W acquisition firmware

**Status: NEXT after protocol v0.1 review.**

The first protocol implementation candidate is now documented in [`../../../protocol/spec-v0.1.md`](../../../protocol/spec-v0.1.md), with golden byte fixtures and a Python reference decoder.

This directory will contain the transport-independent sensor acquisition firmware. It must reuse the validated sensor configuration/FIFO/timestamp behaviour without turning the diagnostic script into a production logger.

Design boundary:

```text
sensor drivers + clock service
          |
    packetizer / queue
          |
     transport adapter
          |
      USB initially
```

The packetizer may not depend on USB, and host code may not depend on Pico GPIO numbers or MicroPython object layouts.

The first acquisition implementation should:

1. extend MCU and LSM native counters to protocol uint64 time values;
2. drain complete timestamped IMU slots into `IMU_BATCH` records;
3. preserve MAG and ToF observation/read brackets;
4. emit raw `CLOCK_SYNC` observations rather than derived mapped timestamps;
5. expose `STATUS` counters/backpressure;
6. queue complete records before the USB transport adapter writes bytes;
7. never print diagnostic text into the binary protocol stream.

Before that firmware is called complete, define the deferred stream/config metadata record described in protocol v0.1.
