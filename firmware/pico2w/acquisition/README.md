# Pico 2 W acquisition firmware

**Status: PLANNED / next development stage.**

This directory will contain the transport-independent sensor acquisition firmware once the packet specification is frozen enough to implement. It must reuse the validated sensor configuration/timestamp behaviour without turning the diagnostic script into a production logger.

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
