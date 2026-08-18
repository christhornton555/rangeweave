# Rangeweave protocol

**Status: v0.1 EXPERIMENTAL / implementation candidate.**

The normative byte-level specification is [`spec-v0.1.md`](spec-v0.1.md).

Current protocol assets:

- [`spec-v0.1.md`](spec-v0.1.md) — framing, versioning and record layouts.
- [`test-vectors/v0.1.json`](test-vectors/v0.1.json) — canonical byte fixtures and expected semantic values.
- [`../host/python/rangeweave_protocol.py`](../host/python/rangeweave_protocol.py) — dependency-light Python reference codec/stream decoder.
- [`../tests/test_protocol.py`](../tests/test_protocol.py) — golden-vector, CRC, COBS and corruption/resynchronisation tests.

The semantic format is independent of Pico, MicroPython, USB and Python. A future Kotlin/Android implementation must consume the same fixtures.
