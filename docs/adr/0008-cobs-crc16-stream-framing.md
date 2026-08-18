# ADR: COBS + CRC16 stream framing

- **Status:** Accepted for protocol v0.1 implementation candidate
- **Date:** 2026-08-18

## Context

Rangeweave must carry binary sensor records over USB now, while remaining usable over Android USB, BLE/Wi-Fi, UART or local storage later. A consumer may attach mid-stream, and one damaged packet must not destroy framing for all following data.

The framing layer also needs to be straightforward to implement in MicroPython and Kotlin without a serialization dependency.

## Decision

Protocol v0.1 uses:

- COBS encoding for each complete decoded frame;
- `0x00` as the frame delimiter;
- a fixed 12-byte little-endian Rangeweave header;
- CRC-16/CCITT-FALSE over the decoded header + payload;
- a maximum decoded frame size of 1024 bytes;
- a global uint32 packet sequence counter.

A bad delimited frame is discarded as a unit. Parsing resumes at the next zero delimiter.

## Consequences

- A host can attach to the middle of a byte stream and recover at a delimiter.
- Arbitrary binary payloads do not require transport-specific escaping.
- CRC detects damaged/misframed records independently of lower transport layers.
- Framing code can remain the same for USB, BLE/Wi-Fi stream adapters, UART and recorded byte streams.
- The protocol pays small framing/checksum overhead in exchange for explicit recovery and cross-platform simplicity.

Any future framing replacement requires a new ADR and a migration/versioning plan.
