# Rangeweave capture format

**Status: implementation candidate for Phase 1 capture/replay.**

This document defines the first canonical host-side recording layout for Rangeweave. The capture format is deliberately separate from the wire-protocol version: a future capture-format revision may add metadata or sidecar files without changing Rangeweave packet semantics.

## Goals

A Rangeweave recording should be:

- lossless with respect to the Rangeweave wire bytes that were selected for the session;
- replayable through the exact same `StreamDecoder` used for live input;
- self-describing enough to identify protocol/source configuration and stream health;
- integrity-checkable without proprietary tooling;
- independent of USB, Windows, Python object serialization, or a particular visualization/mapping stack.

## Session directory

The initial layout is:

```text
capture_YYYYMMDD_HHMMSSZ[_label]/
  metadata.json
  packets.bin
  notes.txt
```

Local `captures/` directories are ignored by Git. Small reviewed regression fixtures may later be added under `datasets/` using an explicit repository policy.

`calibration.json` is intentionally **not required yet**. Calibration gets its own versioned schema when the geometry/calibration phase begins rather than inventing an unstable placeholder in the capture format.

## `packets.bin`

`packets.bin` is the authority for recorded sensor data. It contains the same COBS-framed, CRC-protected, `0x00`-delimited Rangeweave frames used live. No decoded Python objects, converted units, mapped timestamps, compressed arrays, or visualization-specific structures replace those bytes.

For a normally completed serial capture:

1. startup/backlog bytes are discarded during a configurable warm-up;
2. the recorder discards through one observed `0x00` delimiter before beginning the file, so the first saved byte is at a known frame boundary;
3. once the requested duration expires, recording continues only far enough to observe a closing delimiter;
4. the completed file therefore consists of complete wire frames and may be replayed with arbitrary file-read chunk sizes.

The actual recorded duration can be slightly longer than the requested duration because of this final frame-boundary completion.

Interrupted or errored sessions are preserved rather than silently deleted. Their `metadata.json` status records that the file may not have a normal clean boundary.

## `metadata.json`

Capture metadata uses:

```json
{
  "format": "rangeweave-capture",
  "format_version": 1
}
```

The current implementation records:

- capture status (`recording`, `complete`, `interrupted`, or `error`);
- UTC start/end timestamps plus requested and measured duration;
- source kind and serial API parameters;
- `packets.bin` byte count and SHA-256;
- protocol versions observed in valid frames;
- first/last sequence numbers and total detectable sequence gaps;
- per-record counts;
- decoder good/bad/empty-delimiter counts and semantic decode errors;
- the latest `STREAM_INFO`, including a decoded view of known fields **and all raw TLVs as hex**;
- first/last `STATUS` records and wrap-aware deltas for the acquisition/drop/error counters used by the Phase 1 health gate.

The metadata deliberately does not include a computer username, host serial number, Pico unique ID, MAC address, or another newly introduced globally stable identity.

`STREAM_INFO.session_id` remains the ephemeral session identifier defined by protocol v0.1.

## `notes.txt`

`notes.txt` is UTF-8 free text supplied by the operator. It is intentionally not parsed by the capture/replay core. Typical notes might describe a scene, measured wall distance, motion pattern, fixture/jig state, or experimental purpose.

## Replay and parity

Replay reads `packets.bin` through a file byte source and feeds those bytes into the same `rangeweave_protocol.StreamDecoder` used by live capture.

For a session directory, replay checks that:

- file size matches metadata;
- SHA-256 matches metadata;
- decoder good/bad/empty-delimiter and semantic-error counts match metadata;
- first/last sequence, sequence-gap total, and record counts match metadata.

This establishes two separate properties:

1. **byte integrity** — the recorded file has not silently changed;
2. **decode parity** — replaying those bytes through the reference decoder reproduces the capture-time stream summary.

A later viewer, geometry layer, orientation estimator, or mapping algorithm should consume decoded records downstream of this same boundary instead of creating a special recording parser.

## Cross-platform rule

Python is the first implementation, not the definition of the recording semantics. An Android/Kotlin or future embedded/local-storage implementation should be able to produce or consume the same `packets.bin` bytes and equivalent JSON metadata without understanding Python classes.

## Current commands

With Thonny or other serial clients closed:

```powershell
py host/python/capture.py COM5 --seconds 30 --name stationary --notes "Sensor stationary on desk"
```

This creates a new session under `captures/` by default.

Replay it with:

```powershell
py host/python/replay.py captures/capture_YYYYMMDD_HHMMSSZ_stationary
```

Use `--strict` when a non-zero exit code is desired for recorded stream corruption, sequence gaps, or acquisition-health counter increases in addition to metadata-integrity failures.
