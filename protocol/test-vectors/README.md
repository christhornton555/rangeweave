# Protocol golden test vectors

Canonical cross-language fixtures live here.

## Current fixture

[`v0.1.json`](v0.1.json) contains one valid frame for every protocol v0.1 record type plus a deliberately corrupted-frame/resynchronisation stream case.

Each vector includes:

- complete wire bytes as hexadecimal, including the trailing `00` delimiter;
- COBS-decoded frame bytes (including CRC) as hexadecimal;
- expected record type and global packet sequence;
- expected semantic fields in language-neutral JSON.

The Python conformance tests consume this file now. The Kotlin/Android decoder must consume the **same file**, not a translated copy.

Do not regenerate fixtures casually after a protocol change: changing a fixture that represents an existing protocol version is a compatibility break unless the old fixture was demonstrably wrong and the correction is documented.
