# Protocol golden test vectors

Add small byte-level fixtures here as soon as the first packet layout exists. Each fixture should include:

- raw packet bytes;
- expected decoded semantic fields in a language-neutral JSON/YAML representation;
- corruption/resynchronization cases where relevant;
- protocol version.

Python and Kotlin tests must consume the same fixtures.
