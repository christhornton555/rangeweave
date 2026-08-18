# Cross-project tests

Cross-cutting compatibility/regression tests live here when they span packages or platforms.

Implemented now:

- [`test_protocol.py`](test_protocol.py) — protocol v0.1 golden vectors, CRC check value, COBS round-trip and corruption/resynchronisation behaviour.

Future cross-project gates include:

- Python <-> Kotlin conformance outputs using the same protocol fixtures;
- replay regression tests;
- coordinate-transform golden examples;
- reference-dataset metric regression tests.

Run current tests from the repository root with:

```bash
python -m unittest discover -s tests -v
```
