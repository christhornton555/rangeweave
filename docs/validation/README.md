# Validation evidence

This directory contains intentionally published reference runs and summaries used to distinguish **measured behaviour** from roadmap claims.

## Current evidence

- [`reference-unit-v0.5.md`](reference-unit-v0.5.md) - summary of the current reference unit.
- [`pico2w-reference-run-v0.5.txt`](pico2w-reference-run-v0.5.txt) - full console output supplied by the validated run.

## Reproduction reports

When testing another physical unit, record at minimum:

- unit/build ID and date;
- exact MicroPython `os.uname()` / `sys.implementation` output;
- project commit/tag;
- bus scan and WHO_AM_I values;
- complete `REPRODUCIBILITY SELF TEST` block;
- observed `FREQ_FINE`, measured clock tick/ODR and fit RMS;
- maximum FIFO backlog;
- magnetometer successes/attempts/retries/drops;
- ToF fps/errors/max read time;
- final `SYSTEM READY` result;
- relevant wiring/mechanical changes.

Do not reject a unit because its oscillator-derived ODR differs from the reference number if the structural/timing invariants pass.

- [`provenance.md`](provenance.md) - hashes linking the public diagnostic baseline to the exact validated local v0.5 candidate.
- [`runtime-environment.md`](runtime-environment.md) - validated Pico runtime fingerprint and the pre-Git UF2 filename/provenance note.
