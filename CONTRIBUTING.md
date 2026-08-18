# Contributing

Thanks for helping. This project is early enough that reproducibility evidence is more valuable than large speculative refactors.

## Before changing higher-level mapping code

A new reference-style hardware build should first pass the diagnostics in `firmware/pico2w/diagnostics/`.

When filing a hardware issue, include where possible:

- exact board/sensor variants;
- wiring photo;
- complete `i2c_scan.py` output;
- sensor WHO_AM_I output;
- MicroPython `os.uname()` and `sys.implementation` output;
- complete `REPRODUCIBILITY SELF TEST` block;
- whether anything differs from the reference wiring;
- whether the sensor head is breadboard-flexible or rigidly mounted.

Do not assume a different measured IMU ODR is a failure by itself.

## Pull requests

- Keep commits focused.
- Add/update an ADR if changing a frozen architecture decision.
- Update docs and validation evidence when changing register settings, bus topology, coordinate conventions, packet format or calibration schema.
- Do not vendor third-party binaries without an explicit licensing/provenance review and updated `THIRD_PARTY_NOTICES.md`.
- For protocol changes, add/update shared golden fixtures before adding platform-specific decoder behaviour.
- Keep **VALIDATED / EXPERIMENTAL / PLANNED** labels honest.

## Data

Do not upload recordings containing private spaces/people simply because the sensor is RGB-free. Publish only datasets you have deliberately reviewed for release.
