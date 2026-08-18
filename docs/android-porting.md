# Android porting notes

Android is a first-class host target even though Python/PC is the initial reference implementation.

## Rules to preserve now

- The byte protocol must be specified independently of Python classes.
- Every field needs explicit integer width, signedness, endianness, unit and coordinate-frame meaning.
- Golden packet fixtures belong in `protocol/test-vectors/` and must be consumed by Python and Kotlin tests.
- Parsing/geometry code should not live in Activity/Fragment/Compose lifecycle code.
- Prefer Kotlin coroutines/Flow (or another explicit streaming boundary) between transport, decoder and UI.
- Start with Android USB host/OTG for the wired prototype; BLE/Wi-Fi are later transport adapters.
- Do not add JNI/NDK solely for hypothetical performance. Profile first.
- Host algorithms should consume project-owned data interfaces so Open3D/SciPy-specific choices on PC do not become impossible-to-port semantics.

## Breadcrumb required for future features

When adding a host-side feature, note:

- input/output semantic types;
- platform-specific dependency used by Python;
- expected Kotlin equivalent or pure-math implementation;
- golden numeric fixture needed for parity;
- acceptable numeric tolerance;
- threading/lifecycle assumptions.

## First Android milestone

After the packet format stabilises: Android USB host receives a Pico stream and the Kotlin parser passes the same golden byte fixtures as the Python parser. Mapping parity comes later.
