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

## Protocol v0.1 Android breadcrumb

The first Kotlin decoder should be a small platform-independent module that can be unit-tested without an Android device. Android USB should feed bytes into it through an adapter rather than owning packet semantics.

Implementation notes:

- use explicit little-endian reads for every multibyte field; do not rely on a platform default;
- treat protocol `uint32` values as a wider Kotlin numeric type when arithmetic/comparison requires unsigned range, and treat `uint64` timestamps/session IDs without narrowing them to signed 32-bit values;
- COBS and CRC-16/CCITT-FALSE must reproduce the exact bytes/check values in `protocol/test-vectors/v0.1.json`;
- unknown `STREAM_INFO` TLVs should be preserved or ignored, not treated as a decode failure;
- `STREAM_INFO.session_id` is an opaque ephemeral session value, not a user/device identity;
- retain source clock domains exactly: Android should fit common time from `CLOCK_SYNC`, not reinterpret IMU ticks as microseconds or infer timing from packet arrival;
- ToF `layout_id = 0` remains producer-native flattened order until the Rangeweave coordinate-frame/zone-order convention is frozen; UI/rendering code must not silently reinterpret it.

The same `v0.1.json` fixture used by the Python tests is the acceptance fixture for Kotlin. Do not create an Android-specific copy.

## Breadcrumb required for future features

When adding a host-side feature, note:

- input/output semantic types;
- platform-specific dependency used by Python;
- expected Kotlin equivalent or pure-math implementation;
- golden numeric fixture needed for parity;
- acceptable numeric tolerance;
- threading/lifecycle assumptions.

## First Android milestone

After protocol v0.1 survives live Pico acquisition testing: Android USB host receives a Pico stream and the Kotlin parser passes the same golden byte fixtures as the Python parser. Mapping parity comes later.
