# Python reference host

**Status: PLANNED / PC-first reference implementation.**

Initial modules should be dependency-light around the protocol/capture boundary, then use scientific/3D libraries behind project-owned interfaces as needed.

Planned order:

1. packet decoder;
2. USB receiver + lossless recorder;
3. replay adapter using the same decoded stream API;
4. raw health/depth viewer;
5. calibrated 64-point projection;
6. orientation;
7. odometry/mapping baselines.

The protocol and calibration semantics must remain reproducible in Kotlin/Android without importing Python-specific concepts.
