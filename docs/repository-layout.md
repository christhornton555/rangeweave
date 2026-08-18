# Repository layout

```text
rangeweave/
  README.md
  LICENSE
  THIRD_PARTY_NOTICES.md
  CONTRIBUTING.md
  CHANGELOG.md
  .gitignore
  .gitattributes
  .editorconfig

  docs/
    README.md
    project-plan.md
    build-guide.md
    architecture.md
    protocol.md
    calibration.md
    coordinate-frames.md
    android-porting.md
    hardware-porting.md
    development-history.md
    repository-layout.md
    validation/
    adr/

  firmware/
    pico2w/
      diagnostics/
        i2c_scan.py
        imu_bringup.py
        tof_bringup.py
        reproducible_sensor_stack.py
      acquisition/
    esp32/

  protocol/
    README.md
    test-vectors/

  host/
    python/

  android/

  hardware/
    BOM.md
    wiring/
    mechanical/
    pcb/

  tools/
  tests/
  sim/
  datasets/
  .github/
    ISSUE_TEMPLATE/
```
