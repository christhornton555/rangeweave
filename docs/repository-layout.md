# Repository layout

```text
sparse-tof-spatial-mapping/
  README.md
  LICENSE                       # scaffold: choose before public release
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
    licensing.md
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

## Deliberately excluded from normal source control

- superseded dated scratch scripts (`test04`-`test08` etc.);
- old DOCX documentation revisions;
- local copies of MicroPython `.uf2` images;
- the VL53L5CX firmware blob until redistribution/pinning is deliberately handled;
- large/raw personal capture directories;
- local editor/virtual-environment files.

Keep the old local development folder offline as a pre-Git archive if desired. Future history belongs in Git.
