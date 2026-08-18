# Public baseline provenance

| Artifact | SHA-256 | Note |
|---|---|---|
| Local validated v0.5 candidate (not committed) | `ee176394004aa66795aeda0645b6032a7f7f3e485b538c0fea7349c8b864d49c` | Exact script used to create the v0.5 diagnostic baseline before canonical renaming. |
| `firmware/pico2w/diagnostics/reproducible_sensor_stack.py` | `54169d31450c64b396dca8fd6979002042f9444df17b71f8c6859e5388525e5c` | Behaviour-preserving public copy with header/status wording normalized. |
| `firmware/pico2w/diagnostics/i2c_scan.py` | `72da4da34a5856eec047fcb2da9fd32b74d77b7103cfd591d693f70c448a09c7` | Public split-bus scan built from the final validated wiring specification. |
| `firmware/pico2w/diagnostics/imu_bringup.py` | `f6067a0a9d4d801bfadfab4911355718044e99fa2040b42d5e6a204595b1b220` | Cleaned standalone bring-up aligned to final 0x1E/BDU/20-Hz-internal magnetometer configuration. |
| `firmware/pico2w/diagnostics/tof_bringup.py` | `62098639f57f304df282788a3f38a6b1727a4fe65e664e1920073f53841d5c96` | Cleaned standalone bring-up aligned to final I2C1 GP2/GP3 1-MHz topology. |

The public standalone bring-up scripts are curated builder tools, not byte-identical copies of the earliest scratch scripts. The full reproducibility diagnostic preserves the validated v0.5 logic; only public-facing header/status wording was changed.
