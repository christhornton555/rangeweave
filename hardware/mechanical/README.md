# Mechanical / rigid-body notes

**Status: prototype guidance; detailed CAD not yet created.**

For tracking/mapping, the relative transform between the ToF sensor and inertial sensors must remain fixed. A flexible breadboard is fine for electrical bring-up but not for calibration-quality motion data.

Current rules:

- make the sensing head a rigid body;
- keep the VL53L5CX aperture unobstructed;
- label +X/+Y/+Z physically;
- record approximate sensor-origin offsets;
- keep the LIS3MDL away from magnets, motors, steel fasteners and high-current wiring where practical;
- calibrate the magnetometer after installation in the assembled head;
- avoid a cover window until its ToF crosstalk effects are deliberately characterized.
