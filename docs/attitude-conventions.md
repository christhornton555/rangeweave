# Attitude and local-reference conventions

**Status:** frozen for the Phase 3 reference orientation path. These conventions define the project-owned meaning of persistent body attitude; platform, graphics and sensor SDK conventions must be converted at their boundaries rather than leaking inward.

## Frames

Phase 3 introduces a gravity-referenced local orientation frame named `local_reference`.

`device_body` remains the rigid sensing-head frame:

- `+X`: device right;
- `+Y`: device down;
- `+Z`: device forward.

`local_reference` is right-handed and uses the same axis sense:

- `+Y`: gravity down at initialization;
- `+Z`: the horizontal projection of the initial `device_body +Z` direction;
- `+X`: completes the right-handed frame (`+X x +Y = +Z`).

If initial `device_body +Z` is too close to vertical for a stable projection, the implementation falls back to the horizontal projection of initial `device_body +X` and reconstructs `+Z` consistently.

This makes initial yaw repeatable relative to the sensing head, but **not globally referenced**. Accelerometer + gyro cannot determine absolute yaw around gravity. `local_reference` therefore must not be described as north-aligned, magnetic, geographic or globally fixed.

Phase 3 is orientation-only: the origin of `local_reference` is intentionally unspecified.

## Accelerometer meaning

At rest, an accelerometer reports **specific force**, which points opposite physical gravity. In this project:

```text
up_body = normalise(accel_body)
gravity_down_body = -up_body
```

The local reference vectors are therefore:

```text
up_local           = (0, -1, 0)
gravity_down_local = (0, +1, 0)
```

Existing short-baseline code may use the informal phrase "gravity vector" for the stationary accelerometer direction when checking closure. Persistent-attitude code must distinguish measured specific-force/up direction from physical gravity-down direction.

## Rotation contract

Persistent attitude is the active rotation:

```text
R_reference_from_body
```

which maps a vector expressed in the current `device_body` frame into `local_reference`:

```text
v_reference = R_reference_from_body * v_body
```

This deliberately matches the transform direction already used by the short-baseline relative-rotation and boresight paths.

The inverse is the transpose:

```text
v_body = transpose(R_reference_from_body) * v_reference
```

## Quaternion representation

The project-owned quaternion is:

```text
q_reference_from_body = (w, x, y, z)
```

with:

- scalar-first component order `(w, x, y, z)`;
- Hamilton product;
- unit norm;
- active vector rotation;
- right-handed positive rotations;
- column-vector matrix equivalent `R_reference_from_body`.

Quaternion sign has no physical meaning: `q` and `-q` represent the same rotation. Comparisons must account for that equivalence.

## Composition order

For transforms:

```text
q_A_from_B
q_B_from_C
```

the composed transform is:

```text
q_A_from_C = q_A_from_B ⊗ q_B_from_C
```

where `⊗` is the Hamilton product.

The corresponding matrices obey:

```text
R_A_from_C = R_A_from_B * R_B_from_C
```

This is the same composition order used by the existing Rangeweave matrix helpers.

## Gyroscope convention and integration

After the physically validated `imu_sensor -> device_body` mapping, gyroscope samples are angular velocity expressed in the **current `device_body` frame**, in deg/s at the public helper boundary and rad/s internally.

For a body-frame incremental rotation `dq_body` over one timestep:

```text
q_reference_from_body(t + dt)
    = q_reference_from_body(t) ⊗ dq_body
```

That is a **right multiplication** because the measured angular velocity is body-frame angular velocity. It matches the existing short-baseline matrix integration:

```text
R <- R * exp(skew(omega_body * dt))
```

Timestamp intervals come from the recorded LSM hardware timestamp correlation. No orientation path may substitute `sample_index / nominal_ODR`.

## Gravity correction and observability

The first persistent estimator is six-axis in the IMU sense: gyroscope + accelerometer only.

Accelerometer correction may constrain the direction of `up_local` / gravity and therefore control pitch/roll drift when specific force is credible. It must be confidence-gated because linear acceleration is indistinguishable from gravity in a single accelerometer sample.

Rotation about the gravity axis remains unobservable without an additional trusted heading source. Therefore:

- yaw is initialized by the local-reference construction above;
- gyro integration propagates subsequent yaw;
- accelerometer correction must not be presented as absolute yaw correction;
- yaw drift is an expected six-axis limitation, not a hidden estimator failure.

Magnetometer heading remains a later extension after `mag_sensor -> device_body`, hard/soft-iron calibration and disturbance gating are physically validated.

## Golden examples

These examples are normative regression cases.

### Identity

```text
q = (1, 0, 0, 0)
R = identity
```

### +90 degrees about body/reference X

```text
q = (sqrt(0.5), sqrt(0.5), 0, 0)
```

and:

```text
R * (0, 1, 0) = (0, 0, 1)
R * (0, 0, 1) = (0, -1, 0)
```

### Composition

If `q1` maps B -> A and `q2` maps C -> B, then `q1 ⊗ q2` must produce the same vector result as `R(q1) * R(q2)`.

### Body-rate propagation

Starting at identity, integrating a constant `+90 deg/s` body-X angular rate for exactly one second must produce a `+90 deg` X rotation in `R_reference_from_body`, subject only to numerical integration tolerance.

## Boundary rule

Any Android sensor quaternion, graphics quaternion, robotics transform, game-engine rotation or external library object must be converted explicitly into these conventions. Do not rename an external representation and assume it is compatible.
