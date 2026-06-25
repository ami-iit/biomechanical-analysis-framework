# HumanIK Frame Mathematics

This page documents the mathematical conventions used by HumanIK for sensor alignment, calibration, and task setpoint computation.

For a practical usage guide covering the calibration workflow and runtime loop, see [ik-calibration-and-runtime.md](ik-calibration-and-runtime.md).

## Frame Notation

| Symbol | Meaning |
|--------|---------|
| $^W$ | World / inertial frame (origin at IK world) |
| $^{WA}$ | World-anchored frame (identity rotation, translation-only offset) |
| $^S$ | Sensor frame (e.g. IMU-aligned world or external tracker frame) |
| $^M$ | Measured frame (raw output of the sensor) |
| $^L$ | Link / task frame (control point on the model) |
| $\hat{\ }$ | Hat notation marks state-dependent matrices updated by calibration |

Each $H$ matrix is a $4 \times 4$ homogeneous transformation:

$$^A H_B = \begin{bmatrix} ^A R_B & ^A \mathbf{p}_B \\ \mathbf{0} & 1 \end{bmatrix}$$

where $^A R_B \in SO(3)$ is the rotation block and $^A \mathbf{p}_B \in \mathbb{R}^3$ is the translation.

## General Setpoint Composition

All task setpoints follow the same general pattern:

$$^{WA}\hat{H}_L = {^{WA}H_W} \cdot {^W\hat{H}_S} \cdot {^S H_M} \cdot {^M\hat{H}_L}$$

where:

- $^{WA}H_W$ — **world anchor**: identity rotation, translation-only; updated during base resets to keep sensor targets world-centered.
- $^W\hat{H}_S$ — **left calibration matrix**: world-to-sensor alignment; calibrated and updated by `calibrateWorldYaw()` and `calibrateAllWithWorld()`.
- $^S H_M$ — **measurement**: sensor-to-measured-frame transform; comes directly from the sensor stream each cycle.
- $^M\hat{H}_L$ — **right calibration matrix**: measured-frame-to-link extrinsic; loaded from configuration and refined during full calibration.

> **Note on pose and position tasks:** for these tasks only the left matrix $^W\hat{H}_S$ is refined during calibration. The right matrix $^M H_L$ remains a fixed extrinsic from configuration. The world anchor $^{WA}H_W$ carries only a translation offset.

## Task-Specific Formulas

### Orientation Task

**Setpoint (rotation only):**

$$^{WA}\hat{R}_L = \mathrm{rotation}\!\left({^W\hat{H}_S} \cdot {^S H_M} \cdot {^M\hat{H}_L}\right)$$

where $\mathrm{rotation}(\cdot)$ extracts the $3 \times 3$ upper-left block.

**Angular velocity setpoint:**

$$^W\hat{\omega}_L = {^W\hat{R}_S} \cdot {^S \omega_M}$$

---

### Gravity Task

**Setpoint (gravity direction vector):**

$$^{WA}\mathbf{g} = \left(^{WA}\hat{R}_L\right)^T \cdot \mathbf{k}$$

where $\mathbf{k} = [0,\, 0,\, 1]^T$ is the world z-axis. This extracts the third column of the calibrated orientation.

---

### Pose Task

**Setpoint (full SE3):**

$$^{WA}\hat{H}_L = {^{WA}H_W} \cdot {^W\hat{H}_S} \cdot {^S H_M} \cdot {^M H_L}$$

Decomposed into rotation $^{WA}\hat{R}_L$ (upper-left $3 \times 3$) and position $^{WA}\hat{\mathbf{p}}_L$ (upper-right $3 \times 1$).

**Velocity setpoints (mixed representation):**

$$^W\hat{\omega}_L = {^W\hat{R}_S} \cdot {^S\omega_M}$$

$$^W\hat{\mathbf{v}}_L = {^W\hat{R}_S} \cdot \left({^S\mathbf{v}_M} + {^S\omega_M} \times {^M\mathbf{p}_L}\right)$$

---

### Position Task

**Raw sensor measurement (with extrinsic):**

$$^S\mathbf{p}_L = {^S\mathbf{p}_M} + {^S R_M} \cdot {^M\mathbf{p}_L}$$

**World-anchored target position:**

$$^{WA}\mathbf{p}_L = \mathrm{position}\!\left({^{WA}H_W} \cdot {^W\hat{H}_S} \cdot {^S H_M} \cdot {^M H_L}\right)$$

**Linear velocity setpoint:**

$$^W\mathbf{v}_L = {^W\hat{R}_S} \cdot {^S\mathbf{v}_M}$$

The world anchor does not contribute to velocity because $^{WA}H_W$ has identity rotation; world and world-anchored velocities coincide.

## Calibration Procedures

### `calibrateWorldYaw()`

**Purpose:** yaw-only alignment of the world-to-sensor frame, assuming the sensor vertical axis is already gravity-consistent.

**Computation (per task node):**

$$\text{raw offset} = {^W R_L} \cdot \mathrm{rotation}\!\left({^S H_M} \cdot {^M H_L}\right)^T$$

Extract and retain only the yaw component:

$$^W\hat{R}_{S,\,\mathrm{yaw}} = SO3\!\left(0,\, 0,\, \mathrm{yaw}(\text{raw offset})\right)$$

**State updated:**

- Orientation / gravity tasks: $^W\hat{H}_S \leftarrow$ yaw-only offset.
- Pose / position tasks: $^W\hat{H}_S \leftarrow$ yaw-only offset; world anchor translation $^{WA}\mathbf{p}_W$ updated from current base position.

---

### `calibrateAllWithWorld()`

**Purpose:** full 3D calibration with optional reference frame alignment, building on the yaw stage.

**Step 1 — Refine measured-frame-to-link extrinsic:**

$$^M H_L = \left({^W\hat{H}_{S,\,\mathrm{yaw}}} \cdot {^S H_M}\right)^{-1} \cdot {^W R_L}$$

**Step 2 — Apply secondary world calibration:**

$$^W\hat{H}_{S,\,\mathrm{final}} = {^{\mathrm{ref}}H_W} \cdot {^W\hat{H}_{S,\,\mathrm{yaw}}}$$

where $^{\mathrm{ref}}H_W = \left({^W H_{\mathrm{ref}}}\right)^{-1}$ and `ref` is the `frameRef` argument supplied by the caller.

**State updated:**

- Orientation / gravity tasks: $^M\hat{H}_L \leftarrow$ calibrated right matrix.
- Pose / position tasks: $^W\hat{H}_S \leftarrow$ final calibrated left matrix; world anchor translation updated.

---

### World Anchor

The world anchor $^{WA}H_W$ has identity rotation and a translation-only update rule:

$$^{WA}H_W = \begin{bmatrix} I & ^{WA}\mathbf{p}_W \\ \mathbf{0} & 1 \end{bmatrix}$$

**Accumulation during calibration or base reset:**

$$^{WA}\mathbf{p}_W \mathrel{-}= {^W\mathbf{p}_{\mathrm{base}}}$$

When the internal base position is reset to the origin, the negated offset ensures absolute sensor measurements remain correctly centered in the world frame.

**Full reset:**

$$^{WA}H_W = I$$

## Notes

1. **$H$ matrices** provide a unified SE3 representation: rotations and translations compose naturally via matrix multiplication.
2. **Component extraction**: orientation/gravity tasks use only the $R$ block; position tasks use only the $\mathbf{p}$ block with appropriate rotation pre-compensation; pose tasks use all six DOF.
3. **Hat notation** marks calibration matrices that change at calibration time. The world anchor $^{WA}H_W$ is not hat-marked because its rotation is always identity; only its translation is mutable.
