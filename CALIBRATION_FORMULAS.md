# HumanIK Calibration and Alignment Formulas

## Frame Notation

- $^W$ = World/Inertial frame (origin at IK world)
- $^{WA}$ = World-anchored frame (identity rotation, translation-only anchor frame)
- $^S$ = Sensor frame (e.g., IMU-aligned world or externally tracked frame)
- $^M$ = Measured frame (output of sensor measurement)
- $^L$ = Link/task frame (control point on robot)
- Hats ($\hat{\ }$) denote matrices updated via calibration (state-dependent)

## Calibration and Alignment

Setpoints computations follow the general pattern:

$$^{WA}\hat{H}_L =^{WA}H_W \cdot {^W\hat{H}_S} \cdot {^S H_M} \cdot {^M\hat{H}_L}$$

where:
- $^{WA}H_W$ = **World anchor** (identity rotation, translation-only, updated during base resets). Its aim is to have recenter the IK body as if the human was "in the center".
- $^W\hat{H}_S$ = **Left calibration matrix** (world-to-sensor, calibrated/dynamic)
- $^S H_M$ = **Measurement** (sensor-to-measured-frame, from sensor output)
- $^M\hat{H}_L$ = **Right calibration matrix** (measured-frame-to-link, state-dependent)

Each $H$ matrix is a homogeneous transformation (4×4):

$$^A H_B = \begin{bmatrix} ^A R_B & ^A \mathbf{p}_B \\\ \mathbf{0} & 1 \end{bmatrix}$$

where $^A R_B$ is the rotation and $^A \mathbf{p}_B$ is the translation.

**Note on Pose/Position Tasks:** For pose and position tasks, only the left matrix $^W\hat{H}_S$ is refined, $^{WA}H_W$ carries the anchor translation, and $^M H_L$ remains a fixed extrinsic.

---

## Task-Specific Instantiations

### Orientation Task (IMU or External Sensor)

**Setpoint (rotation component only):**

$$^{WA}\hat{R}_L = \text{rotation}(^W\hat{H}_S \cdot ^S H_M \cdot {^M\hat{H}_L})$$

where `rotation()` extracts the $3 \times 3$ rotation block from the homogeneous transformation.

**Angular Velocity (same rotation applied to sensor measurement):**

$$^W\hat{\omega}_L = {^W\hat{R}_S} \cdot {^S \omega_M}$$

---

### Gravity Task

**Setpoint (gravity vector):**

$$^{WA}\mathbf{g} = (^{WA}\hat{R}_L)^T \cdot \mathbf{k}$$

where $\mathbf{k} = [0, 0, 1]^T$ is the z-axis direction, extracted from the third column of the calibrated orientation.

---

### Pose Task (Full SE3)

**Setpoint (complete homogeneous transformation):**

$$^{WA}\hat{H}_L = {^{WA}H_W} \cdot {^W\hat{H}_S} \cdot {^S H_M} \cdot {^M H_L}$$

Decomposed as:
- Rotation: $^{WA}\hat{R}_L$ (first 3×3 block)
- Position: $^{WA}\hat{\mathbf{p}}_L$ (first 3×1 block)

**Velocity Setpoint (twist expressed in the aligned frame):**

$$^{W}\hat{\omega}_L = {^{W}\hat{R}_S} \cdot {^S\omega_M}$$


$$^{W}\hat{\mathbf{v}}_L = {^{W}\hat{R}_S} \cdot \left({^S\mathbf{v}_M} + {^S\omega_M} \times {^M\mathbf{p}_L}\right)$$

---

### Position Task (Translation Component with World Anchor)

**Raw sensor measurement (with extrinsic):**

$$^S\mathbf{p}_L = ^S\mathbf{p}_M + ^S R_M \cdot {^M\mathbf{p}_L}$$

**World-frame target position (with anchor offset):**

$$^{WA}\mathbf{p}_L = \text{position}({^{WA}H_W} \cdot {^W\hat{H}_S} \cdot {^S H_M} \cdot {^M H_L})$$

where $^{WA}H_W$ has identity rotation and only its translation is updated during calibration.

**Velocity Setpoint (linear velocity expressed in the aligned frame):**

$$^{W}\mathbf{v}_L = {^{W}\hat{R}_S} \cdot {^S\mathbf{v}_M}$$

The world anchor does not contribute to velocity; world velocities coincide with world-anchored velocities because the anchor is translation-only.

---

## Calibration and Alignment Procedures

### Recenter World Alignment

The world-anchored transform is used to keep sensor-based setpoints centered after base resets:

$$^{WA}H_W = \begin{bmatrix} I & ^{WA}\mathbf{p}_W \\\ \mathbf{0} & 1 \end{bmatrix}$$

where the rotation is always identity and only the translation $^{WA}\mathbf{p}_W$ is updated.

**Accumulation (during base reset or calibration):**

$$^{WA}\mathbf{p}_W \mathrel{-}= {^W\mathbf{p}_{base}}$$

When the internal base position is reset to the origin, this negated offset ensures that absolute sensor measurements remain world-frame-centered.

### Reset World Alignment

$$^{WA}H_W = I$$

---

### calibrateWorldYaw()

**Purpose:** Yaw-only alignment of world-to-sensor frame (assumes gravity is vertical)

**Computation (per link/node):**

$$^{WA}\hat{H}_{S,yaw} = ^W R_L \cdot \text{rotation}(^S H_M \cdot {^M H_L})^T$$

Extract and apply yaw-only component:

$$^{WA}\hat{R}_{S,yaw} = \text{SO3}(0, 0, \text{yaw}(\cdots))$$

**Updates:**
- Orientation/Gravity tasks: $^W\hat{H}_S$ ← yaw-only offset
- Pose/Position tasks: $^{WA}H_W$ ← updated anchor translation, $^W\hat{H}_S$ ← yaw-only offset

---

### calibrateAllWithWorld()

**Purpose:** Full 3D calibration with optional reference frame alignment

**Step 1 – Update measured-frame-to-link extrinsic:**

$${^M H_L} = (^W\hat{H}_{S,yaw} \cdot ^S H_M)^{-1} \cdot (^W R_L)$$

**Step 2 – Secondary world calibration:**

$$^{WA}\hat{H}_{S,final} = {^{ref}H_W} \cdot {^{WA}\hat{H}_{S,yaw}}$$

where $^{ref}H_W = (^W H_{ref})^{-1}$

**Updates:**
- Orientation/Gravity tasks: $^M\hat{H}_L$ ← calibrated right matrix
- Pose/Position tasks: $^{WA}H_W$ ← updated anchor translation, $^W\hat{H}_S$ ← calibrated left matrix

---

## Notes

1. **H matrices** provide a unified representation: rotations and translations are composed naturally via matrix multiplication.

2. **Component extraction**: For orientation/gravity, only the rotation $R$ is used; for position, only translation $\mathbf{p}$ with appropriate rotation pre-compensation; for pose, all six DOFs.

3. **Hats ($\hat{\ }$)** denote state-dependent calibration matrices; unmarked quantities are either measurements (sensor output) or fixed extrinsics. The world-anchored transform $^{WA}H_W$ is not hat-marked because its rotation is fixed to identity; only its translation changes.

4. **Anchor translation** is stored in $^{WA}H_W$ and accumulates negated base XY displacements to keep sensor measurements world-frame-centered despite internal base resets.

5. **Calibration is cumulative**: a yaw alignment may be followed by a full 3D refinement.

6. **Extrinsic calibration** ($^W\hat{H}_S$ and $^M\hat{H}_L$) may be updated online; task setpoints are recomputed each cycle using current measurement $^S H_M$ and the world anchor $^{WA}H_W$ when relevant.
