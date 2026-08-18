# BAFStateProvider

`baf_state_provider` is a YARP device plugin that wraps `BiomechanicalAnalysis::IK::HumanIK` to perform whole-body human state estimation. It bridges an IWear device and exposes the result via the `IHumanState` and `IWearableTargets` interfaces.

## Parameters

| Parameter | Type | Description |
|-----------|------|-------------|
| `period` | float [s] | Control loop period (default `0.017`) |
| `urdf` | string | URDF filename (resolved via YARP ResourceFinder) |
| `floatingBaseFrame` | string | Floating base link name (default `"Pelvis"`) |
| `forceFixedBase` | bool | If `true`, force output base orientation to identity quaternion and base velocity to zero (default `false`) |
| `rpcPortPrefix` | string | Prefix used for the RPC port and other device ports |
| `tasks` | list | IK tasks to enable, each defined by a corresponding `<group>`/section in the config |

The device forwards its whole configuration to the underlying `BiomechanicalAnalysis::IK::HumanIK` solver (via `BipedalLocomotion::ParametersHandler`), so BAF IK task groups (`SO3Task`, `PositionTask`, `PoseTask`, `FloorContactTask`, `GravityTask`, ...) are configured directly in the device config, there is no separate IK config file.

Constant tasks are owned by the IK library: if a task group defines `const_*` parameters, HumanIK treats it as a world-frame constant and the device does not need to map it to a sensor.

### `TASK_TO_SENSORS` group

Maps each sensor-driven kinematic task name to the wearable sensor providing its target. Task names must match those listed in `tasks` and are conventionally uppercase and end with `_TASK`.

```ini
[TASK_TO_SENSORS]
CHEST_TASK "iFeelSuit::vLink::Node#6"
L_UPPER_ARM_TASK "iFeelSuit::vLink::Node#5"
L_HAND_PALM_TASK "TransformServer::pose::left_glove"
```

### Optional fixed references (`const_*`)

For supported kinematic tasks, you can define a static target directly in the task group using `const_*` parameters. HumanIK interprets those values as world-frame constants and applies them at the beginning of each IK advance cycle.

Supported parameters by task type:

| Task type | Fixed parameters |
|-----------|------------------|
| `SO3Task`, `GravityTask` | `const_rotation_matrix` (9 values, row-major) or `const_quaternion` (`[w x y z]`), optional `const_angular_velocity` (3) |
| `PositionTask` | `const_position` (3), optional `const_linear_velocity` (3) |
| `PoseTask` | `const_position` (3) and `const_rotation_matrix` (9) or `const_quaternion` (`[w x y z]`), optional `const_linear_velocity` (3), optional `const_angular_velocity` (3) |
| `FloorContactTask` | `const_wrench` (6 values: force then torque) |

Validation rules:

- `PoseTask`: position and orientation must be both present.
- Velocity-only fixed parameters are rejected (for example, `const_linear_velocity` without `const_position`).

## RPC

The device exposes an RPC port at `/<rpcPortPrefix>/BAFStateProvider/rpc:i`.

| Command | Description |
|---------|-------------|
| `calibrateAll [refFrame]` | Perform T-pose calibration using the current sensor readings, optionally relative to `refFrame`. |
| `resetAll` | Clear all calibration matrices. |
| `resetWorldAnchorTranslation` | Reset the translation component of the world anchor. |
| `recenterWorldAnchor` | Recenter the world anchor to the current base position. |
| `resetJointState` | Reset the estimated joint state. |
