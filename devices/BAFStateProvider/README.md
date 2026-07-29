# BAFStateProvider

`baf_state_provider` is a YARP device plugin that wraps `BiomechanicalAnalysis::IK::HumanIK` to perform whole-body human state estimation. It bridges an IWear device and exposes the result via the `IHumanState` and `IWearableTargets` interfaces.

## Parameters

| Parameter | Type | Description |
|-----------|------|-------------|
| `period` | float [s] | Control loop period (default `0.017`) |
| `urdf` | string | URDF filename (resolved via YARP ResourceFinder) |
| `floatingBaseFrame` | string | Floating base link name (default `"Pelvis"`) |
| `rpcPortPrefix` | string | Prefix used for the RPC port and other device ports |
| `tasks` | list | IK tasks to enable, each defined by a corresponding `<group>`/section in the config |
| `const_tasks` | list | Optional fixed-only kinematic tasks. They must define `const_*` parameters and are not mapped in `TASK_TO_SENSORS`. |

The device forwards its whole configuration to the underlying `BiomechanicalAnalysis::IK::HumanIK` solver (via `BipedalLocomotion::ParametersHandler`), so BAF IK task groups (`SO3Task`, `PositionTask`, `PoseTask`, `FloorContactTask`, `GravityTask`, ...) are configured directly in the device config, there is no separate IK config file.

### `TASK_TO_SENSORS` group

Maps each sensor-driven kinematic task name to the wearable sensor providing its target. Task names must match those listed in `tasks` and are conventionally uppercase and end with `_TASK`.

```ini
[TASK_TO_SENSORS]
CHEST_TASK "iFeelSuit::vLink::Node#6"
L_UPPER_ARM_TASK "iFeelSuit::vLink::Node#5"
L_HAND_PALM_TASK "TransformServer::pose::left_glove"
```

Tasks listed in `const_tasks` must not appear in this group.

### `const_tasks` list

Use `const_tasks` to define kinematic targets that are configured with static `const_*` values and kept separate from sensor mapping. Each task name in `const_tasks` must have a corresponding task group and must define a valid fixed reference.

Validation rules for `const_tasks`:

- A task cannot appear in both `tasks` and `const_tasks`.
- A task in `const_tasks` cannot appear in `TASK_TO_SENSORS`.
- A task in `const_tasks` must define a valid `const_*` reference.
- Supported task types are `SO3Task`, `GravityTask`, `PositionTask`, `PoseTask`, `FloorContactTask`.

### Optional fixed references (`const_*`)

For supported kinematic tasks, you can define a static target directly in the task group using `const_*` parameters.

This can be used in two ways:

- Inside `tasks`: the task remains an IK task and can still use a fixed target as input.
- Inside `const_tasks`: the task is configured as fixed-only and is not sensor-mapped.

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
