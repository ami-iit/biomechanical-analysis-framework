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

### `TASK_TO_SENSORS` group

Maps each IK task name to the wearable sensor providing its target. Task names must match those listed in `tasks` and are conventionally uppercase and end with `_TASK`.

```ini
[TASK_TO_SENSORS]
CHEST_TASK "iFeelSuit::vLink::Node#6"
L_UPPER_ARM_TASK "iFeelSuit::vLink::Node#5"
L_HAND_PALM_TASK "TransformServer::pose::left_glove"
```

## RPC

The device exposes an RPC port at `/<rpcPortPrefix>/BAFStateProvider/rpc:i`.

| Command | Description |
|---------|-------------|
| `calibrateAll [refFrame]` | Perform T-pose calibration using the current sensor readings, optionally relative to `refFrame`. |
| `resetAll` | Clear all calibration matrices. |
| `resetWorldAnchorTranslation` | Reset the translation component of the world anchor. |
| `recenterWorldAnchor` | Recenter the world anchor to the current base position. |
| `resetJointState` | Reset the estimated joint state. |
