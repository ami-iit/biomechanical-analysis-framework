# Devices

YARP device plugins built on top of the BiomechanicalAnalysis Framework (BAF).

| Device | Description | README | Example config |
|--------|-------------|--------|-----------------|
| `baf_state_provider` | Whole-body human state estimation using the BAF inverse kinematics solver. | [BAFStateProvider/README.md](BAFStateProvider/README.md) | [conf/xml/BAFStateProvider_example.xml](conf/xml/BAFStateProvider_example.xml) |

## Fixed Kinematic Tasks

The `baf_state_provider` device supports **fixed kinematic references** for IK tasks. Instead of mapping all tasks to wearable sensors via the `TASK_TO_SENSORS` mapping, you can define static targets using `fixed_*` parameters directly within task groups.

Each task type supports specific fixed parameters:
- **SO3Task / GravityTask**: `fixed_rotation_matrix` or `fixed_quaternion`, optional `fixed_angular_velocity`
- **PositionTask**: `fixed_position`, optional `fixed_linear_velocity`
- **PoseTask**: `fixed_position` + (`fixed_rotation_matrix` or `fixed_quaternion`), optional velocities
- **FloorContactTask**: `fixed_wrench`

For configuration examples, see [conf/xml/BAFStateProvider_example_with_fixed_references.xml](conf/xml/BAFStateProvider_example_with_fixed_references.xml) and refer to the `fixed_tasks` parameter documentation in [BAFStateProvider/README.md](BAFStateProvider/README.md).
