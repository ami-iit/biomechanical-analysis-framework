# Devices

YARP device plugins built on top of the BiomechanicalAnalysis Framework (BAF).

| Device | Description | README | Example config |
|--------|-------------|--------|-----------------|
| `baf_state_provider` | Whole-body human state estimation using the BAF inverse kinematics solver. | [BAFStateProvider/README.md](BAFStateProvider/README.md) | [conf/xml/BAFStateProvider_example.xml](conf/xml/BAFStateProvider_example.xml) |

## Fixed Kinematic Tasks

The `baf_state_provider` device supports **fixed kinematic references** for IK tasks through the `BiomechanicalAnalysis::IK::HumanIK` library.

Each task type supports specific fixed parameters:
- **SO3Task / GravityTask**: `const_rotation_matrix` or `const_quaternion`, optional `const_angular_velocity`
- **PositionTask**: `const_position`, optional `const_linear_velocity`
- **PoseTask**: `const_position` + (`const_rotation_matrix` or `const_quaternion`), optional velocities
- **FloorContactTask**: `const_wrench`

For configuration examples, see [conf/xml/BAFStateProvider_example_with_const_references.xml](conf/xml/BAFStateProvider_example_with_const_references.xml) and refer to the fixed-reference documentation in [BAFStateProvider/README.md](BAFStateProvider/README.md).
