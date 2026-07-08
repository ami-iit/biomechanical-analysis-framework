# baf-ik

A C++17 YARP device plugin for whole-body human state estimation using the [BiomechanicalAnalysis Framework (BAF)](https://github.com/ami-iit/biomechanical-analysis-framework) inverse kinematics solver.

Developed at [Fondazione Istituto Italiano di Tecnologia (IIT)](https://www.iit.it/).

## Overview

The repository builds a single component:

| Component | Target | Description |
|-----------|--------|-------------|
| `BAFStateProvider` | YARP device plugin | Sensor-agnostic wrapper around `BiomechanicalAnalysis::IK::HumanIK`. It bridges an IWear device and exposes the result via the `IHumanState` and `IWearableTargets` interfaces. Plugin name: `baf_state_provider`. |

## Dependencies

- CMake ≥ 3.16
- C++17 compiler
- [Eigen3](https://eigen.tuxfamily.org)
- [iDynTree](https://github.com/robotology/idyntree)
- [BiomechanicalAnalysisFramework](https://github.com/ami-iit/biomechanical-analysis-framework) ≥ 0.2.3
- [BipedalLocomotionFramework](https://github.com/ami-iit/bipedal-locomotion-framework) ≥ 0.16.0
- [YARP](https://github.com/robotology/yarp) (components: `os`, `dev`, `init`)
- [HumanDynamicsEstimation](https://github.com/robotology/human-dynamics-estimation)
- [IWear](https://github.com/robotology/iWear)

## Build and Install

Configure, build, and install into a local `build/install` prefix:

```bash
cmake -B build -S . -DCMAKE_INSTALL_PREFIX=build/install
cmake --build build --config Release --parallel
cmake --build build --config Release --target install
```

### Make the YARP device plugin visible

After installing, tell YARP where to find the plugin manifest and shared library.

**Linux / macOS**
```bash
export YARP_DATA_DIRS=<repo>/build/install/share/yarp:${YARP_DATA_DIRS}
export LD_LIBRARY_PATH=<repo>/build/install/lib/yarp:${LD_LIBRARY_PATH}   # Linux
# export DYLD_LIBRARY_PATH=<repo>/build/install/lib/yarp:${DYLD_LIBRARY_PATH}  # macOS
```

Add these to your shell RC file (e.g. `~/.bashrc`) to make them persistent.

**Windows (cmd)**
```bat
set YARP_DATA_DIRS=<repo>\build\install\share\yarp;%YARP_DATA_DIRS%
set PATH=<repo>\build\install\bin;%PATH%
```

Replace `<repo>` with the absolute path to this repository.

You can verify the plugin is found with:

```bash
yarp plugin baf_state_provider
```

## Configuration (`BAFStateProvider`)

The device is configured via a YARP `.ini` or `.xml` file. Required parameters:

| Parameter | Type | Description |
|-----------|------|-------------|
| `period` | float [s] | Control loop period (default `0.017`) |
| `urdf` | string | URDF filename (resolved via YARP ResourceFinder) |
| `floatingBaseFrame` | string | Floating base link name (default `"Pelvis"`) |
| `ikConfigFile` | string | Absolute path to the BAF IK TOML configuration file (optional if embedded in device config) |

### Required group `[TASK_TO_SENSORS]`

Maps each IK task name to a sensor name. Task names must match those defined in the IK configuration `tasks` parameter.

```ini
[TASK_TO_SENSORS]
taskName1 "sensorSource1::sensorType::sensorName"
taskName2 "sensorSource2::sensorType::sensorName"
```

Example:
```ini
[TASK_TO_SENSORS]
chest "iFeelSuit::vLink::Node#6"
l_upper_arm "iFeelSuit::vLink::Node#5"
l_hand_palm "TransformServer::pose::left_glove"
```

### Runtime calibration RPC

The device exposes an RPC port at `/<rpcPortPrefix>/BAFStateProvider/rpc:i`.

| Command | Description |
|---------|-------------|
| `calibrateAll [refFrame]` | Perform T-pose calibration using the current sensor readings. |
| `resetAll` | Clear all calibration matrices. |
