# HumanIK Calibration, World Alignment, and Runtime Loop

This guide explains how to use HumanIK calibration in practice, how world alignment affects task setpoints, and how to structure the typical runtime loop.

## Scope

This page covers:

- world alignment and calibration concepts used by HumanIK;
- the behavior of orientation, gravity, pose, position, and floor-contact tasks;
- the intended sequence around `calibrateWorldYaw()`, `calibrateAllWithWorld()`, task updates, and `advance()`.

## Frames and Alignment

> For the full mathematical treatment of frame composition, calibration procedures, and per-task setpoint formulas, see [ik-frame-mathematics.md](ik-frame-mathematics.md).

HumanIK combines sensor measurements with task-specific calibration data before sending setpoints to the solver.

The main concepts are:

- `World frame`: the inertial frame used by the IK solver.
- `Sensor frame`: the inertial frame in which the incoming measurement is expressed.
- `Measured frame`: the raw frame reported by the sensor stream.
- `Link or task frame`: the model frame constrained by the IK task.
- `World anchor`: a translation-only offset used to keep pose and position task targets consistent when the internal base state is recentered.

At a high level, HumanIK composes three things:

1. a world-to-sensor alignment estimated during calibration;
2. the current sensor measurement;
3. task extrinsics.

For orientation and gravity tasks, calibration mainly affects the rotational alignment between the solver world and the sensor world.

For pose and position tasks, calibration also interacts with the world anchor so that absolute targets remain meaningful after the base is reset internally.

## Task Behavior Summary

### Orientation tasks

Orientation tasks consume orientation and angular velocity data. Calibration updates the mapping from the sensor world to the IK world and may also refine the fixed sensor-to-link rotation used by the task.

These tasks are updated at runtime with `updateOrientationTask()` or in bulk with `updateOrientationAndGravityTasks()`.

### Gravity tasks

Gravity tasks use the calibrated orientation to extract a gravity direction consistent with the solver world. They are useful when the measurement is orientation-based but the constraint should act on the gravity direction rather than the full link orientation.

These tasks are updated with `updateGravityTask()`.

### Position tasks

Position tasks consume measured position and linear velocity data. Their targets are interpreted through the calibrated world alignment and any configured positional extrinsic.

Unlike pure orientation tasks, position targets are also affected by the world anchor. This is what keeps targets stable after calibration or recentering.

These tasks are updated with `updatePositionTask()` or `updatePositionTasks()`.

### Pose tasks

Pose tasks consume full pose and mixed velocity data. Their rotational part follows the same world-alignment logic used for orientation tasks, while their translational part is combined with the configured extrinsic and the current world anchor.

These tasks are updated with `updatePoseTask()` or `updatePoseTasks()`.

### Floor-contact tasks

Floor-contact tasks are not calibrated from sensor orientation directly. They act as state-dependent position constraints that become active when the measured vertical force crosses a configured threshold.

## Calibration Workflow

The public calibration flow is centered on three functions declared in [src/IK/include/BiomechanicalAnalysis/IK/InverseKinematics.h](../src/IK/include/BiomechanicalAnalysis/IK/InverseKinematics.h):

- `clearCalibrationMatrices()`
- `calibrateWorldYaw()`
- `calibrateAllWithWorld()`

### 1. Optional reset with `clearCalibrationMatrices()`

Use `clearCalibrationMatrices()` when you want to discard previously accumulated calibration and return task-specific calibration data to its initialized state.

In practice, this is the safest entry point before repeating a calibration sequence with a different setup or after a failed session.

### 2. Yaw alignment with `calibrateWorldYaw()`

`calibrateWorldYaw()` performs a yaw-only alignment between the sensor world and the IK world.

This is useful as a first-stage calibration when the sensor vertical axis is already consistent with gravity and the remaining mismatch is mainly an azimuth offset.

Assumptions are:

- the robot/human reference data is expected to come at the calibration pose;
- orientation data should already be gravity-consistent;
- this stage prepares the state used by the full calibration that follows.

### 3. Full calibration with `calibrateAllWithWorld()`

`calibrateAllWithWorld()` applies the full calibration update and also updates the world anchor used by pose and position tasks.

From a user perspective, this stage does two important things:

- it refines the task calibration using the current measured data and the model state;
- it captures the current base translation into the world anchor so world-frame targets remain centered after the internal base is reset.

The `frameRef` argument identifies the model frame used as the user-facing reference for the world alignment step. It is optional.

### Order matters

The intended usage is:

1. collect the measurements needed for calibration;
2. call `calibrateWorldYaw()`;
3. call `calibrateAllWithWorld()`;
4. continue the runtime loop and let the next `advance()` consume the updated calibration state.

This ordering is reflected in both the example and the IK tests.

## World Anchor and Base Recentering

Pose and position tasks support a world-alignment.

When HumanIK recenters the internal base state, the solver world would otherwise drift away from the absolute targets coming from sensors. To avoid that, HumanIK stores a translation-only world anchor.

The user-facing effect is:

- the internal base can be reset or recentered;
- pose and position task targets stay consistent in world coordinates;
- floor-contact setpoints are refreshed to match the updated internal state.


## Typical Runtime Loop

The practical runtime pattern at a high level is a loop with the following steps:

1. initialize the model, tasks, and HumanIK instance;
2. optionally wait for a dedicated calibration event such as a T-pose request;
3. when calibration is requested, gather the current measurements and run the calibration sequence;
4. at each cycle, update all active tasks with fresh measurements;
5. call `advance()` to solve the IK problem and integrate the state;
6. read back joint and base outputs as needed.

### Pseudocode

```cpp
HumanIK ik;
ik.initialize(paramHandler, kinDyn);
ik.setDt(dt);

while (running)
{
    if (shouldCalibrate)
    {
        std::unordered_map<int, BiomechanicalAnalysis::IK::nodeData> nodeData;
        // Fill nodeData with the current orientation measurements.

        ik.calibrateWorldYaw(nodeData);
        ik.calibrateAllWithWorld(nodeData, "LeftFoot");
    }

    // Update orientation, gravity, floor-contact, pose, position,
    // and regularization tasks with current measurements.

    ik.advance();

    // Read joint positions, velocities, base pose, and base position.
}
```

### What changes after calibration

Immediately after calibration:

- orientation and gravity tasks use the updated world alignment;
- pose and position tasks use the updated world alignment and anchor translation;
- the next `advance()` handles the state transition associated with the calibration update.

This is why calibration is usually treated as a dedicated event inside the runtime loop rather than a background operation hidden inside regular task updates.

## Practical Recommendations

- Keep calibration as an explicit phase in your application logic.
- Calibrate from a known pose and with measurements that satisfy the gravity-alignment assumption.
- Expect a jump in joint position and base pose outputs on the first `advance()` after calibration: both calibration functions reset the internal integrator to identity base + calibration joints before computing new calibration matrices. Any downstream system (visualisation, logging, control) should be prepared to handle this discontinuity.
- Trigger calibration only when the subject is actually at the intended calibration pose; triggering it during free motion will still cause the output jump but the resulting calibration matrices will be incorrect.
- Re-run calibration when the mounting assumptions or the global alignment reference change.
- Treat pose and position tasks as world-aligned tasks: if your application depends on absolute positions, the world anchor behaviour is part of the contract.
- Refresh all measurement-driven tasks every cycle before calling `advance()`.

