// SPDX-FileCopyrightText: Fondazione Istituto Italiano di Tecnologia (IIT)
// SPDX-License-Identifier: BSD-3-Clause

#ifndef BAF_STATE_PROVIDER_H
#define BAF_STATE_PROVIDER_H

#include <hde/interfaces/IHumanState.h>
#include <hde/interfaces/IWearableTargets.h>

#include <yarp/dev/DeviceDriver.h>
#include <yarp/dev/IMultipleWrapper.h>
#include <yarp/os/PeriodicThread.h>

#include <memory>

namespace baf
{
namespace devices
{
class BAFStateProvider;
} // namespace devices
} // namespace baf

/**
 * BAFStateProvider is a YARP device plugin that:
 *   - attaches to an IWear device to receive sensor data
 *   - feeds the data into BAFStateEstimator (wrapping BiomechanicalAnalysis::IK::HumanIK)
 *   - exposes the estimated human state via IHumanState and IWearableTargets
 *
 * Plugin name: baf_state_provider
 *
 * Required configuration parameters:
 *   period          [s]     Control loop period (default 0.017)
 *   urdf            [str]   URDF filename (resolved via YARP ResourceFinder)
 *   floatingBaseFrame [str] Floating base link name (default "Pelvis")
 *
 * BAF IK configuration is embedded directly in the device config. See the
 * BAF IK documentation for the expected task groups and solver parameters.
 *
 * Required configuration group:
 *   [TASK_TO_SENSORS]
 *     Each parameter maps an IK task name (key) to an IWear sensor name (value).
 *     The task type is inferred from the IK TOML; no type annotation is needed here.
 *     Example:
 *       <param name="root_link">TransformServer::pose::root_link_desired</param>
 *       <param name="left_foot">ft::LeftFoot</param>
 *     Supported task types and their required sensor interfaces:
 *       SO3Task / GravityTask  -> IVirtualLinkKinSensor (preferred) or IOrientationSensor
 *       PositionTask           -> IVirtualLinkKinSensor (mandatory)
 *       PoseTask               -> IVirtualLinkKinSensor (preferred) or IPoseSensor
 *       FloorContactTask       -> IForceTorque6DSensor (mandatory)
 *
 * Optional configuration group:
 *   [JOINT_TO_SENSORS]
 *     Each parameter maps a model joint name (key) to an IWear virtual joint sensor name (value).
 *     Example:
 *       <param name="r_knee">vJoint::r_knee_encoder</param>
 *     The reading of each mapped IVirtualJointKinSensor (position and velocity) overrides,
 *     a posteriori, the corresponding joint in the IK-estimated solution every cycle.
 *
 * Runtime calibration RPC port: /<rpcPortPrefix>/BAFStateProvider/rpc:i
 *   Commands: calibrateAll [refFrame]  |  resetAll
 */
class baf::devices::BAFStateProvider final : public yarp::dev::DeviceDriver,
                                             public yarp::dev::IMultipleWrapper,
                                             private yarp::os::PeriodicThread,
                                             public hde::interfaces::IHumanState,
                                             public hde::interfaces::IWearableTargets
{
private:
    class impl;
    std::unique_ptr<impl> pImpl;

public:
    BAFStateProvider();
    ~BAFStateProvider() override;

    // ── DeviceDriver ──────────────────────────────────────────────────────────
    bool open(yarp::os::Searchable& config) override;
    bool close() override;

    // ── IMultipleWrapper ──────────────────────────────────────────────────────
    bool attachAll(const yarp::dev::PolyDriverList& driverList) override;
    bool detachAll() override;

    // ── PeriodicThread ────────────────────────────────────────────────────────
    void run() override;
    void threadRelease() override;

    // ── IHumanState ───────────────────────────────────────────────────────────
    std::vector<std::string> getJointNames() const override;
    size_t getNumberOfJoints() const override;
    std::string getBaseName() const override;
    std::vector<double> getJointPositions() const override;
    std::vector<double> getJointVelocities() const override;
    std::array<double, 3> getBasePosition() const override;
    std::array<double, 4> getBaseOrientation() const override;
    std::array<double, 6> getBaseVelocity() const override;
    std::array<double, 3> getCoMPosition() const override;
    std::array<double, 3> getCoMVelocity() const override;

    // ── IWearableTargets ──────────────────────────────────────────────────────
    std::vector<hde::TargetName> getAllTargetsName() const override;
    std::shared_ptr<hde::WearableSensorTarget> getTarget(const hde::TargetName name) const override;
};

#endif // BAF_STATE_PROVIDER_H
