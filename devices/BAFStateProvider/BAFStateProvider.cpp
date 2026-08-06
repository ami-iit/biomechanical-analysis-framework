// SPDX-FileCopyrightText: Fondazione Istituto Italiano di Tecnologia (IIT)
// SPDX-License-Identifier: BSD-3-Clause

#include "BAFStateProvider.h"
#include "BAFStateEstimator.h"

#include <BiomechanicalAnalysis/IK/InverseKinematics.h>

#include <BipedalLocomotion/ParametersHandler/YarpImplementation.h>

#include <Wearable/IWear/IWear.h>
#include <Wearable/IWear/Sensors/IForceTorque6DSensor.h>
#include <Wearable/IWear/Sensors/IOrientationSensor.h>
#include <Wearable/IWear/Sensors/IPoseSensor.h>
#include <Wearable/IWear/Sensors/IVirtualJointKinSensor.h>
#include <Wearable/IWear/Sensors/IVirtualLinkKinSensor.h>

#include <hde/interfaces/IWearableTargets.h>

#include <iDynTree/EigenHelpers.h>
#include <iDynTree/KinDynComputations.h>
#include <iDynTree/ModelLoader.h>

#include <yarp/os/Bottle.h>
#include <yarp/os/LogStream.h>
#include <yarp/os/ResourceFinder.h>
#include <yarp/os/RpcServer.h>

#include <Eigen/Core>

#include <algorithm>
#include <array>
#include <atomic>
#include <deque>
#include <mutex>
#include <optional>
#include <string>
#include <thread>
#include <unordered_map>
#include <unordered_set>
#include <variant>
#include <vector>

using namespace wearable;
using namespace wearable::sensor;

constexpr auto LogPrefix = "[BAFStateProvider]";
using KinematicTaskKind = BAFStateEstimator::KinematicTaskKind;

// ─────────────────────────────────────────────────────────────────────────────
// Private implementation (pImpl)
// ─────────────────────────────────────────────────────────────────────────────

/// Holds exactly one resolved IWear sensor pointer (or monostate before attachAll).
using SensorVariant = std::variant<std::monostate,
                                   wearable::SensorPtr<const IVirtualLinkKinSensor>,
                                   wearable::SensorPtr<const IOrientationSensor>,
                                   wearable::SensorPtr<const IPoseSensor>,
                                   wearable::SensorPtr<const IForceTorque6DSensor>>;

/**
 * Binds one IK task to one IWear sensor.
 * kind is set at open(); sensor is populated in attachAll().
 */
struct SensorTarget
{
    wearable::WearableName sensorName;
    std::string taskName; ///< IK task name (key in TASK_TO_SENSORS group)
    std::string modelLinkName; ///< URDF link name (= frame_name in IK TOML)
    double contactThreshold{0.0}; ///< only meaningful for FloorContact targets
    KinematicTaskKind kind{KinematicTaskKind::Orientation};
    SensorVariant sensor; ///< resolved in attachAll(); monostate until then
};

/**
 * Binds one model joint to one IWear virtual joint sensor. Used to override, a posteriori,
 * the joint position/velocity computed by the IK solver with a value coming directly from
 * an external device (e.g. an encoder exposed as a wearable virtual joint sensor).
 */
struct JointSensorTarget
{
    wearable::WearableName sensorName;
    std::string jointName; ///< model joint name (matches an entry of BAFStateEstimator::getJointNames())
    size_t jointIndex{0}; ///< resolved index into the joint position/velocity vectors
    wearable::SensorPtr<const IVirtualJointKinSensor> sensor; ///< resolved in attachAll(); nullptr until then
};

/**
 * Thread-safe snapshot of the estimated state, written by run() and read by
 * the IHumanState getters.
 */
struct HumanStateSolution
{
    std::vector<double> jointPositions;
    std::vector<double> jointVelocities;
    std::array<double, 3> basePosition{};
    std::array<double, 4> baseOrientation{}; ///< [w, x, y, z]
    std::array<double, 6> baseVelocity{}; ///< [vx, vy, vz, ωx, ωy, ωz]
    std::array<double, 3> comPosition{};
    std::array<double, 3> comVelocity{};
};

class baf::devices::BAFStateProvider::impl
{
public:
    struct FixedTaskReference
    {
        KinematicTaskKind kind{KinematicTaskKind::Orientation};
        BiomechanicalAnalysis::IK::nodeData orientationData;
        BiomechanicalAnalysis::IK::positionData positionData;
        BiomechanicalAnalysis::IK::poseData poseData;
        Eigen::Matrix<double, 6, 1> wrench{Eigen::Matrix<double, 6, 1>::Zero()};
    };

    struct TaskInfo
    {
        KinematicTaskKind kind;
        std::string modelLinkName;
        double contactThreshold{0.0};
        std::optional<FixedTaskReference> fixedReference;
    };

    enum class RpcCommandType
    {
        CalibrateAll,
        ResetAll,
        ResetWorldAnchorTranslation,
        RecenterWorldAnchor,
        ResetJointState
    };

    struct PendingRpcCommand
    {
        RpcCommandType type;
        std::string refFrame{""};
    };

    // ── Configuration ─────────────────────────────────────────────────────────
    std::string baseName{"Pelvis"};
    std::vector<std::string> jointNames;

    std::vector<SensorTarget> sensorTargets;
    std::vector<JointSensorTarget> jointSensorTargets;

    // ── Core estimator ────────────────────────────────────────────────────────
    BAFStateEstimator estimator;

    // ── IWear ─────────────────────────────────────────────────────────────────
    wearable::IWear* iWear{nullptr};

    // ── State (written by run(), read by getters) ─────────────────────────────
    mutable std::mutex solutionMutex;
    HumanStateSolution solution;

    // ── IWearableTargets: one WearableSensorTarget per target node (orientation or floorContact)
    std::unordered_map<hde::TargetName, std::shared_ptr<hde::WearableSensorTarget>> wearableTargets;

    // ── RPC ───────────────────────────────────────────────────────────────────
    yarp::os::RpcServer rpcPort;
    std::atomic<bool> rpcThreadRunning{false};
    std::thread rpcThread;
    std::mutex rpcCommandsMutex;
    std::deque<PendingRpcCommand> pendingRpcCommands;

    // ── Working buffers reused each cycle ─────────────────────────────────────
    std::unordered_map<std::string, BiomechanicalAnalysis::IK::nodeData> nodeDataMap;
    std::unordered_map<std::string, Eigen::Matrix<double, 6, 1>> ftDataMap;
    std::unordered_map<std::string, BiomechanicalAnalysis::IK::positionData> positionDataMap;
    std::unordered_map<std::string, BiomechanicalAnalysis::IK::poseData> poseDataMap;
    // ── Helpers ───────────────────────────────────────────────────────────────

    /** Convert a wearable Quaternion [w,x,y,z] to manif::SO3d. */
    static manif::SO3d quaternionToSO3(const wearable::Quaternion& q)
    {
        // wearable::Quaternion = std::array<double,4> with layout [w, x, y, z]
        return manif::SO3d(Eigen::Quaterniond(q[0], q[1], q[2], q[3]));
    }

    /** Convert a wearable Vector3 to manif::SO3Tangentd (angular velocity). */
    static manif::SO3Tangentd vector3ToSO3Tangent(const wearable::Vector3& v)
    {
        manif::SO3Tangentd tangent;
        tangent.coeffs() << v[0], v[1], v[2];
        return tangent;
    }

    static bool buildTaskInfoMap(const std::shared_ptr<BipedalLocomotion::ParametersHandler::IParametersHandler>& ikParams,
                                 std::unordered_map<std::string, TaskInfo>& taskInfoMap)
    {
        auto readSizedVector = [](const std::shared_ptr<BipedalLocomotion::ParametersHandler::IParametersHandler>& taskGroup,
                                  const std::string& parameterName,
                                  std::size_t expectedSize,
                                  std::vector<double>& out,
                                  const std::string& taskName) -> bool {
            if (!taskGroup->getParameter(parameterName, out))
            {
                return false;
            }
            if (out.size() != expectedSize)
            {
                yError() << LogPrefix << "Task" << taskName << "parameter" << parameterName << "has size" << out.size() << "but expected"
                         << expectedSize;
                return false;
            }
            return true;
        };

        auto parseFixedRotation = [&](const std::shared_ptr<BipedalLocomotion::ParametersHandler::IParametersHandler>& taskGroup,
                                      const std::string& taskName,
                                      manif::SO3d& rotation,
                                      bool& foundRotation) -> bool {
            std::vector<double> fixedRotationMatrix;
            if (readSizedVector(taskGroup, "const_rotation_matrix", 9, fixedRotationMatrix, taskName))
            {
                const auto matrix = Eigen::Map<Eigen::Matrix<double, 3, 3, Eigen::RowMajor>>(fixedRotationMatrix.data());
                rotation = manif::SO3d(Eigen::Quaterniond(matrix));
                foundRotation = true;
                return true;
            }

            std::vector<double> fixedQuaternion;
            if (readSizedVector(taskGroup, "const_quaternion", 4, fixedQuaternion, taskName))
            {
                // The quaternion layout is [w, x, y, z].
                const Eigen::Quaterniond quat(fixedQuaternion[0], fixedQuaternion[1], fixedQuaternion[2], fixedQuaternion[3]);
                rotation = manif::SO3d(quat);
                foundRotation = true;
                return true;
            }

            foundRotation = false;
            return true;
        };

        std::vector<std::string> tasks;
        if (!ikParams->getParameter("tasks", tasks))
        {
            yError() << LogPrefix << "Cannot read 'tasks' list from IK config";
            return false;
        }

        taskInfoMap.clear();
        auto parseTask = [&](const std::string& taskName) -> bool {
            auto taskGroupWeak = ikParams->getGroup(taskName);
            auto taskGroup = taskGroupWeak.lock();
            if (!taskGroup)
            {
                yError() << LogPrefix << "Cannot read task group:" << taskName;
                return false;
            }

            std::string type;
            if (!taskGroup->getParameter("type", type))
            {
                yError() << LogPrefix << "Task" << taskName << "is missing 'type'";
                return false;
            }

            TaskInfo info;
            bool isSupportedKinematicTask{true};

            if (type == "SO3Task")
            {
                if (!taskGroup->getParameter("frame_name", info.modelLinkName))
                {
                    yError() << LogPrefix << "SO3Task" << taskName << "is missing 'frame_name'";
                    return false;
                }
                info.kind = KinematicTaskKind::Orientation;

                bool hasFixedRotation{false};
                manif::SO3d fixedRotation;
                if (!parseFixedRotation(taskGroup, taskName, fixedRotation, hasFixedRotation))
                {
                    return false;
                }

                std::vector<double> fixedAngularVelocity;
                const bool hasFixedAngularVelocity
                    = readSizedVector(taskGroup, "const_angular_velocity", 3, fixedAngularVelocity, taskName);

                if (hasFixedRotation)
                {
                    FixedTaskReference fixedRef;
                    fixedRef.kind = KinematicTaskKind::Orientation;
                    fixedRef.orientationData.I_R_IMU = fixedRotation;
                    if (hasFixedAngularVelocity)
                    {
                        fixedRef.orientationData.I_omega_IMU.coeffs() << fixedAngularVelocity[0], fixedAngularVelocity[1],
                            fixedAngularVelocity[2];
                    }
                    info.fixedReference = fixedRef;
                } else if (hasFixedAngularVelocity)
                {
                    yError() << LogPrefix << "Task" << taskName
                             << "defines const_angular_velocity but no const_rotation_matrix/const_quaternion";
                    return false;
                }
            } else if (type == "GravityTask")
            {
                if (!taskGroup->getParameter("target_frame_name", info.modelLinkName))
                {
                    yError() << LogPrefix << "GravityTask" << taskName << "is missing 'target_frame_name'";
                    return false;
                }
                info.kind = KinematicTaskKind::Orientation;

                bool hasFixedRotation{false};
                manif::SO3d fixedRotation;
                if (!parseFixedRotation(taskGroup, taskName, fixedRotation, hasFixedRotation))
                {
                    return false;
                }

                std::vector<double> fixedAngularVelocity;
                const bool hasFixedAngularVelocity
                    = readSizedVector(taskGroup, "const_angular_velocity", 3, fixedAngularVelocity, taskName);

                if (hasFixedRotation)
                {
                    FixedTaskReference fixedRef;
                    fixedRef.kind = KinematicTaskKind::Orientation;
                    fixedRef.orientationData.I_R_IMU = fixedRotation;
                    if (hasFixedAngularVelocity)
                    {
                        fixedRef.orientationData.I_omega_IMU.coeffs() << fixedAngularVelocity[0], fixedAngularVelocity[1],
                            fixedAngularVelocity[2];
                    }
                    info.fixedReference = fixedRef;
                } else if (hasFixedAngularVelocity)
                {
                    yError() << LogPrefix << "Task" << taskName
                             << "defines const_angular_velocity but no const_rotation_matrix/const_quaternion";
                    return false;
                }
            } else if (type == "FloorContactTask")
            {
                if (!taskGroup->getParameter("frame_name", info.modelLinkName))
                {
                    yError() << LogPrefix << "FloorContactTask" << taskName << "is missing 'frame_name'";
                    return false;
                }
                if (!taskGroup->getParameter("vertical_force_threshold", info.contactThreshold))
                {
                    yError() << LogPrefix << "FloorContactTask" << taskName << "is missing 'vertical_force_threshold'";
                    return false;
                }
                info.kind = KinematicTaskKind::FloorContact;

                std::vector<double> fixedWrench;
                if (readSizedVector(taskGroup, "const_wrench", 6, fixedWrench, taskName))
                {
                    FixedTaskReference fixedRef;
                    fixedRef.kind = KinematicTaskKind::FloorContact;
                    fixedRef.wrench << fixedWrench[0], fixedWrench[1], fixedWrench[2], fixedWrench[3], fixedWrench[4], fixedWrench[5];
                    info.fixedReference = fixedRef;
                }
            } else if (type == "PositionTask")
            {
                if (!taskGroup->getParameter("frame_name", info.modelLinkName))
                {
                    yError() << LogPrefix << "PositionTask" << taskName << "is missing 'frame_name'";
                    return false;
                }
                info.kind = KinematicTaskKind::Position;

                std::vector<double> fixedPosition;
                const bool hasFixedPosition = readSizedVector(taskGroup, "const_position", 3, fixedPosition, taskName);

                std::vector<double> fixedLinearVelocity;
                const bool hasFixedLinearVelocity = readSizedVector(taskGroup, "const_linear_velocity", 3, fixedLinearVelocity, taskName);

                if (hasFixedPosition)
                {
                    FixedTaskReference fixedRef;
                    fixedRef.kind = KinematicTaskKind::Position;
                    fixedRef.positionData.S_p_M = Eigen::Vector3d(fixedPosition[0], fixedPosition[1], fixedPosition[2]);
                    if (hasFixedLinearVelocity)
                    {
                        fixedRef.positionData.S_v_M
                            = Eigen::Vector3d(fixedLinearVelocity[0], fixedLinearVelocity[1], fixedLinearVelocity[2]);
                    }
                    info.fixedReference = fixedRef;
                } else if (hasFixedLinearVelocity)
                {
                    yError() << LogPrefix << "Task" << taskName << "defines const_linear_velocity but no const_position";
                    return false;
                }
            } else if (type == "PoseTask")
            {
                if (!taskGroup->getParameter("frame_name", info.modelLinkName))
                {
                    yError() << LogPrefix << "PoseTask" << taskName << "is missing 'frame_name'";
                    return false;
                }
                info.kind = KinematicTaskKind::Pose;

                std::vector<double> fixedPosition;
                const bool hasFixedPosition = readSizedVector(taskGroup, "const_position", 3, fixedPosition, taskName);

                bool hasFixedRotation{false};
                manif::SO3d fixedRotation;
                if (!parseFixedRotation(taskGroup, taskName, fixedRotation, hasFixedRotation))
                {
                    return false;
                }

                std::vector<double> fixedLinearVelocity;
                const bool hasFixedLinearVelocity = readSizedVector(taskGroup, "const_linear_velocity", 3, fixedLinearVelocity, taskName);

                std::vector<double> fixedAngularVelocity;
                const bool hasFixedAngularVelocity
                    = readSizedVector(taskGroup, "const_angular_velocity", 3, fixedAngularVelocity, taskName);

                if (hasFixedPosition || hasFixedRotation)
                {
                    if (!(hasFixedPosition && hasFixedRotation))
                    {
                        yError() << LogPrefix << "Task" << taskName
                                 << "must define both const_position and const_rotation_matrix/const_quaternion";
                        return false;
                    }

                    FixedTaskReference fixedRef;
                    fixedRef.kind = KinematicTaskKind::Pose;
                    fixedRef.poseData.S_H_M
                        = manif::SE3d(Eigen::Vector3d(fixedPosition[0], fixedPosition[1], fixedPosition[2]), fixedRotation.quat());
                    if (hasFixedLinearVelocity)
                    {
                        fixedRef.poseData.S_v_M.lin() << fixedLinearVelocity[0], fixedLinearVelocity[1], fixedLinearVelocity[2];
                    }
                    if (hasFixedAngularVelocity)
                    {
                        fixedRef.poseData.S_v_M.ang() << fixedAngularVelocity[0], fixedAngularVelocity[1], fixedAngularVelocity[2];
                    }
                    info.fixedReference = fixedRef;
                } else if (hasFixedLinearVelocity || hasFixedAngularVelocity)
                {
                    yError() << LogPrefix << "Task" << taskName
                             << "defines fixed velocities but no const_position/const_rotation_matrix/const_quaternion";
                    return false;
                }
            } else
            {
                isSupportedKinematicTask = false;
            }

            if (!isSupportedKinematicTask)
            {
                return true;
            }

            taskInfoMap[taskName] = std::move(info);
            return true;
        };

        for (const auto& taskName : tasks)
        {
            if (!parseTask(taskName))
            {
                return false;
            }
        }

        return true;
    }

    void rpcLoop()
    {
        while (rpcThreadRunning.load())
        {
            yarp::os::Bottle cmd, reply;
            if (!rpcPort.read(cmd))
            {
                if (!rpcThreadRunning.load())
                {
                    break;
                }
                continue;
            }

            if (cmd.size() == 0)
            {
                reply.addString("Empty command. Available: calibrateAll [refFrame], resetAll, resetWorldAnchorTranslation, "
                                "recenterWorldAnchor, resetJointState");
            } else
            {
                const std::string command = cmd.get(0).asString();
                if (command == "calibrateAll")
                {
                    std::string refFrame = (cmd.size() > 1) ? cmd.get(1).asString() : "";
                    {
                        std::lock_guard<std::mutex> lock(rpcCommandsMutex);
                        pendingRpcCommands.push_back({RpcCommandType::CalibrateAll, refFrame});
                    }
                    reply.addString("ok");
                } else if (command == "resetAll")
                {
                    {
                        std::lock_guard<std::mutex> lock(rpcCommandsMutex);
                        pendingRpcCommands.push_back({RpcCommandType::ResetAll, ""});
                    }
                    reply.addString("ok");
                } else if (command == "resetWorldAnchorTranslation")
                {
                    {
                        std::lock_guard<std::mutex> lock(rpcCommandsMutex);
                        pendingRpcCommands.push_back({RpcCommandType::ResetWorldAnchorTranslation, ""});
                    }
                    reply.addString("ok");
                } else if (command == "recenterWorldAnchor")
                {
                    {
                        std::lock_guard<std::mutex> lock(rpcCommandsMutex);
                        pendingRpcCommands.push_back({RpcCommandType::RecenterWorldAnchor, ""});
                    }
                    reply.addString("ok");
                } else if (command == "resetJointState")
                {
                    {
                        std::lock_guard<std::mutex> lock(rpcCommandsMutex);
                        pendingRpcCommands.push_back({RpcCommandType::ResetJointState, ""});
                    }
                    reply.addString("ok");
                } else
                {
                    reply.addString("Unknown command. Available: calibrateAll [refFrame], resetAll, resetWorldAnchorTranslation, "
                                    "recenterWorldAnchor, resetJointState");
                }
            }

            rpcPort.reply(reply);
        }
    }
};

// ─────────────────────────────────────────────────────────────────────────────
// Constructor / Destructor
// ─────────────────────────────────────────────────────────────────────────────

baf::devices::BAFStateProvider::BAFStateProvider()
    : yarp::os::PeriodicThread(0.017)
    , pImpl(std::make_unique<impl>())
{
}

baf::devices::BAFStateProvider::~BAFStateProvider()
{
    close();
}

// ─────────────────────────────────────────────────────────────────────────────
// DeviceDriver: open / close
// ─────────────────────────────────────────────────────────────────────────────

bool baf::devices::BAFStateProvider::open(yarp::os::Searchable& config)
{
    // ── Period ────────────────────────────────────────────────────────────────
    double period = config.check("period", yarp::os::Value(0.017)).asFloat64();
    setPeriod(period);
    const bool startAfterOpening = config.check("startAfterOpening", yarp::os::Value(false)).asBool();

    // ── Floating base frame ───────────────────────────────────────────────────
    pImpl->baseName = config.check("floatingBaseFrame", yarp::os::Value("Pelvis")).asString();

    // ── URDF ──────────────────────────────────────────────────────────────────
    if (!config.check("urdf"))
    {
        yError() << LogPrefix << "Missing required parameter 'urdf'";
        return false;
    }
    yarp::os::ResourceFinder rf;
    rf.setDefaultContext("HumanDynamicsEstimation");
    std::string urdfFilePath = rf.findFileByName(config.find("urdf").asString());
    if (urdfFilePath.empty())
    {
        yError() << LogPrefix << "Cannot find URDF file:" << config.find("urdf").asString();
        return false;
    }

    // ── IK configuration ────────────────────────
    auto ikParams = std::make_shared<BipedalLocomotion::ParametersHandler::YarpImplementation>(config);

    std::unordered_map<std::string, impl::TaskInfo> taskInfoMap;
    if (!impl::buildTaskInfoMap(ikParams, taskInfoMap))
    {
        return false;
    }

    std::unordered_set<std::string> configuredTaskNames;

    // ── TASK_TO_SENSORS group ─────────────────────────────────────────────────
    pImpl->sensorTargets.clear();
    if (config.check("TASK_TO_SENSORS"))
    {
        yarp::os::Bottle& group = config.findGroup("TASK_TO_SENSORS");
        // group.get(0) is the group name; task-sensor pairs start at index 1
        for (int i = 1; i < static_cast<int>(group.size()); ++i)
        {
            yarp::os::Bottle* entry = group.get(i).asList();
            if (!entry || entry->size() < 2)
            {
                yWarning() << LogPrefix << "TASK_TO_SENSORS: skipping malformed entry at index" << i;
                continue;
            }

            const std::string taskName = entry->get(0).asString();
            const std::string sensorName = entry->get(1).asString();

            auto taskIt = taskInfoMap.find(taskName);
            if (taskIt == taskInfoMap.end())
            {
                yWarning() << LogPrefix << "TASK_TO_SENSORS: task '" << taskName << "' not found in IK config, ignoring mapping";
                continue;
            }
            if (!configuredTaskNames.insert(taskName).second)
            {
                yError() << LogPrefix << "TASK_TO_SENSORS: duplicate task name '" << taskName << "'";
                return false;
            }

            SensorTarget t;
            t.sensorName = sensorName;
            t.taskName = taskName;
            t.modelLinkName = taskIt->second.modelLinkName;
            t.kind = taskIt->second.kind;
            t.contactThreshold = taskIt->second.contactThreshold;
            pImpl->sensorTargets.push_back(std::move(t));
        }
    }

    // ── Error if a task has neither sensor mapping nor fixed reference ───────
    for (const auto& [name, info] : taskInfoMap)
    {
        if (configuredTaskNames.find(name) == configuredTaskNames.end() && !info.fixedReference.has_value())
        {
            yError() << LogPrefix << "Task '" << name
                     << "' is listed in the IK config 'tasks' parameter but has neither sensor mapping"
                        " in TASK_TO_SENSORS nor a fixed reference";
            return false;
        }
    }

    // ── Load URDF model ───────────────────────────────────────────────────────
    iDynTree::ModelLoader mdlLoader;
    if (!mdlLoader.loadModelFromFile(urdfFilePath))
    {
        yError() << LogPrefix << "Failed to load URDF from:" << urdfFilePath;
        return false;
    }

    // ── KinDynComputations ────────────────────────────────────────────────────
    auto kinDyn = std::make_shared<iDynTree::KinDynComputations>();
    if (!kinDyn->loadRobotModel(mdlLoader.model()))
    {
        yError() << LogPrefix << "Failed to load robot model into KinDynComputations";
        return false;
    }
    if (!kinDyn->setFloatingBase(pImpl->baseName))
    {
        yError() << LogPrefix << "Failed to set floating base to:" << pImpl->baseName;
        return false;
    }

    // Set initial state to zero
    {
        const int nDofs = static_cast<int>(kinDyn->getNrOfDegreesOfFreedom());
        iDynTree::VectorDynSize zeroJoints(nDofs);
        zeroJoints.zero();
        iDynTree::Twist zeroTwist;
        zeroTwist.zero();
        iDynTree::Vector3 gravity;
        gravity.zero();
        gravity(2) = -9.81;
        kinDyn->setRobotState(iDynTree::Transform::Identity(), zeroJoints, zeroTwist, zeroJoints, gravity);
    }

    // ── Initialize estimator ──────────────────────────────────────────────────
    if (!pImpl->estimator.initialize(ikParams, kinDyn))
    {
        yError() << LogPrefix << "Failed to initialize BAFStateEstimator";
        return false;
    }
    if (!pImpl->estimator.setDt(period))
    {
        yError() << LogPrefix << "Failed to set estimator time step";
        return false;
    }

    // Cache joint names
    pImpl->jointNames = pImpl->estimator.getJointNames();
    pImpl->wearableTargets.clear();

    // ── JOINT_TO_SENSORS group (optional) ─────────────────────────────────────
    // Maps a model joint name to an IWear virtual joint sensor whose reading overrides,
    // a posteriori, the IK-solved position/velocity for that joint.
    pImpl->jointSensorTargets.clear();
    if (config.check("JOINT_TO_SENSORS"))
    {
        yarp::os::Bottle& group = config.findGroup("JOINT_TO_SENSORS");
        std::unordered_set<std::string> configuredJointNames;
        // group.get(0) is the group name; joint-sensor pairs start at index 1
        for (int i = 1; i < static_cast<int>(group.size()); ++i)
        {
            yarp::os::Bottle* entry = group.get(i).asList();
            if (!entry || entry->size() < 2)
            {
                yWarning() << LogPrefix << "JOINT_TO_SENSORS: skipping malformed entry at index" << i;
                continue;
            }

            const std::string jointName = entry->get(0).asString();
            const std::string sensorName = entry->get(1).asString();

            auto jointIt = std::find(pImpl->jointNames.begin(), pImpl->jointNames.end(), jointName);
            if (jointIt == pImpl->jointNames.end())
            {
                yWarning() << LogPrefix << "JOINT_TO_SENSORS: joint '" << jointName << "' not found in the model. Ignoring it.";
                continue;
            }
            if (!configuredJointNames.insert(jointName).second)
            {
                yError() << LogPrefix << "JOINT_TO_SENSORS: duplicate joint name '" << jointName << "'";
                return false;
            }

            JointSensorTarget t;
            t.sensorName = sensorName;
            t.jointName = jointName;
            t.jointIndex = static_cast<size_t>(std::distance(pImpl->jointNames.begin(), jointIt));
            pImpl->jointSensorTargets.push_back(std::move(t));
        }
    }

    // ── Initialize WearableSensorTarget objects for IWearableTargets ─────────────
    for (const auto& tgt : pImpl->sensorTargets)
    {
        hde::KinematicTargetType targetType;
        switch (tgt.kind)
        {
        case KinematicTaskKind::Position:
            targetType = hde::KinematicTargetType::position;
            break;
        case KinematicTaskKind::Pose:
            targetType = hde::KinematicTargetType::pose;
            break;
        case KinematicTaskKind::FloorContact:
            targetType = hde::KinematicTargetType::floorContact;
            break;
        default:
            targetType = hde::KinematicTargetType::orientationAndVelocity;
            break;
        }
        auto target = std::make_shared<hde::WearableSensorTarget>(tgt.sensorName, tgt.modelLinkName, targetType);
        pImpl->wearableTargets[tgt.taskName] = target;
    }

    // ── Allocate solution ─────────────────────────────────────────────────────
    {
        std::lock_guard<std::mutex> lock(pImpl->solutionMutex);
        const size_t nj = pImpl->estimator.getNumberOfJoints();
        pImpl->solution.jointPositions.assign(nj, 0.0);
        pImpl->solution.jointVelocities.assign(nj, 0.0);
    }

    // ── RPC port ──────────────────────────────────────────────────────────────
    std::string rpcPrefix = config.check("rpcPortPrefix", yarp::os::Value("")).asString();
    std::string rpcPortName = rpcPrefix + "/BAFStateProvider/rpc:i";
    if (!pImpl->rpcPort.open(rpcPortName))
    {
        yWarning() << LogPrefix << "Failed to open RPC port:" << rpcPortName;
        // Non-fatal: continue without RPC
    } else
    {
        yInfo() << LogPrefix << "RPC port opened at:" << rpcPortName;
        pImpl->rpcThreadRunning = true;
        pImpl->rpcThread = std::thread([this]() { pImpl->rpcLoop(); });
    }

    if (startAfterOpening)
    {
        if (!start())
        {
            yError() << LogPrefix << "Failed to start the periodic thread after open().";
            return false;
        }
        yInfo() << LogPrefix << "Periodic thread started after open() because startAfterOpening=true";
    }

    yInfo() << LogPrefix << "Device opened successfully";
    return true;
}

bool baf::devices::BAFStateProvider::close()
{
    if (isRunning())
        stop();

    pImpl->rpcThreadRunning = false;
    pImpl->rpcPort.interrupt();
    if (pImpl->rpcThread.joinable())
    {
        pImpl->rpcThread.join();
    }
    pImpl->rpcPort.close();
    return true;
}

// ─────────────────────────────────────────────────────────────────────────────
// IMultipleWrapper: attachAll / detachAll
// ─────────────────────────────────────────────────────────────────────────────

bool baf::devices::BAFStateProvider::attachAll(const yarp::dev::PolyDriverList& driverList)
{
    if (driverList.size() > 1)
    {
        yError() << LogPrefix << "This device accepts only one attached PolyDriver";
        return false;
    }

    const yarp::dev::PolyDriverDescriptor* driver = driverList[0];
    if (!driver)
    {
        yError() << LogPrefix << "Passed PolyDriverDescriptor is nullptr";
        return false;
    }

    if (!driver->poly->view(pImpl->iWear))
    {
        yError() << LogPrefix << "Failed to view the IWear interface from the PolyDriver";
        return false;
    }

    // Wait for IWear to be ready
    while (pImpl->iWear->getStatus() == wearable::WearStatus::WaitingForFirstRead)
    {
        yInfo() << LogPrefix << "IWear waiting for first data...";
        yarp::os::Time::delay(5.0);
    }
    if (pImpl->iWear->getStatus() != wearable::WearStatus::Ok)
    {
        yError() << LogPrefix << "IWear status is not Ok (" << static_cast<int>(pImpl->iWear->getStatus()) << ")";
        return false;
    }

    // ── Resolve sensors ───────────────────────────────────────────────────────
    for (auto& tgt : pImpl->sensorTargets)
    {
        if (tgt.kind == KinematicTaskKind::FloorContact)
        {
            auto ftSensor = pImpl->iWear->getForceTorque6DSensor(tgt.sensorName);
            if (!ftSensor)
            {
                yError() << LogPrefix << "FT sensor" << tgt.sensorName << "not found in IWear";
                return false;
            }
            tgt.sensor = ftSensor;
            yInfo() << LogPrefix << "Sensor" << tgt.sensorName << "resolved as ForceTorque6DSensor";
            continue;
        }

        // Try VirtualLinkKinSensor first (provides full kinematics for all non-FT tasks)
        auto vLinkSensor = pImpl->iWear->getVirtualLinkKinSensor(tgt.sensorName);
        if (vLinkSensor)
        {
            tgt.sensor = vLinkSensor;
            yInfo() << LogPrefix << "Sensor" << tgt.sensorName << "resolved as VirtualLinkKinSensor";
            continue;
        }

        // Position tasks require IVirtualLinkKinSensor
        if (tgt.kind == KinematicTaskKind::Position)
        {
            yError() << LogPrefix << "Sensor" << tgt.sensorName << "not found as VirtualLinkKinSensor (required for Position task)";
            return false;
        }

        // Pose tasks fall back to IPoseSensor
        if (tgt.kind == KinematicTaskKind::Pose)
        {
            auto poseSensor = pImpl->iWear->getPoseSensor(tgt.sensorName);
            if (poseSensor)
            {
                tgt.sensor = poseSensor;
                yInfo() << LogPrefix << "Sensor" << tgt.sensorName << "resolved as PoseSensor";
                continue;
            }
            yError() << LogPrefix << "Sensor" << tgt.sensorName
                     << "not found as VirtualLinkKinSensor or PoseSensor (required for Pose task)";
            return false;
        }

        // Orientation tasks fall back to IOrientationSensor (no angular velocity)
        auto orientSensor = pImpl->iWear->getOrientationSensor(tgt.sensorName);
        if (orientSensor)
        {
            tgt.sensor = orientSensor;
            yWarning() << LogPrefix << "Sensor" << tgt.sensorName << "resolved as OrientationSensor (no angular velocity)";
            continue;
        }

        // Orientation tasks also accept IPoseSensor: orientation is extracted from pose
        auto poseSensor = pImpl->iWear->getPoseSensor(tgt.sensorName);
        if (poseSensor)
        {
            tgt.sensor = poseSensor;
            yWarning() << LogPrefix << "Sensor" << tgt.sensorName << "resolved as PoseSensor for Orientation task (no angular velocity)";
            continue;
        }

        yError() << LogPrefix << "Sensor" << tgt.sensorName << "not found as VirtualLinkKinSensor, OrientationSensor or PoseSensor";
        return false;
    }

    // ── Resolve virtual joint sensors ─────────────────────────────────────────
    for (auto& tgt : pImpl->jointSensorTargets)
    {
        auto jointSensor = pImpl->iWear->getVirtualJointKinSensor(tgt.sensorName);
        if (!jointSensor)
        {
            yError() << LogPrefix << "Virtual joint sensor" << tgt.sensorName << "not found in IWear";
            return false;
        }
        tgt.sensor = jointSensor;
        yInfo() << LogPrefix << "Sensor" << tgt.sensorName << "resolved as VirtualJointKinSensor for joint" << tgt.jointName;
    }

    // ── Start the periodic loop (if not already started in open()) ───────────
    if (!isRunning())
    {
        if (!start())
        {
            yError() << LogPrefix << "Failed to start the periodic thread";
            return false;
        }
    }

    yInfo() << LogPrefix << "IWear interface attached successfully";
    return true;
}

bool baf::devices::BAFStateProvider::detachAll()
{
    if (isRunning())
        stop();
    pImpl->iWear = nullptr;
    return true;
}

// ─────────────────────────────────────────────────────────────────────────────
// PeriodicThread: run / threadRelease
// ─────────────────────────────────────────────────────────────────────────────

void baf::devices::BAFStateProvider::run()
{
    // ── Step 1: read sensors and build estimator input maps ───────────────────
    pImpl->nodeDataMap.clear();
    pImpl->positionDataMap.clear();
    pImpl->poseDataMap.clear();
    pImpl->ftDataMap.clear();

    for (const auto& tgt : pImpl->sensorTargets)
    {
        if (tgt.kind == KinematicTaskKind::Orientation)
        {
            BiomechanicalAnalysis::IK::nodeData nd;
            bool valid = false;

            if (auto* s = std::get_if<wearable::SensorPtr<const IVirtualLinkKinSensor>>(&tgt.sensor))
            {
                if (!*s || (*s)->getSensorStatus() != SensorStatus::Ok)
                    continue;
                wearable::Quaternion orientation;
                wearable::Vector3 angularVelocity;
                if (!(*s)->getLinkOrientation(orientation) || !(*s)->getLinkAngularVelocity(angularVelocity))
                {
                    yWarning() << LogPrefix << "Failed to read from sensor" << tgt.sensorName;
                    continue;
                }
                nd.I_R_IMU = impl::quaternionToSO3(orientation);
                nd.I_omega_IMU = impl::vector3ToSO3Tangent(angularVelocity);
                valid = true;
            } else if (auto* s = std::get_if<wearable::SensorPtr<const IOrientationSensor>>(&tgt.sensor))
            {
                if (!*s || (*s)->getSensorStatus() != SensorStatus::Ok)
                    continue;
                wearable::Quaternion orientation;
                if (!(*s)->getOrientationAsQuaternion(orientation))
                {
                    yWarning() << LogPrefix << "Failed to read from sensor" << tgt.sensorName;
                    continue;
                }
                nd.I_R_IMU = impl::quaternionToSO3(orientation);
                // I_omega_IMU is zero-initialized by nodeData default
                valid = true;
            } else if (auto* s = std::get_if<wearable::SensorPtr<const IPoseSensor>>(&tgt.sensor))
            {
                if (!*s || (*s)->getSensorStatus() != SensorStatus::Ok)
                {
                    yWarning() << LogPrefix << "Sensor" << tgt.sensorName << "not Ok";
                    continue;
                }
                wearable::Quaternion orientation;
                wearable::Vector3 position;
                // IPoseSensor::getPose(orientation, position)
                if (!(*s)->getPose(orientation, position))
                {
                    yWarning() << LogPrefix << "Failed to read pose from sensor" << tgt.sensorName;
                    continue;
                }
                nd.I_R_IMU = impl::quaternionToSO3(orientation);
                // I_omega_IMU is zero-initialized by nodeData default
                valid = true;
            }

            if (!valid)
            {
                yWarning() << LogPrefix << "Failed to read orientation data from sensor" << tgt.sensorName;
                continue;
            }

            pImpl->nodeDataMap[tgt.taskName] = nd;
        } else if (tgt.kind == KinematicTaskKind::Position)
        {
            auto* s = std::get_if<wearable::SensorPtr<const IVirtualLinkKinSensor>>(&tgt.sensor);
            if (!s || !*s || (*s)->getSensorStatus() != SensorStatus::Ok)
                continue;

            wearable::Vector3 position;
            if (!(*s)->getLinkPosition(position))
            {
                yWarning() << LogPrefix << "Failed to read position from sensor" << tgt.sensorName;
                continue;
            }
            BiomechanicalAnalysis::IK::positionData pd;
            pd.S_p_M = Eigen::Vector3d(position[0], position[1], position[2]);
            pImpl->positionDataMap[tgt.taskName] = pd;
        } else if (tgt.kind == KinematicTaskKind::Pose)
        {
            wearable::Vector3 position;
            wearable::Quaternion orientation;
            bool ok = false;

            if (auto* s = std::get_if<wearable::SensorPtr<const IVirtualLinkKinSensor>>(&tgt.sensor))
            {
                if (!*s || (*s)->getSensorStatus() != SensorStatus::Ok)
                    continue;
                ok = (*s)->getLinkPose(position, orientation);
            } else if (auto* s = std::get_if<wearable::SensorPtr<const IPoseSensor>>(&tgt.sensor))
            {
                if (!*s || (*s)->getSensorStatus() != SensorStatus::Ok)
                    continue;
                // IPoseSensor::getPose(orientation, position): orientation first, then position
                ok = (*s)->getPose(orientation, position);
            } else
                continue;

            if (!ok)
            {
                yWarning() << LogPrefix << "Failed to read pose from sensor" << tgt.sensorName;
                continue;
            }

            Eigen::Vector3d pos(position[0], position[1], position[2]);
            // wearable::Quaternion = [w, x, y, z]
            BiomechanicalAnalysis::IK::poseData pd;
            pd.S_H_M = manif::SE3d(pos, Eigen::Quaterniond(orientation[0], orientation[1], orientation[2], orientation[3]));
            pImpl->poseDataMap[tgt.taskName] = pd;
        } else if (tgt.kind == KinematicTaskKind::FloorContact)
        {
            auto* s = std::get_if<wearable::SensorPtr<const IForceTorque6DSensor>>(&tgt.sensor);
            if (!s || !*s || (*s)->getSensorStatus() != SensorStatus::Ok)
                continue;

            wearable::Vector6 ft;
            if (!(*s)->getForceTorque6D(ft))
            {
                yWarning() << LogPrefix << "Failed to read FT sensor" << tgt.sensorName;
                continue;
            }
            Eigen::Matrix<double, 6, 1> wrench;
            wrench << ft[0], ft[1], ft[2], ft[3], ft[4], ft[5];
            pImpl->ftDataMap[tgt.taskName] = wrench;
        }
    }

    // Consume pending RPC commands
    std::deque<impl::PendingRpcCommand> pendingCommands;
    {
        std::lock_guard<std::mutex> lock(pImpl->rpcCommandsMutex);
        pendingCommands.swap(pImpl->pendingRpcCommands);
    }

    for (const auto& cmd : pendingCommands)
    {
        if (cmd.type == impl::RpcCommandType::CalibrateAll)
        {
            auto calibrationDataMap = pImpl->nodeDataMap;
            for (const auto& [taskName, pose] : pImpl->poseDataMap)
            {
                BiomechanicalAnalysis::IK::nodeData nd;
                nd.I_R_IMU = manif::SO3d(Eigen::Quaterniond(pose.S_H_M.rotation()));
                calibrationDataMap.emplace(taskName, nd);
            }

            pImpl->estimator.calibrate(calibrationDataMap, cmd.refFrame);
        } else if (cmd.type == impl::RpcCommandType::ResetAll)
        {
            pImpl->estimator.clearCalibration();
        } else if (cmd.type == impl::RpcCommandType::ResetWorldAnchorTranslation)
        {
            if (!pImpl->estimator.resetWorldAnchorTranslation())
            {
                yWarning() << LogPrefix << "resetWorldAnchorTranslation() failed";
            }
        } else if (cmd.type == impl::RpcCommandType::RecenterWorldAnchor)
        {
            if (!pImpl->estimator.recenterWorldAnchor())
            {
                yWarning() << LogPrefix << "recenterWorldAnchor() failed";
            }
        } else if (cmd.type == impl::RpcCommandType::ResetJointState)
        {
            if (!pImpl->estimator.resetJointState())
            {
                yWarning() << LogPrefix << "resetJointState() failed";
            }
        }
    }

    // ── Step 3: feed estimator and advance ────────────────────────────────────
    pImpl->estimator.updateInput(pImpl->nodeDataMap, pImpl->ftDataMap, pImpl->positionDataMap, pImpl->poseDataMap);
    if (!pImpl->estimator.advance())
    {
        yWarning() << LogPrefix << "BAFStateEstimator::advance() failed";
        return;
    }

    // ── Step 4: update wearable targets with actual solver setpoints ───────────
    for (const auto& tgt : pImpl->sensorTargets)
    {
        auto it = pImpl->wearableTargets.find(tgt.taskName);
        if (it == pImpl->wearableTargets.end())
            continue;
        auto& wTarget = it->second;

        if (tgt.kind == KinematicTaskKind::Orientation)
        {
            Eigen::Matrix3d rotation;
            Eigen::Vector3d angularVelocity;
            if (pImpl->estimator.getOrientationTaskSetPoint(tgt.taskName, rotation, angularVelocity))
            {
                std::lock_guard<std::mutex> lock(wTarget->mutex);
                iDynTree::toEigen(wTarget->rotation) = rotation;
                iDynTree::toEigen(wTarget->angularVelocity) = angularVelocity;
            }
        } else if (tgt.kind == KinematicTaskKind::Position)
        {
            Eigen::Vector3d position;
            Eigen::Vector3d linearVelocity;
            if (pImpl->estimator.getPositionTaskSetPoint(tgt.taskName, position, linearVelocity))
            {
                std::lock_guard<std::mutex> lock(wTarget->mutex);
                iDynTree::toEigen(wTarget->position) = position;
                iDynTree::toEigen(wTarget->linearVelocity) = linearVelocity;
            }
        } else if (tgt.kind == KinematicTaskKind::Pose)
        {
            Eigen::Vector3d position;
            Eigen::Matrix3d rotation;
            Eigen::Matrix<double, 6, 1> velocity;
            if (pImpl->estimator.getPoseTaskSetPoint(tgt.taskName, position, rotation, velocity))
            {
                std::lock_guard<std::mutex> lock(wTarget->mutex);
                iDynTree::toEigen(wTarget->position) = position;
                iDynTree::toEigen(wTarget->rotation) = rotation;
                iDynTree::toEigen(wTarget->linearVelocity) = velocity.head<3>();
                iDynTree::toEigen(wTarget->angularVelocity) = velocity.tail<3>();
            }
        }
        // FloorContact: no solver setpoint to expose
    }

    // ── Step 5: copy results into solution (mutex-protected) ──────────────────
    const auto& jp = pImpl->estimator.getJointPositions();
    const auto& jv = pImpl->estimator.getJointVelocities();
    const auto& bp = pImpl->estimator.getBasePosition();
    const auto bR = pImpl->estimator.getBaseOrientation();
    const auto& blv = pImpl->estimator.getBaseLinearVelocity();
    const auto& bav = pImpl->estimator.getBaseAngularVelocity();
    const auto comP = pImpl->estimator.getCoMPosition();
    const auto comV = pImpl->estimator.getCoMVelocity();

    // Base orientation as quaternion [w, x, y, z] (iDynTree convention)
    iDynTree::Rotation rotIdyn;
    iDynTree::toEigen(rotIdyn) = bR;
    auto quat = rotIdyn.asQuaternion();

    {
        std::lock_guard<std::mutex> lock(pImpl->solutionMutex);

        const size_t nj = pImpl->solution.jointPositions.size();
        for (size_t i = 0; i < nj; ++i)
        {
            pImpl->solution.jointPositions[i] = jp(i);
            pImpl->solution.jointVelocities[i] = jv(i);
        }

        pImpl->solution.basePosition = {bp(0), bp(1), bp(2)};
        // iDynTree asQuaternion() returns [w, x, y, z]
        pImpl->solution.baseOrientation = {quat(0), quat(1), quat(2), quat(3)};
        pImpl->solution.baseVelocity = {blv(0), blv(1), blv(2), bav(0), bav(1), bav(2)};
        pImpl->solution.comPosition = {comP(0), comP(1), comP(2)};
        pImpl->solution.comVelocity = {comV(0), comV(1), comV(2)};

        // ── Step 6: override mapped joints with virtual joint sensor readings ──
        for (const auto& tgt : pImpl->jointSensorTargets)
        {
            if (tgt.sensor->getSensorStatus() != SensorStatus::Ok)
            {
                yWarningThrottle(1.0) << LogPrefix << "Virtual joint sensor" << tgt.sensorName << "not Ok, skipping joint" << tgt.jointName;
                continue;
            }

            double position = 0.0;
            double velocity = 0.0;
            const bool okPosition = tgt.sensor->getJointPosition(position);
            const bool okVelocity = tgt.sensor->getJointVelocity(velocity);
            if (!okPosition || !okVelocity)
            {
                yWarning() << LogPrefix << "Failed to read from virtual joint sensor" << tgt.sensorName;
                continue;
            }

            pImpl->solution.jointPositions[tgt.jointIndex] = position;
            pImpl->solution.jointVelocities[tgt.jointIndex] = velocity;
        }
    }
}

void baf::devices::BAFStateProvider::threadRelease()
{
    // Nothing to release for this device
}

// ─────────────────────────────────────────────────────────────────────────────
// IHumanState
// ─────────────────────────────────────────────────────────────────────────────

std::vector<std::string> baf::devices::BAFStateProvider::getJointNames() const
{
    return pImpl->jointNames;
}

size_t baf::devices::BAFStateProvider::getNumberOfJoints() const
{
    return pImpl->jointNames.size();
}

std::string baf::devices::BAFStateProvider::getBaseName() const
{
    return pImpl->baseName;
}

std::vector<double> baf::devices::BAFStateProvider::getJointPositions() const
{
    std::lock_guard<std::mutex> lock(pImpl->solutionMutex);
    return pImpl->solution.jointPositions;
}

std::vector<double> baf::devices::BAFStateProvider::getJointVelocities() const
{
    std::lock_guard<std::mutex> lock(pImpl->solutionMutex);
    return pImpl->solution.jointVelocities;
}

std::array<double, 3> baf::devices::BAFStateProvider::getBasePosition() const
{
    std::lock_guard<std::mutex> lock(pImpl->solutionMutex);
    return pImpl->solution.basePosition;
}

std::array<double, 4> baf::devices::BAFStateProvider::getBaseOrientation() const
{
    std::lock_guard<std::mutex> lock(pImpl->solutionMutex);
    return pImpl->solution.baseOrientation;
}

std::array<double, 6> baf::devices::BAFStateProvider::getBaseVelocity() const
{
    std::lock_guard<std::mutex> lock(pImpl->solutionMutex);
    return pImpl->solution.baseVelocity;
}

std::array<double, 3> baf::devices::BAFStateProvider::getCoMPosition() const
{
    std::lock_guard<std::mutex> lock(pImpl->solutionMutex);
    return pImpl->solution.comPosition;
}

std::array<double, 3> baf::devices::BAFStateProvider::getCoMVelocity() const
{
    std::lock_guard<std::mutex> lock(pImpl->solutionMutex);
    return pImpl->solution.comVelocity;
}

// ─────────────────────────────────────────────────────────────────────────────
// IWearableTargets
// ─────────────────────────────────────────────────────────────────────────────

std::vector<hde::TargetName> baf::devices::BAFStateProvider::getAllTargetsName() const
{
    std::vector<hde::TargetName> names;
    names.reserve(pImpl->wearableTargets.size());
    for (const auto& [name, _] : pImpl->wearableTargets)
        names.push_back(name);
    return names;
}

std::shared_ptr<hde::WearableSensorTarget> baf::devices::BAFStateProvider::getTarget(const hde::TargetName name) const
{
    auto it = pImpl->wearableTargets.find(name);
    if (it == pImpl->wearableTargets.end())
        return nullptr;
    return it->second;
}
