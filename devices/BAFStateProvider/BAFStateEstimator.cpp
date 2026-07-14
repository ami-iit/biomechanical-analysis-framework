// SPDX-FileCopyrightText: Fondazione Istituto Italiano di Tecnologia (IIT)
// SPDX-License-Identifier: BSD-3-Clause

#include "BAFStateEstimator.h"

#include <BiomechanicalAnalysis/Logging/Logger.h>

#include <iDynTree/EigenHelpers.h>
#include <iDynTree/KinDynComputations.h>
#include <iDynTree/Twist.h>
#include <iDynTree/Model.h>

#include <stdexcept>
#include <vector>

// ─────────────────────────────────────────────────────────────────────────────
// Private implementation
// ─────────────────────────────────────────────────────────────────────────────

class BAFStateEstimator::Impl
{
public:
    enum class TaskKind
    {
        Orientation,
        Gravity,
        FloorContact,
        Position,
        Pose
    };

    BiomechanicalAnalysis::IK::HumanIK humanIK;
    std::shared_ptr<iDynTree::KinDynComputations> kinDyn;

    // IK task name -> BAF integer key
    std::unordered_map<std::string, int> taskNameToTaskID;
    std::unordered_map<std::string, TaskKind> taskNameToTaskKind;

    // last input
    std::unordered_map<std::string, BiomechanicalAnalysis::IK::nodeData> nodeData;
    std::unordered_map<std::string, Eigen::Matrix<double, 6, 1>> ftData;
    std::unordered_map<std::string, BiomechanicalAnalysis::IK::positionData> positionData;
    std::unordered_map<std::string, BiomechanicalAnalysis::IK::poseData> poseData;

    // IK output
    Eigen::VectorXd jointPositions;
    Eigen::VectorXd jointVelocities;
    Eigen::Matrix3d baseOrientation;
    Eigen::Vector3d basePosition;
    Eigen::Vector3d baseLinearVelocity;
    Eigen::Vector3d baseAngularVelocity;
    Eigen::VectorXd jointPositionSetPoint;

    // Gravity for KinDyn updates
    Eigen::Vector3d gravity{0.0, 0.0, -9.81};

    // Calibration state
    bool pendingCalibration{false};
    std::unordered_map<std::string, BiomechanicalAnalysis::IK::nodeData> pendingCalibrationData;
    std::string pendingCalibrationReferenceFrame;

    std::vector<std::string> jointNames;
    bool isInitialized{false};

    // ── helpers ───────────────────────────────────────────────────────────

    /** Translate a task-keyed nodeData map to the integer-keyed map required by HumanIK. */
    std::unordered_map<int, BiomechanicalAnalysis::IK::nodeData>
    toIntNodeMap(const std::unordered_map<std::string, BiomechanicalAnalysis::IK::nodeData>& in) const
    {
        std::unordered_map<int, BiomechanicalAnalysis::IK::nodeData> out;
        for (const auto& [taskName, data] : in)
        {
            auto kindIt = taskNameToTaskKind.find(taskName);
            if (kindIt == taskNameToTaskKind.end())
            {
                continue;
            }

            if (kindIt->second != TaskKind::Orientation && kindIt->second != TaskKind::Gravity)
            {
                continue;
            }

            auto idIt = taskNameToTaskID.find(taskName);
            if (idIt != taskNameToTaskID.end())
            {
                out[idIt->second] = data;
            }
        }
        return out;
    }

    /**
     * Translate a string-keyed nodeData map to integer keys used during calibration.
     * Calibration must also cover PoseTask nodes, not only SO3/Gravity nodes.
     */
    std::unordered_map<int, BiomechanicalAnalysis::IK::nodeData>
    toIntCalibrationNodeMap(const std::unordered_map<std::string, BiomechanicalAnalysis::IK::nodeData>& in) const
    {
        std::unordered_map<int, BiomechanicalAnalysis::IK::nodeData> out;
        for (const auto& [taskName, data] : in)
        {
            auto kindIt = taskNameToTaskKind.find(taskName);
            if (kindIt == taskNameToTaskKind.end())
            {
                continue;
            }

            const bool calibrationSupported = (kindIt->second == TaskKind::Orientation || kindIt->second == TaskKind::Gravity
                                               || kindIt->second == TaskKind::Pose || kindIt->second == TaskKind::Position);
            if (!calibrationSupported)
            {
                continue;
            }

            auto idIt = taskNameToTaskID.find(taskName);
            if (idIt != taskNameToTaskID.end())
            {
                out[idIt->second] = data;
            }
        }
        return out;
    }

    /** Translate a task-keyed FT map to the integer-keyed map required by HumanIK. */
    std::unordered_map<int, Eigen::Matrix<double, 6, 1>>
    toIntFTMap(const std::unordered_map<std::string, Eigen::Matrix<double, 6, 1>>& in) const
    {
        std::unordered_map<int, Eigen::Matrix<double, 6, 1>> out;
        for (const auto& [taskName, wrench] : in)
        {
            auto kindIt = taskNameToTaskKind.find(taskName);
            if (kindIt == taskNameToTaskKind.end() || kindIt->second != TaskKind::FloorContact)
            {
                continue;
            }

            auto idIt = taskNameToTaskID.find(taskName);
            if (idIt != taskNameToTaskID.end())
            {
                out[idIt->second] = wrench;
            }
        }
        return out;
    }

    /** Translate a task-keyed position map to the integer-keyed map required by HumanIK. */
    std::unordered_map<int, BiomechanicalAnalysis::IK::positionData>
    toIntPositionMap(const std::unordered_map<std::string, BiomechanicalAnalysis::IK::positionData>& in) const
    {
        std::unordered_map<int, BiomechanicalAnalysis::IK::positionData> out;
        for (const auto& [taskName, data] : in)
        {
            auto kindIt = taskNameToTaskKind.find(taskName);
            if (kindIt == taskNameToTaskKind.end() || kindIt->second != TaskKind::Position)
            {
                continue;
            }

            auto idIt = taskNameToTaskID.find(taskName);
            if (idIt != taskNameToTaskID.end())
            {
                out[idIt->second] = data;
            }
        }
        return out;
    }

    /** Translate a task-keyed pose map to the integer-keyed map required by HumanIK. */
    std::unordered_map<int, BiomechanicalAnalysis::IK::poseData>
    toIntPoseMap(const std::unordered_map<std::string, BiomechanicalAnalysis::IK::poseData>& in) const
    {
        std::unordered_map<int, BiomechanicalAnalysis::IK::poseData> out;
        for (const auto& [taskName, data] : in)
        {
            auto kindIt = taskNameToTaskKind.find(taskName);
            if (kindIt == taskNameToTaskKind.end() || kindIt->second != TaskKind::Pose)
            {
                continue;
            }

            auto idIt = taskNameToTaskID.find(taskName);
            if (idIt != taskNameToTaskID.end())
            {
                out[idIt->second] = data;
            }
        }
        return out;
    }

    /**
     * Parse the IK configuration and map each task name to its HumanIK id and kind.
     * Must be called after humanIK.initialize().
     */
    bool buildLookupTables(
        std::shared_ptr<BipedalLocomotion::ParametersHandler::IParametersHandler> handler)
    {
        std::vector<std::string> tasks;
        if (!handler->getParameter("tasks", tasks))
        {
            BiomechanicalAnalysis::log()->error("[BAFStateEstimator] Cannot read 'tasks' list from IK config");
            return false;
        }

        taskNameToTaskID.clear();
        taskNameToTaskKind.clear();

        int orientationAndGravityCount = 0;

        for (const auto& taskName : tasks)
        {
            auto taskGroupWeak = handler->getGroup(taskName);
            auto taskGroup = taskGroupWeak.lock();
            if (!taskGroup)
            {
                BiomechanicalAnalysis::log()->warn("[BAFStateEstimator] Cannot read task group '{}'", taskName);
                continue;
            }

            std::string type;
            if (!taskGroup->getParameter("type", type))
                continue;

            if (type == "SO3Task")
            {
                int nodeNumber;
                if (taskGroup->getParameter("node_number", nodeNumber))
                {
                    taskNameToTaskID[taskName] = nodeNumber;
                    taskNameToTaskKind[taskName] = TaskKind::Orientation;
                    ++orientationAndGravityCount;
                }
            }
            else if (type == "GravityTask")
            {
                int nodeNumber;
                if (taskGroup->getParameter("node_number", nodeNumber))
                {
                    taskNameToTaskID[taskName] = nodeNumber;
                    taskNameToTaskKind[taskName] = TaskKind::Gravity;
                    ++orientationAndGravityCount;
                }
            }
            else if (type == "FloorContactTask")
            {
                int taskNumber = -1;
                // Preferred key is "floor_contact_task"; "node_number" is deprecated
                bool hasKey = taskGroup->getParameter("floor_contact_task", taskNumber);
                if (!hasKey)
                    hasKey = taskGroup->getParameter("node_number", taskNumber);

                if (hasKey)
                {
                    taskNameToTaskID[taskName] = taskNumber;
                    taskNameToTaskKind[taskName] = TaskKind::FloorContact;
                }
            }
            else if (type == "PositionTask")
            {
                int nodeNumber;
                if (taskGroup->getParameter("node_number", nodeNumber))
                {
                    taskNameToTaskID[taskName] = nodeNumber;
                    taskNameToTaskKind[taskName] = TaskKind::Position;
                }
            }
            else if (type == "PoseTask")
            {
                int nodeNumber;
                if (taskGroup->getParameter("node_number", nodeNumber))
                {
                    taskNameToTaskID[taskName] = nodeNumber;
                    taskNameToTaskKind[taskName] = TaskKind::Pose;
                }
            }
            // Other task types (JOINT_*, BASE_*) do not map to sensor nodes
        }

        if (orientationAndGravityCount == 0)
        {
            BiomechanicalAnalysis::log()->warn("[BAFStateEstimator] No orientation/gravity tasks found in IK config");
        }

        return true;
    }
};

// ─────────────────────────────────────────────────────────────────────────────
// Public interface
// ─────────────────────────────────────────────────────────────────────────────

BAFStateEstimator::BAFStateEstimator()
    : m_impl(std::make_unique<Impl>())
{
}

BAFStateEstimator::~BAFStateEstimator() = default;

bool BAFStateEstimator::initialize(
    std::shared_ptr<BipedalLocomotion::ParametersHandler::IParametersHandler> ikParams,
    std::shared_ptr<iDynTree::KinDynComputations> kinDyn)
{
    if (!ikParams)
    {
        BiomechanicalAnalysis::log()->error("[BAFStateEstimator] ikParams is null");
        return false;
    }
    if (!kinDyn)
    {
        BiomechanicalAnalysis::log()->error("[BAFStateEstimator] kinDyn is null");
        return false;
    }

    m_impl->kinDyn = kinDyn;

    // Initialize the IK solver
    if (!m_impl->humanIK.initialize(ikParams, kinDyn))
    {
        BiomechanicalAnalysis::log()->error("[BAFStateEstimator] HumanIK initialization failed");
        return false;
    }

    // Build taskName -> integer key maps from the same config
    if (!m_impl->buildLookupTables(ikParams))
    {
        return false;
    }

    // Allocate state buffers
    const int nDofs = m_impl->humanIK.getDoFsNumber();
    m_impl->jointPositions.setZero(nDofs);
    m_impl->jointVelocities.setZero(nDofs);
    m_impl->baseOrientation.setIdentity();
    m_impl->basePosition.setZero();
    m_impl->baseLinearVelocity.setZero();
    m_impl->baseAngularVelocity.setZero();
    m_impl->jointPositionSetPoint = m_impl->humanIK.getJointPositionSetPoint();

    // Collect joint names from the model
    const auto& model = kinDyn->model();
    m_impl->jointNames.clear();
    m_impl->jointNames.reserve(nDofs);
    for (int i = 0; i < model.getNrOfJoints(); ++i)
    {
        const auto* joint = model.getJoint(i);
        if (joint && joint->getNrOfDOFs() == 1)
        {
            m_impl->jointNames.push_back(model.getJointName(i));
        }
    }

    m_impl->isInitialized = true;
    return true;
}

bool BAFStateEstimator::setDt(double dt)
{
    if (!m_impl->isInitialized)
    {
        BiomechanicalAnalysis::log()->error("[BAFStateEstimator] setDt() called before initialize()");
        return false;
    }
    return m_impl->humanIK.setDt(dt);
}

void BAFStateEstimator::updateInput(
    const std::unordered_map<std::string, BiomechanicalAnalysis::IK::nodeData>& nodeData,
    const std::unordered_map<std::string, Eigen::Matrix<double, 6, 1>>& ftData,
    const std::unordered_map<std::string, BiomechanicalAnalysis::IK::positionData>& positionData,
    const std::unordered_map<std::string, BiomechanicalAnalysis::IK::poseData>& poseData)
{
    m_impl->nodeData = nodeData;
    m_impl->ftData = ftData;
    m_impl->positionData = positionData;
    m_impl->poseData = poseData;
}

bool BAFStateEstimator::advance()
{
    if (!m_impl->isInitialized)
    {
        BiomechanicalAnalysis::log()->error("[BAFStateEstimator] advance() called before initialize()");
        return false;
    }

    // ── Step 0: apply pending calibration ────────────────────────────────────
    if (m_impl->pendingCalibration)
    {
        auto intMap = m_impl->toIntCalibrationNodeMap(m_impl->pendingCalibrationData);
        m_impl->humanIK.clearCalibrationMatrices();
        m_impl->humanIK.calibrateWorldYaw(intMap);
        m_impl->humanIK.calibrateAllWithWorld(intMap, m_impl->pendingCalibrationReferenceFrame);
        m_impl->pendingCalibration = false;
        BiomechanicalAnalysis::log()->info("[BAFStateEstimator] T-pose calibration applied");
    }

    // ── Step 1: update orientation and gravity tasks ──────────────────────────
    auto intNodeMap = m_impl->toIntNodeMap(m_impl->nodeData);
    if (!m_impl->humanIK.updateOrientationAndGravityTasks(intNodeMap))
    {
        BiomechanicalAnalysis::log()->error("[BAFStateEstimator] updateOrientationAndGravityTasks failed");
        return false;
    }

    // ── Step 2: update floor contact tasks (if FT data provided) ─────────────
    if (!m_impl->ftData.empty())
    {
        auto intFTMap = m_impl->toIntFTMap(m_impl->ftData);
        if (!intFTMap.empty())
        {
            if (!m_impl->humanIK.updateFloorContactTasks(intFTMap))
            {
                BiomechanicalAnalysis::log()->error("[BAFStateEstimator] updateFloorContactTasks failed");
                return false;
            }
        }
    }

    // ── Step 2b: update position tasks (if position data provided) ───────────
    if (!m_impl->positionData.empty())
    {
        auto intPositionMap = m_impl->toIntPositionMap(m_impl->positionData);
        if (!intPositionMap.empty())
        {
            if (!m_impl->humanIK.updatePositionTasks(intPositionMap))
            {
                BiomechanicalAnalysis::log()->error("[BAFStateEstimator] updatePositionTasks failed");
                return false;
            }
        }
    }

    // ── Step 2c: update pose tasks (if pose data provided) ───────────────────
    if (!m_impl->poseData.empty())
    {
        auto intPoseMap = m_impl->toIntPoseMap(m_impl->poseData);
        if (!intPoseMap.empty())
        {
            if (!m_impl->humanIK.updatePoseTasks(intPoseMap))
            {
                BiomechanicalAnalysis::log()->error("[BAFStateEstimator] updatePoseTasks failed");
                return false;
            }
        }
    }

    // ── Step 3: update regularization and constraint tasks ───────────────────
    if (!m_impl->humanIK.updateJointConstraintsTask())
    {
        BiomechanicalAnalysis::log()->error("[BAFStateEstimator] updateJointConstraintsTask failed");
        return false;
    }
    if (!m_impl->humanIK.updateJointRegularizationTask(m_impl->jointPositionSetPoint))
    {
        BiomechanicalAnalysis::log()->error("[BAFStateEstimator] updateJointRegularizationTask failed");
        return false;
    }

    // ── Step 4: solve ─────────────────────────────────────────────────────────
    if (!m_impl->humanIK.advance())
    {
        BiomechanicalAnalysis::log()->error("[BAFStateEstimator] HumanIK advance failed");
        return false;
    }

    // ── Step 5: read back results ─────────────────────────────────────────────
    m_impl->humanIK.getJointPositions(m_impl->jointPositions);
    m_impl->humanIK.getJointVelocities(m_impl->jointVelocities);
    m_impl->humanIK.getBaseOrientation(m_impl->baseOrientation);
    m_impl->humanIK.getBasePosition(m_impl->basePosition);
    m_impl->humanIK.getBaseLinearVelocity(m_impl->baseLinearVelocity);
    m_impl->humanIK.getBaseAngularVelocity(m_impl->baseAngularVelocity);

    // ── Step 6: update KinDyn state for CoM computation ──────────────────────
    // Build base pose and velocity in iDynTree format
    iDynTree::Rotation w_R_b;
    iDynTree::toEigen(w_R_b) = m_impl->baseOrientation;
    iDynTree::Position w_p_b;
    iDynTree::toEigen(w_p_b) = m_impl->basePosition;
    iDynTree::Transform w_H_b(w_R_b, w_p_b);

    iDynTree::Twist baseVelocity;
    iDynTree::toEigen(baseVelocity.getLinearVec3()) = m_impl->baseLinearVelocity;
    iDynTree::toEigen(baseVelocity.getAngularVec3()) = m_impl->baseAngularVelocity;

    iDynTree::VectorDynSize jointPos(m_impl->jointPositions.size());
    iDynTree::toEigen(jointPos) = m_impl->jointPositions;

    iDynTree::VectorDynSize jointVel(m_impl->jointVelocities.size());
    iDynTree::toEigen(jointVel) = m_impl->jointVelocities;

    iDynTree::Vector3 gravity;
    iDynTree::toEigen(gravity) = m_impl->gravity;

    m_impl->kinDyn->setRobotState(w_H_b, jointPos, baseVelocity, jointVel, gravity);

    return true;
}

void BAFStateEstimator::calibrate(
    const std::unordered_map<std::string, BiomechanicalAnalysis::IK::nodeData>& nodeData,
    const std::string& referenceFrame)
{
    m_impl->pendingCalibrationData = nodeData;
    m_impl->pendingCalibrationReferenceFrame = referenceFrame;
    m_impl->pendingCalibration = true;
}

bool BAFStateEstimator::clearCalibration()
{
    if (!m_impl->isInitialized)
        return false;
    m_impl->pendingCalibration = false;
    return m_impl->humanIK.clearCalibrationMatrices();
}

bool BAFStateEstimator::resetWorldAnchorTranslation()
{
    if (!m_impl->isInitialized)
        return false;
    return m_impl->humanIK.resetWorldAnchorTranslation();
}

bool BAFStateEstimator::recenterWorldAnchor()
{
    if (!m_impl->isInitialized)
        return false;
    return m_impl->humanIK.recenterWorldAnchor();
}

bool BAFStateEstimator::resetJointState()
{
    if (!m_impl->isInitialized)
        return false;
    return m_impl->humanIK.resetJointState();
}

// ── Getters ───────────────────────────────────────────────────────────────────

const Eigen::VectorXd& BAFStateEstimator::getJointPositions() const
{
    return m_impl->jointPositions;
}

const Eigen::VectorXd& BAFStateEstimator::getJointVelocities() const
{
    return m_impl->jointVelocities;
}

Eigen::Matrix3d BAFStateEstimator::getBaseOrientation() const
{
    return m_impl->baseOrientation;
}

const Eigen::Vector3d& BAFStateEstimator::getBasePosition() const
{
    return m_impl->basePosition;
}

const Eigen::Vector3d& BAFStateEstimator::getBaseLinearVelocity() const
{
    return m_impl->baseLinearVelocity;
}

const Eigen::Vector3d& BAFStateEstimator::getBaseAngularVelocity() const
{
    return m_impl->baseAngularVelocity;
}

iDynTree::Position BAFStateEstimator::getCoMPosition() const
{
    return m_impl->kinDyn->getCenterOfMassPosition();
}

iDynTree::Vector3 BAFStateEstimator::getCoMVelocity() const
{
    return m_impl->kinDyn->getCenterOfMassVelocity();
}

size_t BAFStateEstimator::getNumberOfJoints() const
{
    return static_cast<size_t>(m_impl->humanIK.getDoFsNumber());
}

const std::vector<std::string>& BAFStateEstimator::getJointNames() const
{
    return m_impl->jointNames;
}

bool BAFStateEstimator::getOrientationTaskSetPoint(const std::string& taskName,
                                                    Eigen::Matrix3d& rotation,
                                                    Eigen::Vector3d& angularVelocity) const
{
    auto kindIt = m_impl->taskNameToTaskKind.find(taskName);
    if (kindIt == m_impl->taskNameToTaskKind.end())
        return false;
    if (kindIt->second != Impl::TaskKind::Orientation && kindIt->second != Impl::TaskKind::Gravity)
        return false;

    auto idIt = m_impl->taskNameToTaskID.find(taskName);
    if (idIt == m_impl->taskNameToTaskID.end())
        return false;

    manif::SO3d W_R_link;
    Eigen::Vector3d W_omega_link;
    if (!m_impl->humanIK.getOrientationTaskSetPoint(idIt->second, W_R_link, W_omega_link))
        return false;

    rotation = W_R_link.rotation();
    angularVelocity = W_omega_link;
    return true;
}

bool BAFStateEstimator::getPositionTaskSetPoint(const std::string& taskName,
                                                 Eigen::Vector3d& position,
                                                 Eigen::Vector3d& linearVelocity) const
{
    auto kindIt = m_impl->taskNameToTaskKind.find(taskName);
    if (kindIt == m_impl->taskNameToTaskKind.end() || kindIt->second != Impl::TaskKind::Position)
        return false;

    auto idIt = m_impl->taskNameToTaskID.find(taskName);
    if (idIt == m_impl->taskNameToTaskID.end())
        return false;

    return m_impl->humanIK.getPositionTaskSetPoint(idIt->second, position, linearVelocity);
}

bool BAFStateEstimator::getPoseTaskSetPoint(const std::string& taskName,
                                             Eigen::Vector3d& position,
                                             Eigen::Matrix3d& rotation,
                                             Eigen::Matrix<double, 6, 1>& velocity) const
{
    auto kindIt = m_impl->taskNameToTaskKind.find(taskName);
    if (kindIt == m_impl->taskNameToTaskKind.end() || kindIt->second != Impl::TaskKind::Pose)
        return false;

    auto idIt = m_impl->taskNameToTaskID.find(taskName);
    if (idIt == m_impl->taskNameToTaskID.end())
        return false;

    manif::SE3d W_H_frame;
    manif::SE3d::Tangent W_v_frame;
    if (!m_impl->humanIK.getPoseTaskSetPoint(idIt->second, W_H_frame, W_v_frame))
        return false;

    position = W_H_frame.translation();
    rotation = W_H_frame.rotation();
    velocity = W_v_frame.coeffs();
    return true;
}
