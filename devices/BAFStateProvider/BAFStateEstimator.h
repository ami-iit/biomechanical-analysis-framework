// SPDX-FileCopyrightText: Fondazione Istituto Italiano di Tecnologia (IIT)
// SPDX-License-Identifier: BSD-3-Clause

#ifndef BAF_STATE_ESTIMATOR_H
#define BAF_STATE_ESTIMATOR_H

#include <BiomechanicalAnalysis/IK/InverseKinematics.h>
#include <BipedalLocomotion/ParametersHandler/IParametersHandler.h>
#include <iDynTree/KinDynComputations.h>
#include <iDynTree/VectorFixSize.h>
#include <iDynTree/Position.h>

#include <Eigen/Core>

#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

/**
 * BAFStateEstimator wraps BiomechanicalAnalysis::IK::HumanIK to provide a clean, sensor-agnostic
 * interface for whole-body state estimation. All internal indices (BAF node_number) are resolved
 * from the IK configuration and are never exposed to the caller.
 *
 * Inputs use IK task names as keys, matching the entries in the BAF IK "tasks" list.
 * The estimator resolves each task name to the corresponding HumanIK integer id.
 */
class BAFStateEstimator
{
public:
    /**
     * Classification of IK task types based on the data they require from sensors.
     */
    enum class KinematicTaskKind
    {
        Orientation, ///< SO3Task or GravityTask: orientation (+ angular velocity if available)
        Position,    ///< PositionTask: 3D position [m]
        Pose,        ///< PoseTask: full SE3 pose (rotation + position)
        FloorContact ///< FloorContactTask: 6D force/torque wrench [N; N⋅m]
    };

    BAFStateEstimator();
    ~BAFStateEstimator();

    /**
     * Initialize the estimator.
     * @param ikParams  Parameters handler already loaded from the BAF IK TOML file.
     *                  The same handler is passed to HumanIK::initialize() and is also
     *                  re-read to build the taskName -> task ID lookup tables.
     * @param kinDyn    Shared KinDynComputations loaded with the URDF model.
     * @return true on success.
     */
    bool initialize(
        std::shared_ptr<BipedalLocomotion::ParametersHandler::IParametersHandler> ikParams,
        std::shared_ptr<iDynTree::KinDynComputations> kinDyn);

    /**
     * Set the integration time step (in seconds).  Must be called after initialize().
     */
    bool setDt(double dt);

    /**
     * Provide new sensor readings before calling advance().
    * @param nodeData      Map from task name to IMU orientation + angular velocity.
    *                      Only SO3Task/GravityTask entries are used.
    * @param ftData        Optional map from task name to 6D wrench (force then torque).
    *                      Only FloorContactTask entries are
     *                      used. The vertical force component (index 2) drives contact detection.
    * @param positionData  Optional map from task name to desired position + velocity.
    *                      Only PositionTask entries are used.
    * @param poseData      Optional map from task name to desired pose + velocity.
    *                      Only PoseTask entries are used.
     */
    void updateInput(
        const std::unordered_map<std::string, BiomechanicalAnalysis::IK::nodeData>& nodeData,
        const std::unordered_map<std::string, Eigen::Matrix<double, 6, 1>>& ftData = {},
        const std::unordered_map<std::string, BiomechanicalAnalysis::IK::positionData>& positionData = {},
        const std::unordered_map<std::string, BiomechanicalAnalysis::IK::poseData>& poseData = {});

    /**
     * Run one IK step.  Call updateInput() before each call to advance().
     * @return true if the IK solver succeeded.
     */
    bool advance();

    /**
     * Schedule a T-pose calibration.  The calibration will be performed at the start of the
     * next advance() call using the data provided here.
     * @param nodeData        Current IMU readings keyed by task name.
     * @param referenceFrame  URDF link name used as the world-frame anchor.
     */
    void calibrate(
        const std::unordered_map<std::string, BiomechanicalAnalysis::IK::nodeData>& nodeData,
        const std::string& referenceFrame = "");

    /**
     * Clear all calibration matrices (equivalent to resetting the T-pose).
     */
    bool clearCalibration();

    /**
     * Reset the world anchor translation used to shift pose/position task setpoints.
     * @return true if the anchor is reset correctly.
     */
    bool resetWorldAnchorTranslation();

    /**
     * Recenter pose/position targets by accumulating the current base XY into the world anchor.
     * @return true if the anchor is updated correctly.
     */
    bool recenterWorldAnchor();

    /**
     * Reset the internal joint state used by the IK solver.
     * @return true if the joint state is reset correctly.
     */
    bool resetJointState();

    // ── State getters ─────────────────────────────────────────────────────────
    // All values are valid after a successful advance().

    const Eigen::VectorXd& getJointPositions() const;
    const Eigen::VectorXd& getJointVelocities() const;
    /** Base orientation as a 3×3 rotation matrix (world frame). */
    Eigen::Matrix3d getBaseOrientation() const;
    /** Base position in the world frame [m]. */
    const Eigen::Vector3d& getBasePosition() const;
    const Eigen::Vector3d& getBaseLinearVelocity() const;
    const Eigen::Vector3d& getBaseAngularVelocity() const;

    /** Centre of mass position (requires a successful advance()). */
    iDynTree::Position getCoMPosition() const;
    /** Centre of mass velocity (requires a successful advance()). */
    iDynTree::Vector3 getCoMVelocity() const;

    size_t getNumberOfJoints() const;
    const std::vector<std::string>& getJointNames() const;

    // ── Solver setpoint getters ───────────────────────────────────────────────
    // These return the actual setpoints that were last passed to the IK solver
    // (i.e. calibrated, transformed values), not the raw sensor inputs.
    // Returns false if the task is unknown or no setpoint has been computed yet.

    /**
    * Get the last orientation setpoint passed to the SO3/GravityTask solver for a given task.
    * @param taskName  IK task name.
     * @param rotation  3×3 rotation matrix W_R_link (world ← link).
     * @param angularVelocity  Angular velocity setpoint [rad/s] in world frame.
     */
    bool getOrientationTaskSetPoint(const std::string& taskName,
                                    Eigen::Matrix3d& rotation,
                                    Eigen::Vector3d& angularVelocity) const;

    /**
    * Get the last position setpoint passed to the PositionTask solver for a given task.
    * @param taskName       IK task name.
     * @param position       Position setpoint [m] in world frame.
     * @param linearVelocity Linear velocity setpoint [m/s] in world frame.
     */
    bool getPositionTaskSetPoint(const std::string& taskName,
                                 Eigen::Vector3d& position,
                                 Eigen::Vector3d& linearVelocity) const;

    /**
    * Get the last pose setpoint passed to the PoseTask solver for a given task.
    * @param taskName       IK task name.
     * @param position       Position part of the pose setpoint [m] in world frame.
     * @param rotation       Rotation part of the pose setpoint (3×3, world ← link).
     * @param velocity       Mixed 6D velocity setpoint [m/s; rad/s] in world frame.
     */
    bool getPoseTaskSetPoint(const std::string& taskName,
                             Eigen::Vector3d& position,
                             Eigen::Matrix3d& rotation,
                             Eigen::Matrix<double, 6, 1>& velocity) const;

private:
    class Impl;
    std::unique_ptr<Impl> m_impl;
};

#endif // BAF_STATE_ESTIMATOR_H
