/**
 * @file InverseKinematic.h
 * @authors Davide Gorbani <davide.gorbani@iit.it>
 */

#ifndef BIOMECHANICAL_ANALYSIS_INVERSE_KINEMATIC_H
#define BIOMECHANICAL_ANALYSIS_INVERSE_KINEMATIC_H

// iDynTree
#include <iDynTree/KinDynComputations.h>

// std
#include <unordered_set>

// BipedalLocomotion
#if __has_include(<BipedalLocomotion/ContinuousDynamicalSystem/FloatingBaseSystemVelocityKinematics.h>)
#include <BipedalLocomotion/ContinuousDynamicalSystem/FloatingBaseSystemVelocityKinematics.h>
#else
#include <BipedalLocomotion/ContinuousDynamicalSystem/FloatingBaseSystemKinematics.h>
namespace BipedalLocomotion
{
namespace ContinuousDynamicalSystem
{
typedef FloatingBaseSystemKinematics FloatingBaseSystemVelocityKinematics;
}
} // namespace BipedalLocomotion
#endif
#include <BipedalLocomotion/ContinuousDynamicalSystem/ForwardEuler.h>
#include <BipedalLocomotion/IK/GravityTask.h>
#include <BipedalLocomotion/IK/JointLimitsTask.h>
#include <BipedalLocomotion/IK/JointTrackingTask.h>
#include <BipedalLocomotion/IK/JointVelocityLimitsTask.h>
#include <BipedalLocomotion/IK/QPInverseKinematics.h>
#include <BipedalLocomotion/IK/R3Task.h>
#include <BipedalLocomotion/IK/SE3Task.h>
#include <BipedalLocomotion/IK/SO3Task.h>
#include <BipedalLocomotion/ParametersHandler/IParametersHandler.h>
#include <BipedalLocomotion/ParametersHandler/StdImplementation.h>
#include <BipedalLocomotion/System/VariablesHandler.h>

namespace BiomechanicalAnalysis
{

namespace IK
{

/**
 * @brief Struct containing the orientation and the angular velocity of an IMU
 */
struct nodeData
{
    manif::SO3d I_R_IMU;
    manif::SO3Tangentd I_omega_IMU = manif::SO3d::Tangent::Zero();
};

/**
 * @brief Struct containing the raw measured position and linear velocity of a frame M in the
 * sensor world S.
 * @note The IK computes the target link/task point by applying the calibrated world alignment and
 * the fixed translational extrinsic configured for the task.
 */
struct positionData
{
    Eigen::Vector3d S_p_M;
    Eigen::Vector3d S_v_M = Eigen::Vector3d::Zero();
};

/**
 * @brief Struct containing the raw measured pose and mixed 6D velocity of a frame M in the sensor
 * world S.
 * @note The IK computes the target link pose as W_H_L = W_H_S * S_H_M * M_H_L, with runtime
 * calibration updating only W_R_S while the translational part of M_H_L stays fixed from
 * initialization.
 */
struct poseData
{
    manif::SE3d S_H_M = manif::SE3d::Identity();
    manif::SE3d::Tangent S_v_M = manif::SE3d::Tangent::Zero();
};

// clang-format off
/**
 * @brief HumanIK class is a class in which the inverse kinematics problem is solved.
*/
// clang-format on
class HumanIK
{
private:
    /**
     * initialize the SO3 task
     * @param taskName name of the task
     * @param taskHandler pointer to the parameters handler
     * @param usedNodeNumbers set of node numbers already assigned to a task, updated on success
     * @return true if the SO3 task is initialized correctly
     */
    bool initializeOrientationTask(const std::string& taskName,
                                   const std::shared_ptr<BipedalLocomotion::ParametersHandler::IParametersHandler> taskHandler,
                                   std::unordered_set<int>& usedNodeNumbers);

    /**
     * initialize the gravity task
     * @param taskName name of the task
     * @param taskHandler pointer to the parameters handler
     * @param usedNodeNumbers set of node numbers already assigned to a task, updated on success
     * @return true if the gravity task is initialized correctly
     */
    bool initializeGravityTask(const std::string& taskName,
                               const std::shared_ptr<BipedalLocomotion::ParametersHandler::IParametersHandler> taskHandler,
                               std::unordered_set<int>& usedNodeNumbers);

    /**
     * initialize the floor contact task (R3Task)
     * @param taskName name of the task
     * @param taskHandler pointer to the parameters handler
     * @param usedNodeNumbers set of node numbers already assigned to a task, updated on success
     * @return true if the floor contact task is initialized correctly
     */
    bool initializeFloorContactTask(const std::string& taskName,
                                    const std::shared_ptr<BipedalLocomotion::ParametersHandler::IParametersHandler> taskHandler,
                                    std::unordered_set<int>& usedNodeNumbers);

    /**
     * Initialize the joint regularization task.
     * @param taskName Name of the task.
     * @param taskHandler Pointer to the parameters handler.
     * @return True if the joint regularization task is initialized correctly, false otherwise.
     */
    bool initializeJointRegularizationTask(const std::string& taskName,
                                           const std::shared_ptr<BipedalLocomotion::ParametersHandler::IParametersHandler> taskHandler);

    /**
     * Initialize the joint constraints task.
     * @param taskName Name of the task.
     * @param taskHandler Pointer to the parameters handler.
     * @return True if the joint constraints task is initialized correctly, false otherwise.
     */
    bool initializeJointConstraintsTask(const std::string& taskName,
                                        const std::shared_ptr<BipedalLocomotion::ParametersHandler::IParametersHandler> taskHandler);

    /**
     * Initialize the joints velocity limit task.
     * @param taskName Name of the task.
     * @param taskHandler Pointer to the parameters handler.
     * @return True if the joint velocity limits task is initialized correctly, false otherwise.
     */
    bool initializeJointVelocityLimitsTask(const std::string& taskName,
                                           const std::shared_ptr<BipedalLocomotion::ParametersHandler::IParametersHandler> taskHandler);

    bool
    initializeBaseVelocityRegularizationTask(const std::string& taskName,
                                             const std::shared_ptr<BipedalLocomotion::ParametersHandler::IParametersHandler> taskHandler);

    /**
     * Initialize the position task (R3Task).
     * @param taskName name of the task
     * @param taskHandler pointer to the parameters handler
     * @param usedNodeNumbers set of node numbers already assigned to a task, updated on success
     * @return true if the position task is initialized correctly
     */
    bool initializePositionTask(const std::string& taskName,
                                const std::shared_ptr<BipedalLocomotion::ParametersHandler::IParametersHandler> taskHandler,
                                std::unordered_set<int>& usedNodeNumbers);

    /**
     * Initialize the pose task (SE3Task).
     * @param taskName name of the task
     * @param taskHandler pointer to the parameters handler
     * @param usedNodeNumbers set of node numbers already assigned to a task, updated on success
     * @return true if the pose task is initialized correctly
     */
    bool initializePoseTask(const std::string& taskName,
                            const std::shared_ptr<BipedalLocomotion::ParametersHandler::IParametersHandler> taskHandler,
                            std::unordered_set<int>& usedNodeNumbers);

    /**
     * Accumulate the current base xy translation in the world anchor used by pose/position tasks.
     */
    void updateWorldAnchorTranslationFromCurrentBaseXY();

    std::chrono::nanoseconds m_dtIntegration; /** Integration time step in nanoseconds */

    /**
     * Struct containing the integrator and the dynamics
     */
    struct System
    {
        std::shared_ptr<BipedalLocomotion::ContinuousDynamicalSystem::ForwardEuler<
            BipedalLocomotion::ContinuousDynamicalSystem::FloatingBaseSystemVelocityKinematics>>
            integrator;
        std::shared_ptr<BipedalLocomotion::ContinuousDynamicalSystem::FloatingBaseSystemVelocityKinematics> dynamics;
    };

    System m_system; /** Struct containing the integrator and the dynamics */
    Eigen::VectorXd m_jointPositions; /** Position of the joints */
    Eigen::VectorXd m_jointVelocities; /** Velocity of the joints */
    Eigen::Matrix4d m_basePose; /** SO3 pose of the base */
    Eigen::Matrix<double, 6, 1> m_baseVelocity; /** Vector containing the linear and angular
                                                   velocity of the base */
    Eigen::Vector3d m_gravity; /** Gravity vector */
    manif::SO3d I_R_link_manif; /** orientation of the link in the inertial frame */
    manif::SO3Tangentd I_omega_link_manif; /** angular velocity of the link in the inertial frame */
    manif::SO3d I_R_link; /** orientation of the link in the inertial frame */
    manif::SO3Tangentd I_omega_link; /** angular velocity of the link in the inertial frame */

    Eigen::VectorXd m_calibrationJointPositions; /** Joint positions for calibration */

    Eigen::VectorXd m_jointPositionSetPoint; /** Custom set point for the regularization task */
    Eigen::VectorXd m_zeroOfDimensionNrDoFs; /** Buffer of dimention nrOfDoFs full of zeros */

    /**
     * Struct containing the SO3 task from the BipedalLocomotion IK, the node number and the
     * rotation matrix between the IMU and the link
     */
    struct OrientationTaskStruct
    {
        std::shared_ptr<BipedalLocomotion::IK::SO3Task> task;
        int nodeNumber;
        manif::SO3d IMU_R_link; // Rotation matrix from the IMU to related link
        manif::SO3d IMU_R_link_init; // Initial value of the rotation matrix from the IMU to related link, set through config
                                     // file
        manif::SO3d calibrationMatrix = manif::SO3d::Identity(); // Initialization (to Identity) of
                                                                 // Rotation matrix from the World
                                                                 // to the World of the IMU, which
                                                                 // will be calibrated using Tpose
                                                                 // script
        manif::SO3d W_R_link; // Calibrated orientation of the link in the inertial frame
        manif::SO3d W_R_link_setPoint = manif::SO3d::Identity(); // Last orientation setpoint passed to the SO3 task
        Eigen::Vector3d W_omega_link_setPoint = Eigen::Vector3d::Zero(); // Last angular velocity setpoint passed to the SO3 task
        bool hasSetPoint{false}; // True once updateOrientationTask() has passed a setpoint to the solver
        Eigen::Vector3d weight; // Weight of the task
        std::string frameName; // Name of the frame in which the task is expressed
    };

    /**
     * Struct containing the gravity task from the BipedalLocomotion IK, the node number and the
     * multiple state weight provider
     */
    struct GravityTaskStruct
    {
        std::shared_ptr<BipedalLocomotion::IK::GravityTask> task;
        manif::SO3d IMU_R_link; // Rotation matrix from the IMU to related link
        manif::SO3d IMU_R_link_init; // Initial value of the rotation matrix from the IMU to related link, set through config
                                     // file
        manif::SO3d calibrationMatrix = manif::SO3d::Identity();
        manif::SO3d W_R_link; // Calibrated orientation of the link in the inertial frame
        Eigen::Vector3d gravityDirectionSetPoint = Eigen::Vector3d::Zero(); // Last direction setpoint passed to the gravity task
        bool hasSetPoint{false}; // True once updateGravityTask() has passed a setpoint to the solver
        Eigen::Vector2d weight;
        int nodeNumber;
        std::string taskName;
        std::string frameName;
    };

    /**
     * Struct containing the R3 task from the BipedalLocomotion IK, the node number and the
     * multiple state weight provider
     */
    struct FloorContactTaskStruct
    {
        std::shared_ptr<BipedalLocomotion::IK::R3Task> task;
        Eigen::Vector3d weight;
        int taskNumber;
        bool footInContact{false};
        Eigen::Vector3d setPointPosition = Eigen::Vector3d::Zero();
        std::string taskName;
        std::string frameName;
        double verticalForceThreshold;
    };

    /**
     * Struct containing the R3 (position) task from BipedalLocomotion IK.
     */
    struct PositionTaskStruct
    {
        std::shared_ptr<BipedalLocomotion::IK::R3Task> task;
        Eigen::Vector3d weight;
        int nodeNumber;
        std::string taskName;
        std::string frameName;
        manif::SO3d W_R_S = manif::SO3d::Identity(); // World alignment rotation applied on the left.
        manif::SO3d S_R_M = manif::SO3d::Identity(); // Fixed orientation of the measured frame M in sensor world S,
                                                     // initialized from config and not recalibrated online.
        Eigen::Vector3d M_p_L = Eigen::Vector3d::Zero(); // Fixed translational extrinsic from the measured frame M to the
                                                         // task/link point L, expressed in M and loaded from config.
        Eigen::Vector3d W_p_frame_setPoint = Eigen::Vector3d::Zero(); // Last position setpoint passed to the R3 task
        Eigen::Vector3d W_v_frame_setPoint = Eigen::Vector3d::Zero(); // Last linear velocity setpoint passed to the R3 task
        bool hasSetPoint{false}; // True once updatePositionTask() has passed a setpoint to the solver
    };

    /**
     * Struct containing the SE3 (position + orientation) task from BipedalLocomotion IK.
     */
    struct PoseTaskStruct
    {
        std::shared_ptr<BipedalLocomotion::IK::SE3Task> task;
        Eigen::Matrix<double, 6, 1> weight;
        int nodeNumber;
        std::string taskName;
        std::string frameName;
        manif::SO3d M_R_L; // Fixed rotational extrinsic from the measured frame M to the related link L
        manif::SO3d M_R_L_init; // Initial value set through config file
        Eigen::Vector3d M_p_L = Eigen::Vector3d::Zero(); // Fixed translational extrinsic from the measured frame M to the link
                                                         // origin L, expressed in M and loaded from config.
        manif::SO3d W_R_S = manif::SO3d::Identity(); // Dynamic world alignment rotation updated by calibration.
        manif::SO3d W_R_link; // Calibrated orientation of the link in the inertial frame
        manif::SE3d W_H_frame_setPoint = manif::SE3d::Identity(); // Last pose setpoint passed to the SE3 task
        manif::SE3d::Tangent W_v_frame_setPoint = manif::SE3d::Tangent::Zero(); // Last mixed velocity setpoint passed to the SE3 task
        bool hasSetPoint{false}; // True once updatePoseTask() has passed a setpoint to the solver
    };

    std::shared_ptr<BipedalLocomotion::IK::JointTrackingTask> m_jointRegularizationTask; /** Joint
                                                                                           regularization
                                                                                           task */

    std::shared_ptr<BipedalLocomotion::IK::JointLimitsTask> m_jointConstraintsTask; /** Joint limits
                                                                                       task */

    std::shared_ptr<BipedalLocomotion::IK::JointVelocityLimitsTask> m_jointVelocityLimitsTask; /** Joint velocity limits task */

    struct BaseVelocityRegularizationTaskStruct
    {
        std::shared_ptr<BipedalLocomotion::IK::R3Task> linearVelocityTask; /** Linear velocity task */
        std::shared_ptr<BipedalLocomotion::IK::SO3Task> angularVelocityTask; /** Angular velocity task */
    };
    BaseVelocityRegularizationTaskStruct m_baseVelocityRegularizationTask; /** Struct containing the base
                                                                                      velocity regularization
                                                                                      tasks */

    manif::SO3d calib_W_R_link = manif::SO3d::Identity(); /** calibration matrix between the world
                                                           and the link */

    std::unordered_map<int, OrientationTaskStruct> m_OrientationTasks; /** unordered map of type
                                                                        OrientationTaskStruct, each
                                                                        element referring to a
                                                                        node*/

    std::unordered_map<int, GravityTaskStruct> m_GravityTasks; /** unordered map of the gravity
                                                                    tasks */

    std::unordered_map<int, FloorContactTaskStruct> m_FloorContactTasks; /** unordered map of the
                                                                    floor contact tasks */

    std::unordered_map<int, PositionTaskStruct> m_PositionTasks; /** unordered map of the position tasks */
    std::unordered_map<int, PoseTaskStruct> m_PoseTasks; /** unordered map of the pose tasks */

    std::shared_ptr<iDynTree::KinDynComputations> m_kinDyn; /** pointer to the KinDynComputations
    object */

    int m_nrDoFs; /** Number of Joint Degrees of Freedom */
    bool m_tPose{false}; /** Flag for resetting the integrator state */
    Eigen::Vector3d m_worldAnchorTranslation = Eigen::Vector3d::Zero(); /** World translation
                                                                          anchor applied to pose
                                                                          and position task
                                                                          setpoints. */

    BipedalLocomotion::IK::QPInverseKinematics m_qpIK; /** QP Inverse Kinematics solver */
    BipedalLocomotion::System::VariablesHandler m_variableHandler; /** Variables handler */

public:
    /**
     * Constructor
     */
    HumanIK(){};

    /**
     * Destructor
     */
    ~HumanIK(){};

    // clang-format off
    /**
     * initialize all the task and the inverse kinematics solver
     * @param handler pointer to the parameters handler
     * @param kinDyn pointer to the KinDynComputations object
     * @return true if all the tasks are initialized correctly
     * @note the following parameters are required by the class
     * |   Group   |         Parameter Name         |       Type      |                                           Description                                          | Mandatory |
     * |:---------:|:------------------------------:|:---------------:|:----------------------------------------------------------------------------------------------:|:---------:|
     * |           |           `tasks`              | `vector<string>`|         Vector containing the list of the tasks considered in the IK.                          |    Yes    |
     * |   `IK`    | `robot_velocity_variable_name` |     `string`    | Name of the variable contained in `VariablesHandler` describing the generalized robot velocity |    Yes    |
     * |   `IK`    |           `verbosity`          |      `bool`     |                         Verbosity of the solver. Default value `false`                         |     No    |
     * Where the generalized robot velocity is a vector containing the base spatialvelocity
     * (expressed in mixed representation) and the joint velocities.
     * For **each** task listed in the parameter `tasks` the user must specify all the parameters
     * required by the task itself but `robot_velocity_variable_name` since is already specified in
     * the `IK` group. Moreover, each task requires a parameter `type` that identifies the type of
     * task. The supported task types are: `SO3Task`, `GravityTask`, `FloorContactTask`,
     * `JointRegularizationTask`, `JointConstraintTask`, `JointVelocityLimitsTask`,
     * `BaseVelocityRegularizationTask`, `PositionTask`, `PoseTask`.
     * The "SO3Task" requires the following parameters:
     * |   Group   |         Parameter Name         |       Type      |                                       Description                                       | Mandatory |
     * |:---------:|:------------------------------:|:---------------:|:---------------------------------------------------------------------------------------:|:---------:|
     * | `SO3Task` |           `type`               |     `string`    |                         Type of the task. The value to be set is `SO3Task`              |  Yes |
     * | `SO3Task` | `robot_velocity_variable_name` |     `string`    |Name of the variable contained in `VariablesHandler` describing the generalized robot velocity|  Yes |
     * | `SO3Task` |        `node_number`           |      `int`      |                    Node number of the task. The node number must be unique.             |  Yes |
     * | `SO3Task` |      `rotation_matrix`         | `vector<double>`|    Rotation matrix between the IMU and the link. By default it set to identity.         |  No  |
     * | `SO3Task` |         `frame_name`           |     `string`    |                          Name of the frame in which the task is expressed.              |  Yes |
     * | `SO3Task` |         `kp_angular`           |     `double`    |                        Value of the gain of the angular velocity feedback.              |  Yes |
     * | `SO3Task` |           `weight`             | `vector<double>`|                        Weight of the task. Default value is (1.0, 1.0, 1.0)             |  yes |
     * `SO3Task` is a placeholder for the name of the task contained in the `tasks` list.
     *
     * The "GravityTask" requires the following parameters:
     * |    Group    |         Parameter Name         |    Type    |                                         Description                                          | Mandatory |
     * |:-----------:|:------------------------------:|:----------:|:--------------------------------------------------------------------------------------------:|:---------:|
     * |`GravityTask`|           `type`               |  `string`  |                         Type of the task. The value to be set is `GravityTask`               |  Yes  |
     * |`GravityTask`| `robot_velocity_variable_name` |  `string`  |Name of the variable contained in `VariablesHandler` describing the generalized robot velocity|  Yes  |
     * |`GravityTask`|        `node_number`           |   `int`    |                    Node number of the task. The node number must be unique.                  |  Yes  |
     * |`GravityTask`|            `kp`                |  `double`  |                          Gain of the distance controller                                     |  Yes  |
     * |`GravityTask`|     `target_frame_name`        |  `string`  |                 Name of the frame to which apply the gravity task                            |  Yes  |
     * |`GravityTask`|           `weight`             |`vector<double>`|                                Weight of the task                                        |  Yes  |
     * |`GravityTask`|      `rotation_matrix`         |`vector<double>`|     Rotation matrix between the IMU and the link. By default it set to identity.         |  No   |
     *
     * The "floorContactTask" requires the following parameters:
     * |    Group    |         Parameter Name         |    Type    |                                         Description                                          | Mandatory |
     * |:-----------:|:------------------------------:|:----------:|:--------------------------------------------------------------------------------------------:|:---------:|
     * |`FloorContactTask`|           `type`               |  `string`  |                       Type of the task. The value to be set is `FloorContactTask`       |  Yes  |
     * |`FloorContactTask`| `robot_velocity_variable_name` |  `string`  |Name of the variable contained in `VariablesHandler` describing the generalized robot velocity|  Yes  |
     * |`FloorContactTask`|         `floor_contact_task`   |   `int`    |                  Node number of the task. The node number must be unique.               |  Yes  |
     * |`FloorContactTask`|          `kp_linear`           |  `double`  |                          Gain of the distance controller                                |  Yes  |
     * |`FloorContactTask`|         `frame_name`           |  `string`  |                 Name of the frame to which apply the floor contact task                 |  Yes  |
     * |`FloorContactTask`|   `vertical_force_threshold`   |  `double`  |                 Threshold of the vertical force to consider the foot in contact         |  Yes  |
     * |`FloorContactTask`|           `weight`             |  `vector<double>`  |                           Weight of the task                                    |  Yes  |
     *
     * The "JointRegularizationTask" requires the following parameters:
     * |          Group          |         Parameter Name         |    Type    |                                         Description                                          | Mandatory |
     * |:-----------------------:|:------------------------------:|:----------:|:--------------------------------------------------------------------------------------------:|:---------:|
     * |`JointRegularizationTask`|            `type`              |  `string`  |                Type of the task. The value to be set is `JointRegularizationTask`            |    Yes    |
     * |`JointRegularizationTask`| `robot_velocity_variable_name` |  `string`  |Name of the variable contained in `VariablesHandler` describing the generalized robot velocity|    Yes    |
     * |`JointRegularizationTask`|           `weight`             |  `double`  |                                Weight associated to the task.                                |    No     |
     *
     * The "JointConstraintsTask" requires the following parameters:
     * |        Group         |         Parameter Name         |    Type    |                                         Description                                          | Mandatory |
     * |:--------------------:|:------------------------------:|:----------:|:--------------------------------------------------------------------------------------------:|:---------:|
     * |`JointConstraintsTask`|            `type`              |  `string`  |                 Type of the task. The value to be set is `JointConstraintsTask`              |    Yes    |
     * |`JointConstraintsTask`| `robot_velocity_variable_name` |  `string`  |Name of the variable contained in `VariablesHandler` describing the generalized robot velocity|    Yes    |
     * |`JointConstraintsTask`|       `use_model_limits`       |   `bool`   |           Flag to be set to true if the limits to be used are the ones of the urdf model     |    Yes    |
     * |`JointConstraintsTask`|        `sampling_time`         |  `double`  |                                   Sampling time in seconds                                   |    Yes    |
     * |`JointConstraintsTask`|          `k_limits`            |  `double`  |             Proportional controller gain. It must be a positive number lower than 1          |    Yes    |
     * |`JointConstraintsTask`|        `joints_list`           |`vector<string>`| Vector containing the joints name to set the limits. Required `use_model_limits` is set to false.   |    No     |
     * |`JointConstraintsTask`|        `upper_limits`          |`vector<double>`| Vector containing the upper limits of the specified joints. Required `use_model_limits` is set to false.   |    No     |
     * |`JointConstraintsTask`|        `lower_limits`          |`vector<double>`| Vector containing the lower limits of the specified joints. Required `use_model_limits` is set to false.   |    No     |
     *
     * The "PositionTask" controls the 3D position of a frame using a proportional controller in R3.
        * The set-point is provided at runtime via `updatePositionTask()` as a raw measured position of
        * frame M in sensor world S. The task applies the fixed extrinsic from M to the controlled point.
     * |      Group       |         Parameter Name         |       Type          |                                         Description                                          | Mandatory |
     * |:----------------:|:------------------------------:|:-------------------:|:--------------------------------------------------------------------------------------------:|:---------:|
     * | `PositionTask`   |           `type`               |     `string`        |                     Type of the task. The value to be set is `PositionTask`                  |    Yes    |
     * | `PositionTask`   | `robot_velocity_variable_name` |     `string`        | Name of the variable contained in `VariablesHandler` describing the generalized robot velocity|    Yes    |
     * | `PositionTask`   |        `node_number`           |      `int`          |                    Node number of the task. The node number must be unique.                  |    Yes    |
     * | `PositionTask`   |         `frame_name`           |     `string`        |                          Name of the frame whose position is controlled.                     |    Yes    |
        * | `PositionTask`   |      `rotation_matrix`         | `vector<double>`    | Fixed orientation of measured frame M in sensor world S. Default is identity.               |    No     |
        * | `PositionTask`   |      `position_offset`         | `vector<double>`    | Fixed translation from measured frame M to the controlled point, expressed in M.            |    No     |
     * | `PositionTask`   |         `kp_linear`            | `double` or `vector<double>` |              Gain of the proportional position controller.                          |    Yes    |
     * | `PositionTask`   |           `weight`             |  `vector<double>`   |          Weight of the task (3 elements).                                                    |    Yes    |
     * | `PositionTask`   |            `mask`              |  `vector<bool>`     |  Mask to control only a subset of axes, e.g. `[1,0,1]` for x and z only. Default `[1,1,1]` |    No     |
     *
     * The "PoseTask" controls both position and orientation of a frame using proportional controllers
    * in R3 and SO3. The set-point is provided at runtime via `updatePoseTask()` as a raw measured
    * pose S_H_M. Runtime calibration updates only W_R_S, while the full extrinsic M_H_L stays fixed
    * from initialization.
     * |    Group    |         Parameter Name         |       Type          |                                         Description                                          | Mandatory |
     * |:-----------:|:------------------------------:|:-------------------:|:--------------------------------------------------------------------------------------------:|:---------:|
     * | `PoseTask`  |           `type`               |     `string`        |                       Type of the task. The value to be set is `PoseTask`                    |    Yes    |
     * | `PoseTask`  | `robot_velocity_variable_name` |     `string`        | Name of the variable contained in `VariablesHandler` describing the generalized robot velocity|    Yes    |
     * | `PoseTask`  |        `node_number`           |      `int`          |                    Node number of the task. The node number must be unique.                  |    Yes    |
     * | `PoseTask`  |         `frame_name`           |     `string`        |                    Name of the frame whose pose (position + orientation) is controlled.      |    Yes    |
    * | `PoseTask`  |      `rotation_matrix`         | `vector<double>`    | Fixed rotational extrinsic from measured frame M to controlled frame L. Default is identity. |    No     |
    * | `PoseTask`  |      `position_offset`         | `vector<double>`    | Fixed translation from measured frame M to controlled frame L, expressed in M.              |    No     |
     * | `PoseTask`  |         `kp_linear`            | `double` or `vector<double>` |              Gain of the proportional position controller.                          |    Yes    |
     * | `PoseTask`  |        `kp_angular`            | `double` or `vector<double>` |              Gain of the proportional orientation controller.                       |    Yes    |
     * | `PoseTask`  |           `weight`             |  `vector<double>`   |          Weight of the task (6 elements: 3 linear + 3 angular).                              |    Yes    |
     * | `PoseTask`  |            `mask`              |  `vector<bool>`     |  Mask to control only a subset of linear axes. Default `[1,1,1]`. Angular part is always controlled. |    No     |
     * @note The following `ini` file presents an example of the configuration that can be used to
     * build the HumanIK class.
     *  ~~~~~{.ini}
     * tasks                           ("PELVIS_TASK", "GRAVITY_TASK", "RIGHT_TOE_TASK", "JOINT_LIMITS_TASK", "JOINT_REG_TASK")
     *
     * [IK]
     * robot_velocity_variable_name    "robot_velocity"
     * verbosity                       false
     *
     * [PELVIS_TASK]
     * type                            "SO3Task"
     * robot_velocity_variable_name    "robot_velocity"
     * frame_name                      "Pelvis"
     * kp_angular                      5.0
     * node_number                     3
     * weight                          (1.0, 1.0, 1.0)
     * rotation_matrix                 (0.0, 1.0, 0.0,
     *                                  0.0, 0.0, -1.0,
     *                                 -1.0, 0.0, 0.0)
     *
     * [GRAVITY_TASK]
     * type                            "GravityTask"
     * robot_velocity_variable_name    "robot_velocity"
     * target_frame_name               "link10"
     * kp                              1.0
     * node_number                     10
     * weight                          (1.0 1.0)
     *
     * [RIGHT_TOE_TASK]
     * type                            "FloorContactTask"
     * robot_velocity_variable_name    "robot_velocity"
     * frame_name                      "RightToe"
     * kp_linear                       60.0
     * node_number                     2
     * weight                          (1.0 1.0 1.0)
     * vertical_force_threshold        60.0
     *
     * [JOINT_LIMITS_TASK]
     * type                            "JointConstraintTask"
     * robot_velocity_variable_name    "robot_velocity"
     * use_model_limits                false
     * sampling_time                   0.01
     * k_limits                        1.0
     * joints_list                     ("jLeftKnee_rotz", "jRightKnee_rotz", "jLeftAnkle_rotz", "jRightAnkle_rotz")
     * upper_bounds                    (0.0, 0.0, 0.0, 0.0)
     * lower_bounds                    (0.0, 0.0, 0.0, 0.0)
     *
     * [JOINT_REG_TASK]
     * type                            "JointRegularizationTask"
     * robot_velocity_variable_name    "robot_velocity"
     * weight                          0.000001
    */
    // clang-format on
    bool initialize(std::weak_ptr<const BipedalLocomotion::ParametersHandler::IParametersHandler> handler,
                    std::shared_ptr<iDynTree::KinDynComputations> kinDyn);

    /**
     * set the integration time step
     * @param dt integration time step in seconds
     * @return true if the integration time step is set correctly
     */
    bool setDt(const double dt);

    /**
     * get the integration time step
     * @return integration time step in seconds
     */
    double getDt() const;

    /**
     * get the number of DoFs
     * @return number of DoFs
     */
    int getDoFsNumber() const;

    /**
     * set the orientation and the angular velocity for a given node of a SO3 task
     * @param node node number
     * @param I_R_IMU orientation of the IMU
     * @param I_omega_IMU angular velocity of the IMU
     * @return true if the orientation setpoint is set correctly
     */
    bool
    updateOrientationTask(const int node, const manif::SO3d& I_R_IMU, const manif::SO3Tangentd& I_omega_IMU = manif::SO3d::Tangent::Zero());

    /**
     * set the orientation setpoint for a given node of a gravity task
     * @param node node number
     * @param I_R_IMU orientation of the IMU
     * @param I_omega_IMU angular velocity of the IMU
     * @return true if the orientation setpoint is set correctly
     */
    bool updateGravityTask(const int node, const manif::SO3d& I_R_IMU);

    /**
     * set the position setpoint for a given node of a floor contact task
     * @param node node number
     * @param verticalForce vertical force
     * @return true if the orientation setpoint is set correctly
     */
    bool updateFloorContactTask(const int node, const double verticalForce, const double linkHeight = 0.0);

    /**
     * set the setpoint for the joint regularization task.
     * This function is to be called before the advance function to set the joint constraints
     * This function does not allow to set the position regularization setpoint
     * @param jointPositions joint positions, by defualt it is set to zero
     * @param jointVelocities joint velocities, by defualt it is set to zero
     * @return true if the joint regularization task is set correctly
     */
    bool updateJointRegularizationTask();

    /**
     * set the setpoint for the joint regularization task.
     * This function is to be called before the advance function to set the joint constraints
     * This function allows to set the position regularization setpoint
     * @param jointPositionSetPoint setpoint for the joint regularization task: it is a vector of the same size of the number of DoFs
     * @return true if the joint regularization task is set correctly
     */
    bool updateJointRegularizationTask(const Eigen::VectorXd& jointPositionSetPoint);

    /**
     * get the setpoint for the joint regularization task.
     */
    Eigen::VectorXd getJointPositionSetPoint();

    /**
     * update the joint constraint task.
     * This function is to be called before the advance function to set the joint constraints
     * @return true if the joint regularization task is updated correctly
     */
    bool updateJointConstraintsTask();

    /**
     * update the orientation for all the nodes of the SO3 and gravity tasks
     * @param nodeStruct unordered map containing the struct node data (see
     * https://github.com/ami-iit/biomechanical-analysis-framework/blob/338129086dca24989552a20ecc1c9dec0492806a/src/IK/include/BiomechanicalAnalysis/IK/InverseKinematics.h#L32)
     * containing the orientation and the angular velocity of an IMU, associated to the node number
     * @return true if the calibration matrix is set correctly
     */
    bool updateOrientationAndGravityTasks(const std::unordered_map<int, nodeData>& nodeStruct);

    /**
     * update the floor contact task for all the nodes
     * @param footInContact unordered map containing the node number and the vertical force
     * @return true if the calibration matrix is set correctly
     */
    bool updateFloorContactTasks(const std::unordered_map<int, Eigen::Matrix<double, 6, 1>>& wrenchMap, const double linkHeight = 0.0);

    /**
     * Set the position set-point for a given position task node.
     * @param node node number
     * @param data raw measured position and linear velocity of frame M in sensor world S
     * @return true if the set-point is set correctly
     */
    bool updatePositionTask(const int node, const positionData& data);

    /**
     * Set the position set-point for all position task nodes.
     * @param positionMap unordered map from node number to desired positionData
     * @return true if all set-points are set correctly
     */
    bool updatePositionTasks(const std::unordered_map<int, positionData>& positionMap);

    /**
     * Set the pose set-point for a given pose task node.
     * @param node node number
     * @param data raw measured pose and mixed 6D velocity of frame M in sensor world S
     * @return true if the set-point is set correctly
     */
    bool updatePoseTask(const int node, const poseData& data);

    /**
     * Set the pose set-point for all pose task nodes.
     * @param poseMap unordered map from node number to desired poseData
     * @return true if all set-points are set correctly
     */
    bool updatePoseTasks(const std::unordered_map<int, poseData>& poseMap);

    /**
     * clear the calibration rotations and fixed extrinsics of all tasks to their initialized values
     * @return true if the calibration matrices are cleared correctly
     */
    bool clearCalibrationMatrices();

    /**
     * remove the offset on the yaw of the IMUs world
     * @param nodeStruct unordered map containing the struct node data (see
     * https://github.com/ami-iit/biomechanical-analysis-framework/blob/338129086dca24989552a20ecc1c9dec0492806a/src/IK/include/BiomechanicalAnalysis/IK/InverseKinematics.h#L32)
     * containing the orientation and the angular velocity of an IMU, associated to the node number
     * @return true if the calibration matrix is set correctly
     * @note gravity is expected to be aligned with the z-axis of the IMU frame
     */
    bool calibrateWorldYaw(std::unordered_map<int, nodeData> nodeStruct);

    /**
     * compute the calibration matrix between the IMU frame and the associated link frame
     * @param nodeStruct unordered map containing the struct node data (see
     * https://github.com/ami-iit/biomechanical-analysis-framework/blob/338129086dca24989552a20ecc1c9dec0492806a/src/IK/include/BiomechanicalAnalysis/IK/InverseKinematics.h#L32)
     * containing the orientation and the angular velocity of an IMU, associated to the node number
     * @param frameRef reference frame used as world
     * @return true if the calibration matrix is set correctly
     */
    bool calibrateAllWithWorld(std::unordered_map<int, nodeData> nodeStruct, std::string frameRef = "");

    /**
     * this function solves the inverse kinematics problem and integrate the joint velocity to
     * compute the joint positions and the base pose; it also updates the state of the
     * KinDynComputations object passed to the class
     * @return true if the inverse kinematics solver is advanced correctly
     */
    bool advance();

    /**
     * get the joint positions
     * @param jointPositions joint positions
     * @return true if the joint positions are retrieved correctly
     */
    bool getJointPositions(Eigen::Ref<Eigen::VectorXd> jointPositions) const;

    /**
     * get the joint velocities
     * @param jointVelocities joint velocities
     * @return true if the joint velocities are retrieved correctly
     */
    bool getJointVelocities(Eigen::Ref<Eigen::VectorXd> jointVelocities) const;

    /**
     * get the base position
     * @param basePosition base position
     * @return true if the base position is retrieved correctly
     */
    bool getBasePosition(Eigen::Ref<Eigen::Vector3d> basePosition) const;

    /**
     * get the base linear velocity
     * @param baseVelocity base linear velocity
     * @return true if the base linear velocity is retrieved correctly
     */
    bool getBaseLinearVelocity(Eigen::Ref<Eigen::Vector3d> baseVelocity) const;

    /**
     * get the base orientation
     * @param baseOrientation base orientation
     * @return true if the base orientation is retrieved correctly
     */
    bool getBaseOrientation(Eigen::Ref<Eigen::Matrix3d> baseOrientation) const;

    /**
     * get the base angular velocity
     * @param baseAngularVelocity base angular velocity
     * @return true if the base angular velocity is retrieved correctly
     */
    bool getBaseAngularVelocity(Eigen::Ref<Eigen::Vector3d> baseAngularVelocity) const;

    /**
     * get the calibration matrix between the world and the link
     * @param node node number
     * @return calibration matrix
     */
    const manif::SO3d& getCalibratedIMURotation(int node) const;

    /**
     * get the frame name of a node
     * @param node node number
     * @return frame name
     */
    std::string getNodeFrameName(int node) const;

    /**
     * Get the last setpoint passed to a SO3 orientation task.
     * @param node node number
     * @param W_R_link orientation setpoint passed to the solver
     * @param W_omega_link angular velocity setpoint passed to the solver
     * @return true if a setpoint is available and copied in output parameters
     */
    bool getOrientationTaskSetPoint(int node, manif::SO3d& W_R_link, Eigen::Vector3d& W_omega_link) const;

    /**
     * Get the last setpoint passed to a gravity task.
     * @param node node number
     * @param gravityDirection direction setpoint passed to the solver
     * @return true if a setpoint is available and copied in output parameters
     */
    bool getGravityTaskSetPoint(int node, Eigen::Vector3d& gravityDirection) const;

    /**
     * Get the last setpoint passed to a R3 position task.
     * @param node node number
     * @param W_p_frame position setpoint passed to the solver
     * @param W_v_frame linear velocity setpoint passed to the solver
     * @return true if a setpoint is available and copied in output parameters
     */
    bool getPositionTaskSetPoint(int node, Eigen::Vector3d& W_p_frame, Eigen::Vector3d& W_v_frame) const;

    /**
     * Get the last setpoint passed to a SE3 pose task.
     * @param node node number
     * @param W_H_frame pose setpoint passed to the solver
     * @param W_v_frame mixed velocity setpoint passed to the solver
     * @return true if a setpoint is available and copied in output parameters
     */
    bool getPoseTaskSetPoint(int node, manif::SE3d& W_H_frame, manif::SE3d::Tangent& W_v_frame) const;

    /**
     * Get the current world anchor translation used to re-anchor pose/position setpoints.
     * @param worldAnchorTranslation anchor translation currently applied in world frame
     * @return true if the anchor is retrieved correctly
     */
    bool getWorldAnchorTranslation(Eigen::Ref<Eigen::Vector3d> worldAnchorTranslation) const;
};

} // namespace IK
} // namespace BiomechanicalAnalysis

#endif // BIOMECHANICAL_ANALYSIS_INVERSE_KINEMATIC_H
