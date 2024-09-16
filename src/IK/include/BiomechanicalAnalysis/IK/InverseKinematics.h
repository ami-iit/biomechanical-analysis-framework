/**
 * @file InverseKinematic.h
 * @authors Davide Gorbani <davide.gorbani@iit.it>
 */

#ifndef BIOMECHANICAL_ANALYSIS_INVERSE_KINEMATIC_H
#define BIOMECHANICAL_ANALYSIS_INVERSE_KINEMATIC_H

// iDynTree
#include <iDynTree/KinDynComputations.h>

// BipedalLocomotion
#include <BipedalLocomotion/ContinuousDynamicalSystem/FloatingBaseSystemKinematics.h>
#include <BipedalLocomotion/ContinuousDynamicalSystem/ForwardEuler.h>
#include <BipedalLocomotion/IK/GravityTask.h>
#include <BipedalLocomotion/IK/JointLimitsTask.h>
#include <BipedalLocomotion/IK/JointTrackingTask.h>
#include <BipedalLocomotion/IK/JointVelocityLimitsTask.h>
#include <BipedalLocomotion/IK/QPInverseKinematics.h>
#include <BipedalLocomotion/IK/R3Task.h>
#include <BipedalLocomotion/IK/SO3Task.h>
#include <BipedalLocomotion/ParametersHandler/IParametersHandler.h>
#include <BipedalLocomotion/ParametersHandler/StdImplementation.h>
#include <BipedalLocomotion/System/VariablesHandler.h>

namespace BiomechanicalAnalysis
{

namespace IK
{

struct nodeData
{
    manif::SO3d I_R_IMU;
    manif::SO3Tangentd I_omega_IMU = manif::SO3d::Tangent::Zero();
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
     * @param handler pointer to the parameters handler
     * @return true if the SO3 task is initialized correctly
     */
    bool initializeOrientationTask(const std::string& taskName,
                                   const std::shared_ptr<BipedalLocomotion::ParametersHandler::IParametersHandler> taskHandler);

    /**
     * initialize the gravity task
     * @param taskName name of the task
     * @param handler pointer to the parameters handler
     * @return true if the gravity task is initialized correctly
     */
    bool initializeGravityTask(const std::string& taskName,
                               const std::shared_ptr<BipedalLocomotion::ParametersHandler::IParametersHandler> taskHandler);

    /**
     * initialize the R3 task
     * @param taskName name of the task
     * @param handler pointer to the parameters handler
     * @return true if the R3 task is initialized correctly
     */
    bool initializeFloorContactTask(const std::string& taskName,
                                    const std::shared_ptr<BipedalLocomotion::ParametersHandler::IParametersHandler> taskHandler);

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

    std::chrono::nanoseconds m_dtIntegration; /** Integration time step in nanoseconds */

    /**
     * Struct containing the integrator and the dynamics
     */
    struct System
    {
        std::shared_ptr<BipedalLocomotion::ContinuousDynamicalSystem::ForwardEuler<
            BipedalLocomotion::ContinuousDynamicalSystem::FloatingBaseSystemKinematics>>
            integrator;
        std::shared_ptr<BipedalLocomotion::ContinuousDynamicalSystem::FloatingBaseSystemKinematics> dynamics;
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

    /**
     * Struct containing the SO3 task from the BipedalLocomotion IK, the node number and the
     * rotation matrix between the IMU and the link
     */
    struct OrientationTaskStruct
    {
        std::shared_ptr<BipedalLocomotion::IK::SO3Task> task;
        int nodeNumber;
        manif::SO3d IMU_R_link; // Rotation matrix from the IMU to related link, set through config
                                // file
        manif::SO3d calibrationMatrix = manif::SO3d::Identity(); // Initialization (to Identity) of
                                                                 // Rotation matrix from the World
                                                                 // to the World of the IMU, which
                                                                 // will be calibrated using Tpose
                                                                 // script
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
        manif::SO3d IMU_R_link;
        manif::SO3d calibrationMatrix = manif::SO3d::Identity();
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
        int nodeNumber;
        bool footInContact{false};
        Eigen::Vector3d setPointPosition;
        std::string taskName;
        std::string frameName;
        double verticalForceThreshold;
    };

    std::shared_ptr<BipedalLocomotion::IK::JointTrackingTask> m_jointRegularizationTask; /** Joint
                                                                                           regularization
                                                                                           task */

    std::shared_ptr<BipedalLocomotion::IK::JointLimitsTask> m_jointConstraintsTask; /** Joint limits
                                                                                       task */

    std::shared_ptr<BipedalLocomotion::IK::JointVelocityLimitsTask> m_jointVelocityLimitsTask; /** Joint velocity limits task */

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

    std::shared_ptr<iDynTree::KinDynComputations> m_kinDyn; /** pointer to the KinDynComputations
    object */

    int m_nrDoFs; /** Number of Joint Degrees of Freedom */
    bool m_tPose{false}; /** Flag for resetting the integrator state */

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
     * task. Up to now, only the "SO3Task" is implemented.
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
     * |`FloorContactTask`|         `node_number`          |   `int`    |                  Node number of the task. The node number must be unique.               |  Yes  |
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
    bool updateFloorContactTask(const int node, const double verticalForce);

    /**
     * set the setpoint for the joint regularization task.
     * This function is to be called before the advance function to set the joint constraints
     * @param jointPositions joint positions, by defualt it is set to zero
     * @param jointVelocities joint velocities, by defualt it is set to zero
     * @return true if the joint regularization task is set correctly
     */
    bool updateJointRegularizationTask();

    /**
     * update the joint constraint task.
     * This function is to be called before the advance function to set the joint constraints
     * @return true if the joint regularization task is updated correctly
     */
    bool updateJointConstraintsTask();

    /**
     * update the orientation for all the nodes of the SO3 and gravity tasks
     * @param nodeStruct unordered map containing the node number and the calibration matrix
     * @return true if the calibration matrix is set correctly
     */
    bool updateOrientationAndGravityTasks(const std::unordered_map<int, nodeData>& nodeStruct);

    /**
     * update the floor contact task for all the nodes
     * @param footInContact unordered map containing the node number and the vertical force
     * @return true if the calibration matrix is set correctly
     */
    bool updateFloorContactTasks(const std::unordered_map<int, Eigen::Matrix<double, 6, 1>>& wrenchMap);

    /**
     * remove the offset on the yaw of the IMUs world
     * @param nodeStruct unordered map containing the node number and the IMUs measurements
     * @return true if the calibration matrix is set correctly
     */
    bool calibrateWorldYaw(std::unordered_map<int, nodeData> nodeStruct);

    /**
     * compute the calibration matrix between the IMU frame and the associated link frame
     * @param nodeStruct unordered map containing the node number and the IMUs measurements
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
};

} // namespace IK
} // namespace BiomechanicalAnalysis

#endif // BIOMECHANICAL_ANALYSIS_INVERSE_KINEMATIC_H
