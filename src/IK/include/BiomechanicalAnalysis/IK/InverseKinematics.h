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
#include <BipedalLocomotion/ContinuousDynamicalSystem/MultiStateWeightProvider.h>
#include <BipedalLocomotion/IK/GravityTask.h>
#include <BipedalLocomotion/IK/QPInverseKinematics.h>
#include <BipedalLocomotion/IK/SO3Task.h>
#include <BipedalLocomotion/ParametersHandler/IParametersHandler.h>
#include <BipedalLocomotion/System/VariablesHandler.h>

namespace BiomechanicalAnalysis
{

namespace IK
{

// clang-format off
/**
 * @brief HumanIK class is a class in which the inverse kinematics problem is solved.
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
 * | `SO3Task` |           `node`               |      `int`      |                    Node number of the task. The node number must be unique.             |  Yes |
 * | `SO3Task` |      `rotation_matrix`         | `vector<double>`|    Rotation matrix between the IMU and the link. By default it set to identity.         |  No  |
 * | `SO3Task` |         `frame_name`           |     `string`    |                          Name of the frame in which the task is expressed.              |  Yes |
 * | `SO3Task` |         `kp_angular`           |     `double`    |                        Value of the gain of the angular velocity feedback.              |  Yes |
 * `SO3Task` is a placeholder for the name of the task contained in the `tasks` list.
*/
// clang-format on
class HumanIK
{
private:
    bool initializeGravityTask(
        const std::string& taskName,
        const std::shared_ptr<BipedalLocomotion::ParametersHandler::IParametersHandler> handler);
    std::chrono::nanoseconds m_dtIntegration; /** Integration time step in nanoseconds */

    /**
     * Struct containing the integrator and the dynamics
     */
    struct System
    {
        std::shared_ptr<BipedalLocomotion::ContinuousDynamicalSystem::ForwardEuler<
            BipedalLocomotion::ContinuousDynamicalSystem::FloatingBaseSystemKinematics>>
            integrator;
        std::shared_ptr<BipedalLocomotion::ContinuousDynamicalSystem::FloatingBaseSystemKinematics>
            dynamics;
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
        manif::SO3d IMU_R_link;
        manif::SO3d calibrationMatrix = manif::SO3d::Identity();
    };

    /**
     * Struct containing the gravity task from the BipedalLocomotion IK, the node number and the
     * multiple state weight provider
     */
    struct GravityTaskStruct
    {
        std::shared_ptr<BipedalLocomotion::IK::GravityTask> task;
        std::shared_ptr<BipedalLocomotion::ContinuousDynamicalSystem::MultiStateWeightProvider>
            weightProvider;
        int nodeNumber;
    };

    manif::SO3d calib_R_link = manif::SO3d::Identity();

    std::unordered_map<int, OrientationTaskStruct> m_OrientationTasks; /** unordered map of the
                                                                    orientation tasks */

    std::unordered_map<int, GravityTaskStruct> m_GravityTasks; /** unordered map of the gravity
                                                                    tasks */

    std::shared_ptr<iDynTree::KinDynComputations> m_kinDyn; /** pointer to the KinDynComputations
                                                               object */

    int m_nrDoFs; /** Number of Joint Degrees of Freedom */

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
     * | `SO3Task` |           `node`               |      `int`      |                    Node number of the task. The node number must be unique.             |  Yes |
     * | `SO3Task` |      `rotation_matrix`         | `vector<double>`|    Rotation matrix between the IMU and the link. By default it set to identity.         |  No  |
     * | `SO3Task` |         `frame_name`           |     `string`    |                          Name of the frame in which the task is expressed.              |  Yes |
     * | `SO3Task` |         `kp_angular`           |     `double`    |                        Value of the gain of the angular velocity feedback.              |  Yes |
     * `SO3Task` is a placeholder for the name of the task contained in the `tasks` list.
     * The "GravityTask" requires the following parameters:
     * |    Group    |         Parameter Name         |    Type    |                                         Description                                          | Mandatory |
     * |:-----------:|:------------------------------:|:----------:|:--------------------------------------------------------------------------------------------:|:---------:|
     * |`GravityTask`|           `type`               |  `string`  |                         Type of the task. The value to be set is `GravityTask`               |  Yes  |
     * |`GravityTask`| `robot_velocity_variable_name` |  `string`  |Name of the variable contained in `VariablesHandler` describing the generalized robot velocity|  Yes  |
     * |`GravityTask`|           `node`               |   `int`    |                    Node number of the task. The node number must be unique.                  |  Yes  |
     * |`GravityTask`|            `kp`                |  `double`  |                          Gain of the distance controller                                     |  Yes  |
     * |`GravityTask`|     `target_frame_name`        |  `string`  |                 Name of the frame to which apply the gravity task                            |  Yes  |
     * @note The following `ini` file presents an example of the configuration that can be used to
     * build the HumanIK class.
     *  ~~~~~{.ini}
     * tasks                           ("PELVIS_TASK")
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
     * rotation_matrix                 (0.0, 1.0, 0.0,
     *                                  0.0, 0.0, -1.0,
     *                                 -1.0, 0.0, 0.0)
    */
    // clang-format on
    bool
    initialize(std::weak_ptr<const BipedalLocomotion::ParametersHandler::IParametersHandler> handler,
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
     * set the orientation and the angular velocity of a given node
     * @param node node number
     * @param I_R_IMU orientation of the IMU
     * @param I_omega_IMU angular velocity of the IMU
     */
    bool setNodeSetPoint(const int node,
                         const manif::SO3d& I_R_IMU,
                         const manif::SO3Tangentd& I_omega_IMU = manif::SO3d::Tangent::Zero());

    bool TPoseCalibrationNode(const int node, const manif::SO3d& I_R_IMU);

    /**
     * advance the inverse kinematics solver
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
