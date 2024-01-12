/**
 * @file InverseKinematic.h
 * @authors Davide Gorbani <davide.gorbani@iit.it>
 */

#ifndef BIOMECHANICAL_ANALYSIS_INVERSE_KINEMATIC_H
#define BIOMECHANICAL_ANALYSIS_INVERSE_KINEMATIC_H

// iDynTree
#include <iDynTree/KinDynComputations.h>

// BipedalLocomotion
#include <BipedalLocomotion/IK/QPInverseKinematics.h>
#include <BipedalLocomotion/ParametersHandler/IParametersHandler.h>
#include <BipedalLocomotion/System/VariablesHandler.h>
#include <BipedalLocomotion/IK/SO3Task.h>
#include <BipedalLocomotion/ContinuousDynamicalSystem/FloatingBaseSystemKinematics.h>
#include <BipedalLocomotion/ContinuousDynamicalSystem/ForwardEuler.h>

namespace BiomechanicalAnalysis
{

namespace IK
{

/**
 * @brief HumanIK class is a class in which the inverse kinematics problem is solved.
*/
class HumanIK
{
private:

    std::chrono::nanoseconds m_dtIntegration; /** Integration time step in nanoseconds */

    /**
     * Struct containing the integrator and the dynamics
    */
    struct System
    {
        std::shared_ptr<BipedalLocomotion::ContinuousDynamicalSystem::ForwardEuler<BipedalLocomotion::ContinuousDynamicalSystem::FloatingBaseSystemKinematics>> integrator;
        std::shared_ptr<BipedalLocomotion::ContinuousDynamicalSystem::FloatingBaseSystemKinematics> dynamics;
    };

    System m_system; /** Struct containing the integrator and the dynamics */

    Eigen::VectorXd m_jointPositions; /** Position of the joints */
    Eigen::VectorXd m_jointVelocities; /** Velocity of the joints */
    Eigen::Matrix4d m_basePose; /** SO3 pose of the base */
    Eigen::Matrix<double, 6, 1> m_baseVelocity; /** Vector containing the linear and angular velocity of the base */
    Eigen::Vector3d m_gravity; /** Gravity vector */

    manif::SO3d I_R_link_manif; /** orientation of the link in the inertial frame */
    manif::SO3Tangentd I_omega_link_manif; /** angular velocity of the link in the inertial frame */

    iDynTree::Rotation I_R_link; /** orientation of the link in the inertial frame */
    iDynTree::AngVelocity I_omega_link; /** angular velocity of the link in the inertial frame */

    /**
     * Struct containing the orientation task, the node number and the rotation matrix between the IMU and the link
    */
    struct OrientationTask
    {
        std::shared_ptr<BipedalLocomotion::IK::SO3Task> task;
        int nodeNumber;
        iDynTree::Rotation IMU_R_link = iDynTree::Rotation::Identity();
    };

    std::unordered_map<int, OrientationTask> m_OrientationTasks; /** unordered map of the orientation tasks */

    std::shared_ptr<iDynTree::KinDynComputations> m_kinDyn; /** pointer to the KinDynComputations object */

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

    /**
     * initialize all the task and the inverse kinematics solver
     * @param handler pointer to the parameters handler
     * @param kinDyn pointer to the KinDynComputations object
     * @return true if all the tasks are initialized correctly
    */
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
     * set the orientation and the angular velocity of a given node
     * @param node node number
     * @param I_R_IMU orientation of the IMU
     * @param I_omega_IMU angular velocity of the IMU
    */
    bool setNodeSetPoint(int node,const iDynTree::Rotation &I_R_IMU,
                                           const iDynTree::AngVelocity &I_omega_IMU);

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
