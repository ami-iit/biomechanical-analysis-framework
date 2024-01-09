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

class HumanIK
{
private:
    // Integration time step in nanoseconds
    std::chrono::nanoseconds m_dtIntegration;

    // Struct to integrate the base and joint velocities
    struct System
    {
        std::shared_ptr<BipedalLocomotion::ContinuousDynamicalSystem::ForwardEuler<BipedalLocomotion::ContinuousDynamicalSystem::FloatingBaseSystemKinematics>> integrator;
        std::shared_ptr<BipedalLocomotion::ContinuousDynamicalSystem::FloatingBaseSystemKinematics> dynamics;
    };

    // System to integrate the base and joint velocities
    System m_system;    

    // Joint positions and velocities
    Eigen::VectorXd m_jointPositions;
    Eigen::VectorXd m_jointVelocities;
    Eigen::Matrix4d m_basePose;
    Eigen::Matrix<double, 6, 1> m_baseVelocity;
    Eigen::Vector3d m_gravity;

    // tasks
    std::shared_ptr<BipedalLocomotion::IK::SO3Task> m_link1OrientationTask;

    // pointer to the KinDynComputations object
    std::shared_ptr<iDynTree::KinDynComputations> m_kinDyn;

    // Number of Joint Degrees of Freedom
    int m_nrDoFs;

    BipedalLocomotion::IK::QPInverseKinematics m_qpIK;
    BipedalLocomotion::System::VariablesHandler m_variableHandler;

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
     * set the number of DoFs
     * @param nrDoFs number of DoFs
     * @return true if the number of DoFs is set correctly
    */
    bool setDoFsNumber(const int nrDoFs);

    /**
     * get the number of DoFs
     * @return number of DoFs
    */
    int getDoFsNumber() const;

    /**
     * set the initial joint positions
     * @param qInitial initial joint positions
     * @return true if the initial joint positions are set correctly
    */
    bool setInitialJointPositions(const Eigen::Ref<const Eigen::VectorXd> qInitial);

    /**
     * set the orientation and angular velocity for the link 1 task
     * @param link1Orientation link 1 orientation
     * @param link1AngularVelocity link 1 angular velocity
     * @return true if the link 1 orientation and angular velocity are set correctly
    */
    bool setLink1OrientationAndAngVel(const manif::SO3d &link1Orientation,
                                      const manif::SO3Tangentd &link1AngularVelocity);

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
