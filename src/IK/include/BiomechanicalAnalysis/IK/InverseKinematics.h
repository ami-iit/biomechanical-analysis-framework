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
    HumanIK(){}; // constructor
    ~HumanIK(){}; // destructor

    /**
     * initialize all the task and the inverse kinematics solver
    */
    bool initialize(std::weak_ptr<const BipedalLocomotion::ParametersHandler::IParametersHandler> handler,
                std::shared_ptr<iDynTree::KinDynComputations> kinDyn);

    /**
     * set the integration time step
    */
    bool setDt(const double dt);

    /**
     * get the integration time step
    */
    double getDt() const;

    /**
     * set the number of DoFs
    */
    bool setDoFsNumber(const int nrDoFs);

    /**
     * get the number of DoFs
    */
    int getDoFsNumber() const;

    /**
     * set the initial joint positions
    */
    bool setInitialJointPositions(const Eigen::Ref<const Eigen::VectorXd> qInitial);

    bool setLink1OrientationAndAngVel(const manif::SO3d &link1Orientation,
                                      const manif::SO3Tangentd &link1AngularVelocity);

    /**
     * advance the inverse kinematics solver
    */
    bool advance();

    /**
     * get the joint positions
    */
    bool getJointPositions(Eigen::Ref<Eigen::VectorXd> jointPositions) const;

    /**
     * get the joint velocities
    */
    bool getJointVelocities(Eigen::Ref<Eigen::VectorXd> jointVelocities) const;

    /**
     * get the base position
    */
    bool getBasePosition(Eigen::Ref<Eigen::Vector3d> basePosition) const;

    /**
     * get the base linear velocity
    */
    bool getBaseLinearVelocity(Eigen::Ref<Eigen::Vector3d> baseVelocity) const;

    /**
     * get the base orientation
    */
    bool getBaseOrientation(Eigen::Ref<Eigen::Matrix3d> baseOrientation) const;

    /**
     * get the base angular velocity
    */
    bool getBaseAngularVelocity(Eigen::Ref<Eigen::Vector3d> baseAngularVelocity) const;
};

} // namespace IK
} // namespace BiomechanicalAnalysis


#endif // BIOMECHANICAL_ANALYSIS_INVERSE_KINEMATIC_H
