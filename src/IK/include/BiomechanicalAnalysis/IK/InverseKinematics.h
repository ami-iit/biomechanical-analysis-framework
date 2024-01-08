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

    manif::SO3d I_R_link_manif;
    manif::SO3Tangentd I_omega_link_manif;

    iDynTree::Rotation I_R_link;
    iDynTree::AngVelocity I_omega_link;

    struct OrientationTask
    {
        /* data */
        std::shared_ptr<BipedalLocomotion::IK::SO3Task> task;
        int nodeNumber;
        iDynTree::Rotation IMU_R_link = iDynTree::Rotation::Identity();
    };

    // tasks
    OrientationTask m_PelvisTask;
    OrientationTask m_T8Task;
    OrientationTask m_RightUpperArmTask;
    OrientationTask m_RightForeArmTask;
    OrientationTask m_LeftUpperArmTask;
    OrientationTask m_LeftForeArmTask;
    OrientationTask m_RightUpperLegTask;
    OrientationTask m_RightLowerLegTask;
    OrientationTask m_LeftUpperLegTask;
    OrientationTask m_LeftLowerLegTask;

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
     * get the number of DoFs
    */
    int getDoFsNumber() const;

    /**
     * set the initial joint positions
    */
    bool setInitialJointPositions(const Eigen::Ref<const Eigen::VectorXd> qInitial);

    bool setNodeSetPoint(int node,const iDynTree::Rotation &I_R_IMU,
                                           const iDynTree::AngVelocity &I_omega_IMU);

    // set the set point for the orientation tasks
    bool setPelvisSetPoint(const manif::SO3d &pelvisOrientation,
                                      const manif::SO3Tangentd &pelvisAngularVelocity);

    bool setT8SetPoint(const manif::SO3d &T8Orientation,
                                      const manif::SO3Tangentd &T8AngularVelocity);

    bool setRightUpperArmSetPoint(const manif::SO3d &RightUpperArmOrientation,
                                        const manif::SO3Tangentd &RightUpperArmAngularVelocity);

    bool setRightForeArmSetPoint(const manif::SO3d &RightForeArmOrientation,
                                        const manif::SO3Tangentd &RightForeArmAngularVelocity);

    bool setLeftUpperArmSetPoint(const manif::SO3d &LeftUpperArmOrientation,
                                        const manif::SO3Tangentd &LeftUpperArmAngularVelocity);

    bool setLeftForeArmSetPoint(const manif::SO3d &LeftForeArmOrientation,
                                        const manif::SO3Tangentd &LeftForeArmAngularVelocity);

    bool setRightUpperLegSetPoint(const manif::SO3d &RightUpperLegOrientation,
                                        const manif::SO3Tangentd &RightUpperLegAngularVelocity);

    bool setRightLowerLegSetPoint(const manif::SO3d &RightLowerLegOrientation,
                                        const manif::SO3Tangentd &RightLowerLegAngularVelocity);

    bool setLeftUpperLegSetPoint(const manif::SO3d &LeftUpperLegOrientation,
                                        const manif::SO3Tangentd &LeftUpperLegAngularVelocity);

    bool setLeftLowerLegSetPoint(const manif::SO3d &LeftLowerLegOrientation,
                                        const manif::SO3Tangentd &LeftLowerLegAngularVelocity);

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
