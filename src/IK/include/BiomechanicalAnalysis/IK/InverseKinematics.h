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

namespace BiomechanicalAnalysis
{

namespace IK
{

class HumanIK
{
private:
    // Integration time step
    double m_dtIntegration;

    // Joint positions and velocities
    Eigen::VectorXd m_jointPositions;
    Eigen::VectorXd m_jointVelocities;
    Eigen::Vector3d m_basePosition;
    manif::SE3Tangentd m_baseVelocity;
    manif::SO3d m_baseOrientation;
    Eigen::Vector3d m_baseAngularVelocity;

    // tasks
    std::shared_ptr<BipedalLocomotion::IK::SO3Task> m_link1OrientationTask;

    // Number of Joint Degrees of Freedom
    int m_nrDoFs;

    BipedalLocomotion::IK::QPInverseKinematics m_qpIK;
    BipedalLocomotion::System::VariablesHandler m_variableHandler;

public:
    HumanIK(){}; // constructor
    ~HumanIK(){}; // destructor

    // initialize all the task and the inverse kinematics solver
    bool initialize(std::weak_ptr<const BipedalLocomotion::ParametersHandler::IParametersHandler> handler,
                std::shared_ptr<iDynTree::KinDynComputations> kinDyn);

    // set the integration time step
    bool setDt(const double dt);

    // get the integration time step
    double getDt() const;

    // set the number of DoFs
    bool setDoFsNumber(const int nrDoFs);

    // get the number of DoFs
    int getDoFsNumber() const;

    // set the initial joint positions
    bool setInitialJointPositions(const Eigen::Ref<const Eigen::VectorXd> qInitial);

    bool setLink1OrientationAndAngVel(const manif::SO3d &link1Orientation,
                                      const manif::SO3Tangentd &link1AngularVelocity);

    // compute the next state
    bool advance();

    // get the joint poistions
    bool getJointPositions(Eigen::Ref<Eigen::VectorXd> jointPositions) const;

    // get the joint velocities
    bool getJointVelocities(Eigen::Ref<Eigen::VectorXd> jointVelocities) const;

    // get the base position
    bool getBasePosition(Eigen::Ref<Eigen::Vector3d> basePosition) const;

    // get the base velocity
    bool getBaseVelocity(manif::SE3Tangentd & baseVelocity) const;

    // get the base orientation
    bool getBaseOrientation(manif::SO3d& baseOrientation) const;

    // get the base angular velocity
    bool getBaseAngularVelocity(Eigen::Ref<Eigen::Vector3d> baseAngularVelocity) const;
};

} // namespace IK
} // namespace BiomechanicalAnalysis


#endif // BIOMECHANICAL_ANALYSIS_INVERSE_KINEMATIC_H