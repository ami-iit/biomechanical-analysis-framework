/**
 * @file InverseKinematic.h
 * @authors Davide Gorbani <davide.gorbani@iit.it>
 */

#ifndef BIOMECHANICAL_ANALYSIS_INVERSE_KINEMATIC_H
#define BIOMECHANICAL_ANALYSIS_INVERSE_KINEMATIC_H

#include <BipedalLocomotion/IK/IKLinearTask.h>

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
    Eigen::VectorXd m_q;
    Eigen::VectorXd m_qDot;

    // Number of Joint Degrees of Freedom
    int m_nrDoFs;

public:
    HumanIK(){}; // constructor
    ~HumanIK(){}; // destructor

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

    // set the state of the system
    bool setState();

    // compute the next state
    bool advance();

    // get the joint poistions
    bool getJointPositions(Eigen::Ref<Eigen::VectorXd> q) const;

    // get the joint velocities
    bool getJointVelocities(Eigen::Ref<Eigen::VectorXd> qDot) const;
};

} // namespace IK
} // namespace BiomechanicalAnalysis


#endif // BIOMECHANICAL_ANALYSIS_INVERSE_KINEMATIC_H
