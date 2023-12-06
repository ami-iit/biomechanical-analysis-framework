#include <BiomechanicalAnalysis/IK/InverseKinematics.h>
#include <iostream>

using namespace BiomechanicalAnalysis::IK;


bool HumanIK::initialize(std::weak_ptr<const BipedalLocomotion::ParametersHandler::IParametersHandler> handler,
                std::shared_ptr<iDynTree::KinDynComputations> kinDyn)
{
    // TODO implement me

    return true;
}

bool HumanIK::setDt(const double dt)
{
    m_dtIntegration = dt;

    return true;
}

double HumanIK::getDt() const
{
    return m_dtIntegration;
}

bool HumanIK::setDoFsNumber(const int nrDoFs)
{
    m_nrDoFs = nrDoFs;
    m_jointPositions.resize(m_nrDoFs);
    m_jointVelocities.resize(m_nrDoFs);

    return true;
}

int HumanIK::getDoFsNumber() const
{
    return m_nrDoFs;
}

bool HumanIK::setInitialJointPositions(const Eigen::Ref<const Eigen::VectorXd> qInitial)
{
    m_jointPositions = qInitial;

    return true;
}

bool HumanIK::setState()
{
    std::cout << "HumanIK::setState()" << std::endl;

    // TODO implement me

    return true;
}

bool HumanIK::advance()
{
    bool ok{true};
    ok = ok && m_qpIK.advance();
    ok = ok && m_qpIK.isOutputValid();

    // integrate the joint velocities
    m_jointPositions += m_dtIntegration * m_jointVelocities;

    return true;
}

bool HumanIK::getJointPositions(Eigen::Ref<Eigen::VectorXd> jointPositions) const
{
    jointPositions = m_jointPositions;

    return true;
}

bool HumanIK::getJointVelocities(Eigen::Ref<Eigen::VectorXd> jointVelocities) const
{
    jointVelocities = m_jointVelocities;

    return true;
}
