#include <BiomechanicalAnalysis/IK/InverseKinematic.h>
#include <iostream>

using namespace BiomechanicalAnalysis::IK;


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
    m_q.resize(m_nrDoFs);
    m_qDot.resize(m_nrDoFs);

    return true;
}

int HumanIK::getDoFsNumber() const
{
    return m_nrDoFs;
}

bool HumanIK::setInitialJointPositions(const Eigen::Ref<const Eigen::VectorXd> qInitial)
{
    m_q = qInitial;

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
    // TODO implement me
    m_qDot.setConstant(1.0);

    // integrate the joint velocities
    m_q += m_dtIntegration * m_qDot;

    return true;
}

bool HumanIK::getJointPositions(Eigen::Ref<Eigen::VectorXd> q) const
{
    q = m_q;

    return true;
}

bool HumanIK::getJointVelocities(Eigen::Ref<Eigen::VectorXd> qDot) const
{
    qDot = m_qDot;

    return true;
}
