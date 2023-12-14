#include <BiomechanicalAnalysis/IK/InverseKinematics.h>
#include <BiomechanicalAnalysis/Logging/Logger.h>
#include <iostream>

using namespace BiomechanicalAnalysis::IK;


bool HumanIK::initialize(std::weak_ptr<const BipedalLocomotion::ParametersHandler::IParametersHandler> handler,
                std::shared_ptr<iDynTree::KinDynComputations> kinDyn)
{
    constexpr std::size_t highPriority = 0;
    constexpr std::size_t lowPriority = 1;
    constexpr auto logPrefix = "[HumanIK::initialize]";

    m_jointPositions.resize(kinDyn->getNrOfDegreesOfFreedom());
    m_jointVelocities.resize(kinDyn->getNrOfDegreesOfFreedom());

    auto ptr = handler.lock();
    if (ptr == nullptr)
    {
        BiomechanicalAnalysis::log()->error("{} parameters handler is a null pointer.", logPrefix);
        return false;
    }

    bool ok = m_qpIK.initialize(ptr->getGroup("IK"));
    auto group = ptr->getGroup("IK").lock();
    std::string variable;
    group->getParameter("robot_velocity_variable_name", variable);
    m_variableHandler.addVariable(variable, kinDyn->getNrOfDegreesOfFreedom() + 6);

    m_link1OrientationTask = std::make_shared<BipedalLocomotion::IK::SO3Task>();
    ok = ok && m_link1OrientationTask->setKinDyn(kinDyn);
    ok = ok && m_link1OrientationTask->initialize(ptr->getGroup("LINK1_TASK"));
    ok = ok && m_qpIK.addTask(m_link1OrientationTask, "link1_task", highPriority);

    m_qpIK.finalize(m_variableHandler);

    return ok;
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

bool HumanIK::setLink1OrientationAndAngVel(const manif::SO3d &link1Orientation,
                                           const manif::SO3Tangentd &link1AngularVelocity)
{
    m_link1OrientationTask->setSetPoint(link1Orientation, link1AngularVelocity);

    return true;
}

bool HumanIK::advance()
{
    bool ok{true};
    ok = ok && m_qpIK.advance();
    ok = ok && m_qpIK.isOutputValid();

    if(ok)
    {
        m_jointVelocities = m_qpIK.getOutput().jointVelocity;
        m_baseVelocity = m_qpIK.getOutput().baseVelocity;
    }

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

bool HumanIK::getBasePosition(Eigen::Ref<Eigen::Vector3d> basePosition) const
{
    basePosition = m_basePosition;

    return true;
}

bool HumanIK::getBaseVelocity(manif::SE3Tangentd &baseVelocity) const
{
    baseVelocity = m_baseVelocity;

    return true;
}

bool HumanIK::getBaseOrientation(manif::SO3d& baseOrientation) const
{
    baseOrientation = m_baseOrientation;

    return true;
}

bool HumanIK::getBaseAngularVelocity(Eigen::Ref<Eigen::Vector3d> baseAngularVelocity) const
{
    baseAngularVelocity = m_baseAngularVelocity;

    return true;
}
