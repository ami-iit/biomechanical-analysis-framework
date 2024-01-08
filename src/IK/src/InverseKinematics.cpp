#include <BiomechanicalAnalysis/IK/InverseKinematics.h>
#include <BiomechanicalAnalysis/Logging/Logger.h>
#include <BipedalLocomotion/Conversions/ManifConversions.h>
#include <iostream>

using namespace BiomechanicalAnalysis::IK;
using namespace BipedalLocomotion::ContinuousDynamicalSystem;
using namespace BipedalLocomotion::Conversions;
using namespace std::chrono_literals;

bool HumanIK::initialize(std::weak_ptr<const BipedalLocomotion::ParametersHandler::IParametersHandler> handler,
                std::shared_ptr<iDynTree::KinDynComputations> kinDyn)
{
    constexpr std::size_t highPriority = 0;
    constexpr std::size_t lowPriority = 1;
    constexpr auto logPrefix = "[HumanIK::initialize]";

    if ((kinDyn == nullptr) || (!kinDyn->isValid()))
    {
        BiomechanicalAnalysis::log()->error("{} Invalid kinDyn object.", logPrefix);
        return false;
    }

    m_kinDyn = kinDyn;

    m_jointPositions.resize(m_kinDyn->getNrOfDegreesOfFreedom());
    m_jointVelocities.resize(m_kinDyn->getNrOfDegreesOfFreedom());

    kinDyn->getRobotState(m_basePose, m_jointPositions, m_baseVelocity, m_jointVelocities, m_gravity);

    m_system.dynamics = std::make_shared<FloatingBaseSystemKinematics>();
    m_system.dynamics->setState({m_basePose.topRightCorner<3, 1>(), toManifRot(m_basePose.topLeftCorner<3,3>()) , m_jointPositions});

    m_system.integrator = std::make_shared<ForwardEuler<FloatingBaseSystemKinematics>>();
    m_system.integrator->setDynamicalSystem(m_system.dynamics);

    auto ptr = handler.lock();
    if (ptr == nullptr)
    {
        BiomechanicalAnalysis::log()->error("{} Invalid parameters handler.", logPrefix);
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
    dt;
    m_dtIntegration = std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::duration<double>(dt));

    return m_system.integrator->setIntegrationStep(m_dtIntegration);
}

double HumanIK::getDt() const
{
    return m_dtIntegration.count();
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

    if(!ok)
    {
        BiomechanicalAnalysis::log()->error("[HumanIK::advance] Error in the QP solver.");
        return false;
    }
    m_jointVelocities = m_qpIK.getOutput().jointVelocity;
    m_baseVelocity = m_qpIK.getOutput().baseVelocity.coeffs();

    ok = ok && m_system.dynamics->setControlInput({m_baseVelocity, m_jointVelocities});
    ok = ok && m_system.integrator->integrate(0s, m_dtIntegration);

    if(ok)
    {
        const auto& [basePosition, baseRotation, jointPosition]
                    = m_system.integrator->getSolution();
        m_basePose.topRightCorner<3, 1>() = basePosition;
        m_basePose.topLeftCorner<3, 3>() = baseRotation.rotation();
        m_jointPositions = jointPosition;
        m_kinDyn->setRobotState(m_basePose, jointPosition, m_baseVelocity, m_jointVelocities, m_gravity);
    }
    else
    {
        BiomechanicalAnalysis::log()->error("[HumanIK::advance] Error in the integration.");
    }

    return ok;
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
    basePosition = m_basePose.topRightCorner<3, 1>();

    return true;
}

bool HumanIK::getBaseLinearVelocity(Eigen::Ref<Eigen::Vector3d> baseVelocity) const
{
    baseVelocity = m_baseVelocity.topRows<3>();

    return true;
}

bool HumanIK::getBaseOrientation(Eigen::Ref<Eigen::Matrix3d> baseOrientation) const
{
    baseOrientation = m_basePose.topLeftCorner<3, 3>();

    return true;
}

bool HumanIK::getBaseAngularVelocity(Eigen::Ref<Eigen::Vector3d> baseAngularVelocity) const
{
    baseAngularVelocity = m_baseVelocity.bottomRows<3>();

    return true;
}
