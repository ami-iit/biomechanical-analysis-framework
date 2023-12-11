#include <BiomechanicalAnalysis/IK/InverseKinematics.h>
#include <iostream>

using namespace BiomechanicalAnalysis::IK;


bool HumanIK::initialize(std::weak_ptr<const BipedalLocomotion::ParametersHandler::IParametersHandler> handler,
                std::shared_ptr<iDynTree::KinDynComputations> kinDyn)
{
    constexpr std::size_t highPriority = 0;
    constexpr std::size_t lowPriority = 1;

    m_jointPositions.resize(kinDyn->getNrOfDegreesOfFreedom());
    m_jointVelocities.resize(kinDyn->getNrOfDegreesOfFreedom());

    std::cout << "nr of dofs = " << kinDyn->getNrOfDegreesOfFreedom() << std::endl;

    auto ptr = handler.lock();
    if (ptr == nullptr)
    {
        std::cerr << "[HumanIK::initialize] Invalid parameter handler." << std::endl;
        return false;
    }

    bool ok = m_qpIK.initialize(ptr->getGroup("IK"));
    auto group = ptr->getGroup("IK").lock();
    std::string variable;
    group->getParameter("robot_velocity_variable_name", variable);
    m_variableHandler.addVariable(variable, kinDyn->getNrOfDegreesOfFreedom() + 6);

    m_PelvisTask = std::make_shared<BipedalLocomotion::IK::SO3Task>();
    ok = ok && m_PelvisTask->setKinDyn(kinDyn);
    ok = ok && m_PelvisTask->initialize(ptr->getGroup("PELVIS_TASK"));
    ok = ok && m_qpIK.addTask(m_PelvisTask, "pelvis_task", highPriority);

    m_T8Task = std::make_shared<BipedalLocomotion::IK::SO3Task>();
    ok = ok && m_T8Task->setKinDyn(kinDyn);
    ok = ok && m_T8Task->initialize(ptr->getGroup("T8_TASK"));
    ok = ok && m_qpIK.addTask(m_T8Task, "t8_task", highPriority);

    m_RightUpperArmTask = std::make_shared<BipedalLocomotion::IK::SO3Task>();
    ok = ok && m_RightUpperArmTask->setKinDyn(kinDyn);
    ok = ok && m_RightUpperArmTask->initialize(ptr->getGroup("RIGHT_UPPER_ARM_TASK"));
    ok = ok && m_qpIK.addTask(m_RightUpperArmTask, "right_upper_arm_task", lowPriority);

    m_RightForeArmTask = std::make_shared<BipedalLocomotion::IK::SO3Task>();
    ok = ok && m_RightForeArmTask->setKinDyn(kinDyn);
    ok = ok && m_RightForeArmTask->initialize(ptr->getGroup("RIGHT_FORE_ARM_TASK"));
    ok = ok && m_qpIK.addTask(m_RightForeArmTask, "right_fore_arm_task", lowPriority);

    m_LeftUpperArmTask = std::make_shared<BipedalLocomotion::IK::SO3Task>();
    ok = ok && m_LeftUpperArmTask->setKinDyn(kinDyn);
    ok = ok && m_LeftUpperArmTask->initialize(ptr->getGroup("LEFT_UPPER_ARM_TASK"));
    ok = ok && m_qpIK.addTask(m_LeftUpperArmTask, "left_upper_arm_task", lowPriority);

    m_LeftForeArmTask = std::make_shared<BipedalLocomotion::IK::SO3Task>();
    ok = ok && m_LeftForeArmTask->setKinDyn(kinDyn);
    ok = ok && m_LeftForeArmTask->initialize(ptr->getGroup("LEFT_FORE_ARM_TASK"));
    ok = ok && m_qpIK.addTask(m_LeftForeArmTask, "left_fore_arm_task", lowPriority);

    m_RightUpperLegTask = std::make_shared<BipedalLocomotion::IK::SO3Task>();
    ok = ok && m_RightUpperLegTask->setKinDyn(kinDyn);
    ok = ok && m_RightUpperLegTask->initialize(ptr->getGroup("RIGHT_UPPER_LEG_TASK"));
    ok = ok && m_qpIK.addTask(m_RightUpperLegTask, "right_upper_leg_task", lowPriority);

    m_RightLowerLegTask = std::make_shared<BipedalLocomotion::IK::SO3Task>();
    ok = ok && m_RightLowerLegTask->setKinDyn(kinDyn);
    ok = ok && m_RightLowerLegTask->initialize(ptr->getGroup("RIGHT_LOWER_LEG_TASK"));
    ok = ok && m_qpIK.addTask(m_RightLowerLegTask, "right_lower_leg_task", lowPriority);

    m_LeftUpperLegTask = std::make_shared<BipedalLocomotion::IK::SO3Task>();
    ok = ok && m_LeftUpperLegTask->setKinDyn(kinDyn);
    ok = ok && m_LeftUpperLegTask->initialize(ptr->getGroup("LEFT_UPPER_LEG_TASK"));
    ok = ok && m_qpIK.addTask(m_LeftUpperLegTask, "left_upper_leg_task", lowPriority);

    m_LeftLowerLegTask = std::make_shared<BipedalLocomotion::IK::SO3Task>();
    ok = ok && m_LeftLowerLegTask->setKinDyn(kinDyn);
    ok = ok && m_LeftLowerLegTask->initialize(ptr->getGroup("LEFT_LOWER_LEG_TASK"));
    ok = ok && m_qpIK.addTask(m_LeftLowerLegTask, "left_lower_leg_task", lowPriority);

    m_qpIK.finalize(m_variableHandler);

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

bool HumanIK::setPelvisSetPoint(const manif::SO3d &pelvisOrientation,
                                           const manif::SO3Tangentd &pelvisAngularVelocity)
{
    m_PelvisTask->setSetPoint(pelvisOrientation, pelvisAngularVelocity);

    return true;
}

bool HumanIK::setT8SetPoint(const manif::SO3d &T8Orientation,
                                        const manif::SO3Tangentd &T8AngularVelocity)
{
    m_T8Task->setSetPoint(T8Orientation, T8AngularVelocity);

    return true;
}

bool HumanIK::setRightUpperArmSetPoint(const manif::SO3d &rightUpperArmOrientation,
                                        const manif::SO3Tangentd &rightUpperArmAngularVelocity)
{
    m_RightUpperArmTask->setSetPoint(rightUpperArmOrientation, rightUpperArmAngularVelocity);

    return true;
}

bool HumanIK::setRightForeArmSetPoint(const manif::SO3d &rightForeArmOrientation,
                                        const manif::SO3Tangentd &rightForeArmAngularVelocity)
{
    m_RightForeArmTask->setSetPoint(rightForeArmOrientation, rightForeArmAngularVelocity);

    return true;
}

bool HumanIK::setLeftUpperArmSetPoint(const manif::SO3d &leftUpperArmOrientation,
                                        const manif::SO3Tangentd &leftUpperArmAngularVelocity)
{
    m_LeftUpperArmTask->setSetPoint(leftUpperArmOrientation, leftUpperArmAngularVelocity);

    return true;
}

bool HumanIK::setLeftForeArmSetPoint(const manif::SO3d &leftForeArmOrientation,
                                        const manif::SO3Tangentd &leftForeArmAngularVelocity)
{
    m_LeftForeArmTask->setSetPoint(leftForeArmOrientation, leftForeArmAngularVelocity);

    return true;
}

bool HumanIK::setRightUpperLegSetPoint(const manif::SO3d &rightUpperLegOrientation,
                                        const manif::SO3Tangentd &rightUpperLegAngularVelocity)
{
    m_RightUpperLegTask->setSetPoint(rightUpperLegOrientation, rightUpperLegAngularVelocity);

    return true;
}

bool HumanIK::setRightLowerLegSetPoint(const manif::SO3d &rightLowerLegOrientation,
                                        const manif::SO3Tangentd &rightLowerLegAngularVelocity)
{
    m_RightLowerLegTask->setSetPoint(rightLowerLegOrientation, rightLowerLegAngularVelocity);

    return true;
}

bool HumanIK::setLeftUpperLegSetPoint(const manif::SO3d &leftUpperLegOrientation,
                                        const manif::SO3Tangentd &leftUpperLegAngularVelocity)
{
    m_LeftUpperLegTask->setSetPoint(leftUpperLegOrientation, leftUpperLegAngularVelocity);

    return true;
}

bool HumanIK::setLeftLowerLegSetPoint(const manif::SO3d &leftLowerLegOrientation,
                                        const manif::SO3Tangentd &leftLowerLegAngularVelocity)
{
    m_LeftLowerLegTask->setSetPoint(leftLowerLegOrientation, leftLowerLegAngularVelocity);

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
