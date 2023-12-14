#include <BiomechanicalAnalysis/IK/InverseKinematics.h>
#include <BiomechanicalAnalysis/Logging/Logger.h>
#include <iostream>
#include <iDynTree/EigenHelpers.h>

using namespace BiomechanicalAnalysis::IK;


bool HumanIK::initialize(std::weak_ptr<const BipedalLocomotion::ParametersHandler::IParametersHandler> handler,
                std::shared_ptr<iDynTree::KinDynComputations> kinDyn)
{
    constexpr std::size_t highPriority = 0;
    constexpr std::size_t lowPriority = 1;
    constexpr auto logPrefix = "[HumanIK::initialize]";

    Eigen::Vector3d Weight;
    Weight.setConstant(100.0);

    m_nrDoFs = kinDyn->getNrOfDegreesOfFreedom();

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

    m_PelvisTask.task = std::make_shared<BipedalLocomotion::IK::SO3Task>();
    ok = ok && m_PelvisTask.task->setKinDyn(kinDyn);
    ok = ok && m_PelvisTask.task->initialize(ptr->getGroup("PELVIS_TASK"));
    auto pelvisParam = ptr->getGroup("PELVIS_TASK").lock();
    if (!pelvisParam->getParameter("node_number", m_PelvisTask.nodeNumber))
    {
        std::cerr << "[baf] Parameter node_number of the PELVIS_TASK task is missing" << std::endl;
        return false;
    }
    std::cout << "node number pelvis: " << m_PelvisTask.nodeNumber << std::endl;
    m_PelvisTask.IMU_R_link = iDynTree::Rotation(0.0, 1.0, 0.0,
                                                0.0, 0.0, -1.0,
                                                -1.0, 0.0, 0.0);
    ok = ok && m_qpIK.addTask(m_PelvisTask.task, "pelvis_task", highPriority);

    m_T8Task.task = std::make_shared<BipedalLocomotion::IK::SO3Task>();
    ok = ok && m_T8Task.task->setKinDyn(kinDyn);
    ok = ok && m_T8Task.task->initialize(ptr->getGroup("T8_TASK"));
    auto T8Param = ptr->getGroup("T8_TASK").lock();
    if (!T8Param->getParameter("node_number", m_T8Task.nodeNumber))
    {
        std::cerr << "[baf] Parameter node_number of the T8_TASK task is missing" << std::endl;
        return false;
    }
    m_T8Task.IMU_R_link = iDynTree::Rotation(0.0, 1.0, 0.0,
                                                0.0, 0.0, 1.0,
                                                1.0, 0.0, 0.0);
    ok = ok && m_qpIK.addTask(m_T8Task.task, "T8_task", lowPriority, Weight);

    m_RightUpperArmTask.task = std::make_shared<BipedalLocomotion::IK::SO3Task>();
    ok = ok && m_RightUpperArmTask.task->setKinDyn(kinDyn);
    ok = ok && m_RightUpperArmTask.task->initialize(ptr->getGroup("RIGHT_UPPER_ARM_TASK"));
    auto rightUpperArmParam = ptr->getGroup("RIGHT_UPPER_ARM_TASK").lock();
    if (!rightUpperArmParam->getParameter("node_number", m_RightUpperArmTask.nodeNumber))
    {
        std::cerr << "[baf] Parameter node_number of the RIGHT_UPPER_ARM_TASK task is missing" << std::endl;
        return false;
    }
    ok = ok && m_qpIK.addTask(m_RightUpperArmTask.task, "right_upper_arm_task", lowPriority, Weight);

    m_RightForeArmTask.task = std::make_shared<BipedalLocomotion::IK::SO3Task>();
    ok = ok && m_RightForeArmTask.task->setKinDyn(kinDyn);
    ok = ok && m_RightForeArmTask.task->initialize(ptr->getGroup("RIGHT_FORE_ARM_TASK"));
    auto rightForeArmParam = ptr->getGroup("RIGHT_FORE_ARM_TASK").lock();
    if (!rightForeArmParam->getParameter("node_number", m_RightForeArmTask.nodeNumber))
    {
        std::cerr << "[baf] Parameter node_number of the RIGHT_FORE_ARM_TASK task is missing" << std::endl;
        return false;
    }
    ok = ok && m_qpIK.addTask(m_RightForeArmTask.task, "right_fore_arm_task", lowPriority, Weight);

    m_LeftUpperArmTask.task = std::make_shared<BipedalLocomotion::IK::SO3Task>();
    ok = ok && m_LeftUpperArmTask.task->setKinDyn(kinDyn);
    ok = ok && m_LeftUpperArmTask.task->initialize(ptr->getGroup("LEFT_UPPER_ARM_TASK"));
    auto leftUpperArmParam = ptr->getGroup("LEFT_UPPER_ARM_TASK").lock();
    if (!leftUpperArmParam->getParameter("node_number", m_LeftUpperArmTask.nodeNumber))
    {
        std::cerr << "[baf] Parameter node_number of the LEFT_UPPER_ARM_TASK task is missing" << std::endl;
        return false;
    }
    ok = ok && m_qpIK.addTask(m_LeftUpperArmTask.task, "left_upper_arm_task", lowPriority, Weight);

    m_LeftForeArmTask.task = std::make_shared<BipedalLocomotion::IK::SO3Task>();
    ok = ok && m_LeftForeArmTask.task->setKinDyn(kinDyn);
    ok = ok && m_LeftForeArmTask.task->initialize(ptr->getGroup("LEFT_FORE_ARM_TASK"));
    auto leftForeArmParam = ptr->getGroup("LEFT_FORE_ARM_TASK").lock();
    if (!leftForeArmParam->getParameter("node_number", m_LeftForeArmTask.nodeNumber))
    {
        std::cerr << "[baf] Parameter node_number of the LEFT_FORE_ARM_TASK task is missing" << std::endl;
        return false;
    }
    ok = ok && m_qpIK.addTask(m_LeftForeArmTask.task, "left_fore_arm_task", lowPriority, Weight);

    m_RightUpperLegTask.task = std::make_shared<BipedalLocomotion::IK::SO3Task>();
    ok = ok && m_RightUpperLegTask.task->setKinDyn(kinDyn);
    ok = ok && m_RightUpperLegTask.task->initialize(ptr->getGroup("RIGHT_UPPER_LEG_TASK"));
    auto rightUpperLegParam = ptr->getGroup("RIGHT_UPPER_LEG_TASK").lock();
    if (!rightUpperLegParam->getParameter("node_number", m_RightUpperLegTask.nodeNumber))
    {
        std::cerr << "[baf] Parameter node_number of the RIGHT_UPPER_LEG_TASK task is missing" << std::endl;
        return false;
    }
    m_RightUpperLegTask.IMU_R_link = iDynTree::Rotation(1.0, 0.0, 0.0,
                                                        0.0, 0.0, 1.0,
                                                        0.0, -1.0, 0.0);
    ok = ok && m_qpIK.addTask(m_RightUpperLegTask.task, "right_upper_leg_task", lowPriority, Weight);

    m_RightLowerLegTask.task = std::make_shared<BipedalLocomotion::IK::SO3Task>();
    ok = ok && m_RightLowerLegTask.task->setKinDyn(kinDyn);
    ok = ok && m_RightLowerLegTask.task->initialize(ptr->getGroup("RIGHT_LOWER_LEG_TASK"));
    auto rightLowerLegParam = ptr->getGroup("RIGHT_LOWER_LEG_TASK").lock();
    if (!rightLowerLegParam->getParameter("node_number", m_RightLowerLegTask.nodeNumber))
    {
        std::cerr << "[baf] Parameter node_number of the RIGHT_LOWER_LEG_TASK task is missing" << std::endl;
        return false;
    }
    std::cout << "node number right low leg: " << m_RightLowerLegTask.nodeNumber << std::endl;
    m_RightLowerLegTask.IMU_R_link = iDynTree::Rotation(1.0, 0.0, 0.0,
                                                        0.0, 0.0, 1.0,
                                                        0.0, -1.0, 0.0);
    ok = ok && m_qpIK.addTask(m_RightLowerLegTask.task, "right_lower_leg_task", lowPriority, Weight);

    m_LeftUpperLegTask.task = std::make_shared<BipedalLocomotion::IK::SO3Task>();
    ok = ok && m_LeftUpperLegTask.task->setKinDyn(kinDyn);
    ok = ok && m_LeftUpperLegTask.task->initialize(ptr->getGroup("LEFT_UPPER_LEG_TASK"));
    auto leftUpperLegParam = ptr->getGroup("LEFT_UPPER_LEG_TASK").lock();
    if (!leftUpperLegParam->getParameter("node_number", m_LeftUpperLegTask.nodeNumber))
    {
        std::cerr << "[baf] Parameter node_number of the LEFT_UPPER_LEG_TASK task is missing" << std::endl;
        return false;
    }
    m_LeftUpperLegTask.IMU_R_link = iDynTree::Rotation(1.0, 0.0, 0.0,
                                                        0.0, 0.0, -1.0,
                                                        0.0, 1.0, 0.0);
    ok = ok && m_qpIK.addTask(m_LeftUpperLegTask.task, "left_upper_leg_task", lowPriority, Weight);

    m_LeftLowerLegTask.task = std::make_shared<BipedalLocomotion::IK::SO3Task>();
    ok = ok && m_LeftLowerLegTask.task->setKinDyn(kinDyn);
    ok = ok && m_LeftLowerLegTask.task->initialize(ptr->getGroup("LEFT_LOWER_LEG_TASK"));
    auto leftLowerLegParam = ptr->getGroup("LEFT_LOWER_LEG_TASK").lock();
    if (!leftLowerLegParam->getParameter("node_number", m_LeftLowerLegTask.nodeNumber))
    {
        std::cerr << "[baf] Parameter node_number of the LEFT_LOWER_LEG_TASK task is missing" << std::endl;
        return false;
    }
    m_LeftLowerLegTask.IMU_R_link = iDynTree::Rotation(1.0, 0.0, 0.0,
                                                        0.0, 0.0, -1.0,
                                                        0.0, 1.0, 0.0);
    ok = ok && m_qpIK.addTask(m_LeftLowerLegTask.task, "left_lower_leg_task", lowPriority, Weight);

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

int HumanIK::getDoFsNumber() const
{
    return m_nrDoFs;
}

bool HumanIK::setInitialJointPositions(const Eigen::Ref<const Eigen::VectorXd> qInitial)
{
    m_jointPositions = qInitial;

    return true;
}

bool HumanIK::setNodeSetPoint(int node,const iDynTree::Rotation &I_R_IMU,
                                           const iDynTree::AngVelocity &I_omega_IMU)
{
    bool ok;
    if (node == m_PelvisTask.nodeNumber)
    {
        I_R_link = I_R_IMU * m_PelvisTask.IMU_R_link;
        iDynTree::toEigen(I_omega_link) = iDynTree::toEigen(I_omega_IMU).transpose() * iDynTree::toEigen(m_PelvisTask.IMU_R_link);
        I_R_link_manif = manif::SO3d(I_R_link.asQuaternion()(1), I_R_link.asQuaternion()(2), I_R_link.asQuaternion()(3), I_R_link.asQuaternion()(0));
        I_omega_link_manif = manif::SO3Tangentd(iDynTree::toEigen(I_omega_link));
        ok = m_PelvisTask.task->setSetPoint(I_R_link_manif, I_omega_link_manif);
        return ok;
    }
    else if (node == m_T8Task.nodeNumber)
    {
        I_R_link = I_R_IMU * m_T8Task.IMU_R_link;
        iDynTree::toEigen(I_omega_link) = iDynTree::toEigen(I_omega_IMU).transpose() * iDynTree::toEigen(m_T8Task.IMU_R_link);
        I_R_link_manif = manif::SO3d(I_R_link.asQuaternion()(1), I_R_link.asQuaternion()(2), I_R_link.asQuaternion()(3), I_R_link.asQuaternion()(0));
        I_omega_link_manif = manif::SO3Tangentd(iDynTree::toEigen(I_omega_link));
        ok = m_T8Task.task->setSetPoint(I_R_link_manif, I_omega_link_manif);
        return ok;
    }
    else if (node == m_RightUpperArmTask.nodeNumber)
    {
        I_R_link = I_R_IMU * m_RightUpperArmTask.IMU_R_link;
        iDynTree::toEigen(I_omega_link) = iDynTree::toEigen(I_omega_IMU).transpose() * iDynTree::toEigen(m_RightUpperArmTask.IMU_R_link);
        I_R_link_manif = manif::SO3d(I_R_link.asQuaternion()(1), I_R_link.asQuaternion()(2), I_R_link.asQuaternion()(3), I_R_link.asQuaternion()(0));
        I_omega_link_manif = manif::SO3Tangentd(iDynTree::toEigen(I_omega_link));
        ok = m_RightUpperArmTask.task->setSetPoint(I_R_link_manif, I_omega_link_manif);
        return ok;
    }
    else if (node == m_RightForeArmTask.nodeNumber)
    {
        I_R_link = I_R_IMU * m_RightForeArmTask.IMU_R_link;
        iDynTree::toEigen(I_omega_link) = iDynTree::toEigen(I_omega_IMU).transpose() * iDynTree::toEigen(m_RightForeArmTask.IMU_R_link);
        I_R_link_manif = manif::SO3d(I_R_link.asQuaternion()(1), I_R_link.asQuaternion()(2), I_R_link.asQuaternion()(3), I_R_link.asQuaternion()(0));
        I_omega_link_manif = manif::SO3Tangentd(iDynTree::toEigen(I_omega_link));
        ok = m_RightForeArmTask.task->setSetPoint(I_R_link_manif, I_omega_link_manif);
        return ok;
    }
    else if (node == m_LeftUpperArmTask.nodeNumber)
    {
        I_R_link = I_R_IMU * m_LeftUpperArmTask.IMU_R_link;
        iDynTree::toEigen(I_omega_link) = iDynTree::toEigen(I_omega_IMU).transpose() * iDynTree::toEigen(m_LeftUpperArmTask.IMU_R_link);
        I_R_link_manif = manif::SO3d(I_R_link.asQuaternion()(1), I_R_link.asQuaternion()(2), I_R_link.asQuaternion()(3), I_R_link.asQuaternion()(0));
        I_omega_link_manif = manif::SO3Tangentd(iDynTree::toEigen(I_omega_link));
        ok = m_LeftUpperArmTask.task->setSetPoint(I_R_link_manif, I_omega_link_manif);
        return ok;
    }
    else if (node == m_LeftForeArmTask.nodeNumber)
    {
        I_R_link = I_R_IMU * m_LeftForeArmTask.IMU_R_link;
        iDynTree::toEigen(I_omega_link) = iDynTree::toEigen(I_omega_IMU).transpose() * iDynTree::toEigen(m_LeftForeArmTask.IMU_R_link);
        I_R_link_manif = manif::SO3d(I_R_link.asQuaternion()(1), I_R_link.asQuaternion()(2), I_R_link.asQuaternion()(3), I_R_link.asQuaternion()(0));
        I_omega_link_manif = manif::SO3Tangentd(iDynTree::toEigen(I_omega_link));
        ok = m_LeftForeArmTask.task->setSetPoint(I_R_link_manif, I_omega_link_manif);
        return ok;
    }
    else if (node == m_RightUpperLegTask.nodeNumber)
    {
        I_R_link = I_R_IMU * m_RightUpperLegTask.IMU_R_link;
        iDynTree::toEigen(I_omega_link) = iDynTree::toEigen(I_omega_IMU).transpose() * iDynTree::toEigen(m_RightUpperLegTask.IMU_R_link);
        I_R_link_manif = manif::SO3d(I_R_link.asQuaternion()(1), I_R_link.asQuaternion()(2), I_R_link.asQuaternion()(3), I_R_link.asQuaternion()(0));
        I_omega_link_manif = manif::SO3Tangentd(iDynTree::toEigen(I_omega_link));
        ok = m_RightUpperLegTask.task->setSetPoint(I_R_link_manif, I_omega_link_manif);
        return ok;
    }
    else if (node == m_RightLowerLegTask.nodeNumber)
    {
        I_R_link = I_R_IMU * m_RightLowerLegTask.IMU_R_link;
        iDynTree::toEigen(I_omega_link) = iDynTree::toEigen(I_omega_IMU).transpose() * iDynTree::toEigen(m_RightLowerLegTask.IMU_R_link);
        I_R_link_manif = manif::SO3d(I_R_link.asQuaternion()(1), I_R_link.asQuaternion()(2), I_R_link.asQuaternion()(3), I_R_link.asQuaternion()(0));
        I_omega_link_manif = manif::SO3Tangentd(iDynTree::toEigen(I_omega_link));
        ok = m_RightLowerLegTask.task->setSetPoint(I_R_link_manif, I_omega_link_manif);
        return ok;
    }
    else if (node == m_LeftUpperLegTask.nodeNumber)
    {
        I_R_link = I_R_IMU * m_LeftUpperLegTask.IMU_R_link;
        iDynTree::toEigen(I_omega_link) = iDynTree::toEigen(I_omega_IMU).transpose() * iDynTree::toEigen(m_LeftUpperLegTask.IMU_R_link);
        I_R_link_manif = manif::SO3d(I_R_link.asQuaternion()(1), I_R_link.asQuaternion()(2), I_R_link.asQuaternion()(3), I_R_link.asQuaternion()(0));
        I_omega_link_manif = manif::SO3Tangentd(iDynTree::toEigen(I_omega_link));
        ok = m_LeftUpperLegTask.task->setSetPoint(I_R_link_manif, I_omega_link_manif);
        return ok;
    }
    else if (node == m_LeftLowerLegTask.nodeNumber)
    {
        I_R_link = I_R_IMU * m_LeftLowerLegTask.IMU_R_link;
        iDynTree::toEigen(I_omega_link) = iDynTree::toEigen(I_omega_IMU).transpose() * iDynTree::toEigen(m_LeftLowerLegTask.IMU_R_link);
        I_R_link_manif = manif::SO3d(I_R_link.asQuaternion()(1), I_R_link.asQuaternion()(2), I_R_link.asQuaternion()(3), I_R_link.asQuaternion()(0));
        I_omega_link_manif = manif::SO3Tangentd(iDynTree::toEigen(I_omega_link));
        ok = m_LeftLowerLegTask.task->setSetPoint(I_R_link_manif, I_omega_link_manif);
        return ok;
    }
    else
    {
        std::cerr << "[error] Node number " << node << " is not valid" << std::endl;
        return false;
    }
    
    return ok;
}

bool HumanIK::setPelvisSetPoint(const manif::SO3d &pelvisOrientation,
                                           const manif::SO3Tangentd &pelvisAngularVelocity)
{
    m_PelvisTask.task->setSetPoint(pelvisOrientation, pelvisAngularVelocity);

    return true;
}

bool HumanIK::setT8SetPoint(const manif::SO3d &T8Orientation,
                                        const manif::SO3Tangentd &T8AngularVelocity)
{
    m_T8Task.task->setSetPoint(T8Orientation, T8AngularVelocity);

    return true;
}

bool HumanIK::setRightUpperArmSetPoint(const manif::SO3d &rightUpperArmOrientation,
                                        const manif::SO3Tangentd &rightUpperArmAngularVelocity)
{
    m_RightUpperArmTask.task->setSetPoint(rightUpperArmOrientation, rightUpperArmAngularVelocity);

    return true;
}

bool HumanIK::setRightForeArmSetPoint(const manif::SO3d &rightForeArmOrientation,
                                        const manif::SO3Tangentd &rightForeArmAngularVelocity)
{
    m_RightForeArmTask.task->setSetPoint(rightForeArmOrientation, rightForeArmAngularVelocity);

    return true;
}

bool HumanIK::setLeftUpperArmSetPoint(const manif::SO3d &leftUpperArmOrientation,
                                        const manif::SO3Tangentd &leftUpperArmAngularVelocity)
{
    m_LeftUpperArmTask.task->setSetPoint(leftUpperArmOrientation, leftUpperArmAngularVelocity);

    return true;
}

bool HumanIK::setLeftForeArmSetPoint(const manif::SO3d &leftForeArmOrientation,
                                        const manif::SO3Tangentd &leftForeArmAngularVelocity)
{
    m_LeftForeArmTask.task->setSetPoint(leftForeArmOrientation, leftForeArmAngularVelocity);

    return true;
}

bool HumanIK::setRightUpperLegSetPoint(const manif::SO3d &rightUpperLegOrientation,
                                        const manif::SO3Tangentd &rightUpperLegAngularVelocity)
{
    m_RightUpperLegTask.task->setSetPoint(rightUpperLegOrientation, rightUpperLegAngularVelocity);

    return true;
}

bool HumanIK::setRightLowerLegSetPoint(const manif::SO3d &rightLowerLegOrientation,
                                        const manif::SO3Tangentd &rightLowerLegAngularVelocity)
{
    m_RightLowerLegTask.task->setSetPoint(rightLowerLegOrientation, rightLowerLegAngularVelocity);

    return true;
}

bool HumanIK::setLeftUpperLegSetPoint(const manif::SO3d &leftUpperLegOrientation,
                                        const manif::SO3Tangentd &leftUpperLegAngularVelocity)
{
    m_LeftUpperLegTask.task->setSetPoint(leftUpperLegOrientation, leftUpperLegAngularVelocity);

    return true;
}

bool HumanIK::setLeftLowerLegSetPoint(const manif::SO3d &leftLowerLegOrientation,
                                        const manif::SO3Tangentd &leftLowerLegAngularVelocity)
{
    m_LeftLowerLegTask.task->setSetPoint(leftLowerLegOrientation, leftLowerLegAngularVelocity);

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
