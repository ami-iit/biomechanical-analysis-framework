#include <BiomechanicalAnalysis/ID/InverseDynamics.h>
#include <BiomechanicalAnalysis/Logging/Logger.h>

using namespace BiomechanicalAnalysis::ID;

bool HumanID::initialize(std::shared_ptr<iDynTree::KinDynComputations> kinDyn)
{

    constexpr auto logPrefix = "[HumanID::intizialize]";

    // Check the validity of the kinDyn object
    if ((kinDyn == nullptr) || (!kinDyn->isValid()))
    {
        BiomechanicalAnalysis::log()->error("{} Invalid kinDyn object.", logPrefix);
        return false;
    }

    m_kinDyn = kinDyn;

    // Resize and initialize the KinematicState object
    m_kinState.floatingBaseFrameIndex = m_kinDyn->getFrameIndex(m_kinDyn->getFloatingBase());
    m_kinState.jointsPosition.resize(m_kinDyn->getNrOfDegreesOfFreedom());
    m_kinState.jointsPosition.zero();
    m_kinState.jointsVelocity.resize(m_kinDyn->getNrOfDegreesOfFreedom());
    m_kinState.jointsVelocity.zero();
    m_kinState.baseAngularVelocity.zero();
    m_measurement.resize(m_berdyHelper.getNrOfSensorsMeasurements());

    // Initialize the options
    iDynTree::BerdyOptions berdyOptions;
    berdyOptions.baseLink = m_kinDyn->getFloatingBase();
    berdyOptions.berdyVariant = iDynTree::BerdyVariants::BERDY_FLOATING_BASE;
    berdyOptions.includeAllNetExternalWrenchesAsSensors = true;
    berdyOptions.includeAllNetExternalWrenchesAsDynamicVariables = true;
    berdyOptions.includeAllJointAccelerationsAsSensors = true;
    berdyOptions.includeAllJointTorquesAsSensors = false;
    berdyOptions.includeFixedBaseExternalWrench = false;

    iDynTree::SensorsList sensorsList = m_kinDyn->getRobotModel().sensors();

    std::cout << "nr of accelerometer sensors: "
              << sensorsList.getNrOfSensors(iDynTree::ACCELEROMETER) << std::endl;
    std::cout << "nr of gyroscope sensors: " << sensorsList.getNrOfSensors(iDynTree::GYROSCOPE)
              << std::endl;
    std::cout << "nr of six axis force torque sensors: "
              << sensorsList.getNrOfSensors(iDynTree::SIX_AXIS_FORCE_TORQUE) << std::endl;

    // Initialize the BerdyHelper object
    if (!m_berdyHelper.init(m_kinDyn->getRobotModel(), berdyOptions))
    {
        BiomechanicalAnalysis::log()->error("{} Error initializing the BerdyHelper object.",
                                            logPrefix);
        return false;
    }
    m_estimatedDynamicVariables.resize(m_berdyHelper.getNrOfDynamicVariables());

    // Initialize the BerdySparseMAPSolver object
    m_berdySolver = std::make_unique<iDynTree::BerdySparseMAPSolver>(m_berdyHelper);
    if (!m_berdySolver->initialize())
    {
        BiomechanicalAnalysis::log()->error("{} Error initializing the BerdySparseMAPSolver "
                                            "object.",
                                            logPrefix);
        return false;
    }

    return true;
}

bool HumanID::solve()
{
    constexpr auto logPrefix = "[HumanID::solve]";

    // Update the kinematic state
    m_kinDyn->getJointPos(m_kinState.jointsPosition);
    m_kinDyn->getJointVel(m_kinState.jointsVelocity);
    m_kinState.baseAngularVelocity = m_kinDyn->getBaseTwist().getAngularVec3();

    // Update the berdyHelper object with the current kinematic state
    if (!m_berdyHelper.updateKinematicsFromFloatingBase(m_kinState.jointsPosition,
                                                        m_kinState.jointsVelocity,
                                                        m_kinState.floatingBaseFrameIndex,
                                                        m_kinState.baseAngularVelocity))
    {
        BiomechanicalAnalysis::log()->error("{} Error updating the kinematics from the floating "
                                            "base.",
                                            logPrefix);
        return false;
    }

    m_berdySolver->updateEstimateInformationFloatingBase(m_kinState.jointsPosition,
                                                         m_kinState.jointsVelocity,
                                                         m_kinState.floatingBaseFrameIndex,
                                                         m_kinState.baseAngularVelocity,
                                                         m_measurement);

    if (!m_berdySolver->doEstimate())
    {
        BiomechanicalAnalysis::log()->error("{} Error in the estimation of the dynamics.",
                                            logPrefix);
        return false;
    }

    // Extract the estimated dynamic variables
    m_berdySolver->getLastEstimate(m_estimatedDynamicVariables);

    if (!m_berdyHelper.extractJointTorquesFromDynamicVariables(m_estimatedDynamicVariables,
                                                               m_kinState.jointsPosition,
                                                               m_estimatedJointTorques))
    {
        BiomechanicalAnalysis::log()->error("{} Error extracting the joint torques from the "
                                            "estimated dynamic variables.",
                                            logPrefix);
        return false;
    }

    return true;
}

iDynTree::VectorDynSize HumanID::getJointTorques()
{
    return m_estimatedJointTorques;
}
