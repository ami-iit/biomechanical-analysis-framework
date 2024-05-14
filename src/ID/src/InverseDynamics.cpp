#include <BiomechanicalAnalysis/ID/InverseDynamics.h>
#include <BiomechanicalAnalysis/Logging/Logger.h>

using namespace BiomechanicalAnalysis::ID;

bool HumanID::initialize(
    std::weak_ptr<const BipedalLocomotion::ParametersHandler::IParametersHandler> handler,
    std::shared_ptr<iDynTree::KinDynComputations> kinDyn)
{

    constexpr auto logPrefix = "[HumanID::intizialize]";
    auto ptr = handler.lock();

    if (!ptr->getParameter("humanMass", m_humanMass))
    {
        BiomechanicalAnalysis::log()->error("{} Error getting the 'humanMass' parameter.",
                                            logPrefix);
        return false;
    }

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
    m_jointTorquesHelper.estimatedJointTorques.resize(m_kinDyn->getNrOfDegreesOfFreedom());

    auto jointTorquesHandler = ptr->getGroup("JOINT_TORQUES").lock();
    if (jointTorquesHandler == nullptr)
    {
        BiomechanicalAnalysis::log()->error("{} Error getting the JOINT_TORQUES group.", logPrefix);
        return false;
    }
    // Initialize the BerdyHelper and BerdySparseMAPSolver objects
    if (!initializeJointTorquesHelper(jointTorquesHandler))
    {
        BiomechanicalAnalysis::log()->error("{} Error initializing the joint torques helper.",
                                            logPrefix);
        return false;
    }

    auto extWrenchesHandler = ptr->getGroup("EXTERNAL_WRENCHES").lock();
    if (extWrenchesHandler == nullptr)
    {
        BiomechanicalAnalysis::log()->error("{} Error getting the EXTERNAL_WRENCHES group.",
                                            logPrefix);
        return false;
    }
    if (!initializeExtWrenchesHelper(extWrenchesHandler))
    {
        BiomechanicalAnalysis::log()->error("{} Error initializing the external wrenches helper.",
                                            logPrefix);
        return false;
    }

    m_extWrenchesHelper.estimatedDynamicVariables.resize(
        m_extWrenchesHelper.berdyHelper.getNrOfDynamicVariables());
    m_extWrenchesHelper.measurement.resize(
        m_extWrenchesHelper.berdyHelper.getNrOfSensorsMeasurements());
    m_jointTorquesHelper.measurement.resize(
        m_jointTorquesHelper.berdyHelper.getNrOfSensorsMeasurements());

    iDynTree::SensorsList sensorsList = m_kinDyn->getRobotModel().sensors();

    return true;
}

bool HumanID::updateExtWrenchesMeasurements(
    const std::unordered_map<std::string, iDynTree::Wrench>& wrenches)
{
    constexpr auto logPrefix = "[HumanID::updateExtWrenchesMeasurements]";

    m_extWrenchesHelper.measurement.zero();
    for (int i = 0; i < m_wrenchSources.size(); i++)
    {
        if (m_wrenchSources[i].type == WrenchSourceType::Dummy)
        {
            m_wrenchSources[i].outputFrameTransform.setRotation(
                m_kinDyn->getWorldTransform(m_wrenchSources[i].outputFrame).getRotation());
        } else if (m_wrenchSources[i].type == WrenchSourceType::Fixed)
        {
            if (wrenches.find(m_wrenchSources[i].outputFrame) == wrenches.end())
            {
                BiomechanicalAnalysis::log()->error("{} Wrench {} not found.",
                                                    logPrefix,
                                                    m_wrenchSources[i].outputFrame);
                return false;
            }
            m_wrenchSources[i].wrench = m_wrenchSources[i].outputFrameTransform
                                        * wrenches.at(m_wrenchSources[i].outputFrame);

            iDynTree::IndexRange sensorRange
                = m_extWrenchesHelper.berdyHelper
                      .getRangeLinkSensorVariable(iDynTree::BerdySensorTypes::NET_EXT_WRENCH_SENSOR,
                                                  m_kinDyn->getFrameIndex(
                                                      m_wrenchSources[i].outputFrame));
            for (int j = 0; j < 6; j++)
            {
                m_extWrenchesHelper.measurement(sensorRange.offset + j)
                    = m_wrenchSources[i].wrench(j);
            }
        }
    }
    iDynTree::SpatialForceVector rcmWrench = computeRCMInBaseFrame();
    iDynTree::IndexRange rcmSensorRange = m_extWrenchesHelper.berdyHelper.getRangeRCMSensorVariable(
        iDynTree::BerdySensorTypes::RCM_SENSOR);
    for (int i = 0; i < 6; i++)
    {
        m_extWrenchesHelper.measurement(rcmSensorRange.offset + i) = rcmWrench(i);
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

    m_extWrenchesHelper.berdySolver
        ->updateEstimateInformationFloatingBase(m_kinState.jointsPosition,
                                                m_kinState.jointsVelocity,
                                                m_kinState.floatingBaseFrameIndex,
                                                m_kinState.baseAngularVelocity,
                                                m_extWrenchesHelper.measurement);
    if (!m_extWrenchesHelper.berdySolver->doEstimate())
    {
        BiomechanicalAnalysis::log()->error("{} Error in the estimation of the dynamics.",
                                            logPrefix);
        return false;
    }

    const iDynTree::VectorDynSize& estimatedWrenches
        = m_extWrenchesHelper.berdySolver->getLastEstimate();

    iDynTree::LinkNetExternalWrenches linkExtWrenches(m_kinDyn->getRobotModel());
    m_extWrenchesHelper.berdyHelper
        .extractLinkNetExternalWrenchesFromDynamicVariables(estimatedWrenches, linkExtWrenches);

    for (std::size_t i = 0; i < m_wrenchSources.size(); i++)
    {
        iDynTree::LinkIndex linkIndex
            = m_kinDyn->getRobotModel().getLinkIndex(m_wrenchSources[i].outputFrame);
        for (int j = 0; j < 6; j++)
        {
            m_estimatedExtWrenches[i](j) = linkExtWrenches(linkIndex)(j);
        }
    }

    // Update the berdyHelper object with the current kinematic state
    if (!m_jointTorquesHelper.berdyHelper
             .updateKinematicsFromFloatingBase(m_kinState.jointsPosition,
                                               m_kinState.jointsVelocity,
                                               m_kinState.floatingBaseFrameIndex,
                                               m_kinState.baseAngularVelocity))
    {
        BiomechanicalAnalysis::log()->error("{} Error updating the kinematics from the floating "
                                            "base.",
                                            logPrefix);
        return false;
    }

    m_jointTorquesHelper.berdySolver
        ->updateEstimateInformationFloatingBase(m_kinState.jointsPosition,
                                                m_kinState.jointsVelocity,
                                                m_kinState.floatingBaseFrameIndex,
                                                m_kinState.baseAngularVelocity,
                                                m_jointTorquesHelper.measurement);

    if (!m_jointTorquesHelper.berdySolver->doEstimate())
    {
        BiomechanicalAnalysis::log()->error("{} Error in the estimation of the dynamics.",
                                            logPrefix);
        return false;
    }

    // Extract the estimated dynamic variables
    m_jointTorquesHelper.berdySolver->getLastEstimate(
        m_jointTorquesHelper.estimatedDynamicVariables);

    if (!m_jointTorquesHelper.berdyHelper
             .extractJointTorquesFromDynamicVariables(m_jointTorquesHelper.estimatedDynamicVariables,
                                                      m_kinState.jointsPosition,
                                                      m_jointTorquesHelper.estimatedJointTorques))
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
    return m_jointTorquesHelper.estimatedJointTorques;
}

bool HumanID::initializeJointTorquesHelper(
    const std::shared_ptr<BipedalLocomotion::ParametersHandler::IParametersHandler> groupHandler)
{
    constexpr auto logPrefix = "[HumanID::intizialize::initializeJointTorquesHelper]";

    // Initialize the options
    iDynTree::BerdyOptions berdyOptions;
    berdyOptions.baseLink = m_kinDyn->getFloatingBase();
    berdyOptions.berdyVariant = iDynTree::BerdyVariants::BERDY_FLOATING_BASE;
    berdyOptions.includeAllNetExternalWrenchesAsSensors = true;
    berdyOptions.includeAllNetExternalWrenchesAsDynamicVariables = true;
    berdyOptions.includeAllJointAccelerationsAsSensors = true;
    berdyOptions.includeAllJointTorquesAsSensors = false;
    berdyOptions.includeFixedBaseExternalWrench = false;

    auto removeSensorHandler = groupHandler->getGroup("SENSOR_REMOVAL").lock();
    if (removeSensorHandler == nullptr)
    {
        BiomechanicalAnalysis::log()->error("{} Error getting the 'SENSOR_REMOVAL' group.",
                                            logPrefix);
        return false;
    }

    auto sensorList = m_kinDyn->getRobotModel().sensors();
    for (auto& sensor : mapBerdySensorType)
    {
        std::string sensorName;
        if (removeSensorHandler->getParameter(sensor.second, sensorName))
        {
            if (sensorName == "*")
            {
                if (!sensorList.removeAllSensorsOfType(
                        static_cast<iDynTree::SensorType>(sensor.first)))
                {
                    BiomechanicalAnalysis::log()->error("{} Error removing all sensors of type {}.",
                                                        logPrefix,
                                                        sensor.second);
                    return false;
                }
            } else
            {
                if (!sensorList.removeSensor(static_cast<iDynTree::SensorType>(sensor.first),
                                             sensorName))
                {
                    BiomechanicalAnalysis::log()->error("{} Error removing sensor {}.",
                                                        logPrefix,
                                                        sensorName);
                }
            }
        }
    }

    // Initialize the BerdyHelper object
    if (!m_jointTorquesHelper.berdyHelper.init(m_kinDyn->getRobotModel(), berdyOptions))
    {
        BiomechanicalAnalysis::log()->error("{} Error initializing the BerdyHelper object.",
                                            logPrefix);
        return false;
    }
    m_jointTorquesHelper.estimatedDynamicVariables.resize(
        m_jointTorquesHelper.berdyHelper.getNrOfDynamicVariables());

    std::vector<iDynTree::BerdySensor> berdySensors
        = m_jointTorquesHelper.berdyHelper.getSensorsOrdering();

    // Initialize the BerdySparseMAPSolver object
    m_jointTorquesHelper.berdySolver
        = std::make_unique<iDynTree::BerdySparseMAPSolver>(m_jointTorquesHelper.berdyHelper);
    if (!m_jointTorquesHelper.berdySolver->initialize())
    {
        BiomechanicalAnalysis::log()->error("{} Error initializing the BerdySparseMAPSolver "
                                            "object.",
                                            logPrefix);
        return false;
    }
    return true;
}

bool HumanID::initializeExtWrenchesHelper(
    const std::shared_ptr<BipedalLocomotion::ParametersHandler::IParametersHandler> groupHandler)
{
    constexpr auto logPrefix = "[HumanID::intizialize::initializeExtWrenchesHelper]";

    std::vector<std::string> wrenchSource;
    if (!groupHandler->getParameter("wrenchSources", wrenchSource))
    {
        BiomechanicalAnalysis::log()->error("{} Error getting the wrench source parameter.",
                                            logPrefix);
        return false;
    }

    for (auto& wrench : wrenchSource)
    {
        WrenchSourceData data;
        auto wrenchHandler = groupHandler->getGroup(wrench).lock();
        if (wrenchHandler == nullptr)
        {
            BiomechanicalAnalysis::log()->error("{} Error getting the wrench group {}.",
                                                logPrefix,
                                                wrench);
            return false;
        }
        if (!wrenchHandler->getParameter("outputFrame", data.outputFrame))
        {
            BiomechanicalAnalysis::log()->error("{} Error getting the name parameter.", logPrefix);
            return false;
        }
        std::string type;
        if (!wrenchHandler->getParameter("type", type))
        {
            BiomechanicalAnalysis::log()->error("{} Error getting the type parameter.", logPrefix);
            return false;
        }

        if (type == "fixed")
        {
            data.type = WrenchSourceType::Fixed;

        } else if (type == "dummy")
        {
            data.type = WrenchSourceType::Dummy;
        } else
        {
            BiomechanicalAnalysis::log()->error("{} Invalid 'type' parameter {}.", logPrefix, type);
            return false;
        }

        if (data.type == WrenchSourceType::Fixed)
        {
            std::vector<double> position, orientation;
            if (!wrenchHandler->getParameter("position", position))
            {
                BiomechanicalAnalysis::log()->error("{} Error getting the position parameter.",
                                                    logPrefix);
                return false;
            }
            iDynTree::Position positionIDynTree;
            positionIDynTree(0) = position[0];
            positionIDynTree(1) = position[1];
            positionIDynTree(2) = position[2];

            if (!wrenchHandler->getParameter("orientation", orientation))
            {
                BiomechanicalAnalysis::log()->error("{} Error getting the orientation parameter.",
                                                    logPrefix);
                return false;
            }
            iDynTree::Rotation orientationIDynTree(orientation[0],
                                                   orientation[1],
                                                   orientation[2],
                                                   orientation[3],
                                                   orientation[4],
                                                   orientation[5],
                                                   orientation[6],
                                                   orientation[7],
                                                   orientation[8]);
            data.outputFrameTransform = iDynTree::Transform(orientationIDynTree, positionIDynTree);
        } else if (data.type == WrenchSourceType::Dummy)
        {
            std::vector<double> values;
            if (!wrenchHandler->getParameter("values", values))
            {
                BiomechanicalAnalysis::log()->error("{} Error getting the values parameter.",
                                                    logPrefix);
                return false;
            }
            for (int i = 0; i < 6; i++)
            {
                data.wrench(i) = values[i];
            }
        }
        m_wrenchSources.push_back(data);
    }
    m_estimatedExtWrenches.resize(m_wrenchSources.size());

    if (!groupHandler->getParameter("mu_dyn_variables",
                                    m_mapEstParams.priorDynamicsRegularizationExpected))
    {
        BiomechanicalAnalysis::log()->error("{} Error getting the 'mu_dyn_variables' parameter.",
                                            logPrefix);
        return false;
    }

    if (!groupHandler->getParameter("cov_dyn_variables",
                                    m_mapEstParams.priorDynamicsRegularizationCovarianceValue))
    {
        BiomechanicalAnalysis::log()->error("{} Error getting the 'cov_dyn_variables' parameter.",
                                            logPrefix);
        return false;
    }

    std::vector<std::string> specificElements;
    if (!groupHandler->getParameter("specificElements", specificElements))
    {
        BiomechanicalAnalysis::log()->error("{} Error getting the 'measurements' parameter.",
                                            logPrefix);
        return false;
    }

    for (auto& element : specificElements)
    {
        std::vector<double> covariance;
        if (!groupHandler->getParameter(element, covariance))
        {
            BiomechanicalAnalysis::log()->error("{} Error getting the '{}' parameter.",
                                                logPrefix,
                                                element);
            return false;
        }
        m_mapEstParams.specificMeasurementsCovariance[element] = covariance;
    }

    if (!groupHandler->getParameter("cov_measurements_RCM_SENSOR",
                                    m_mapEstParams.specificMeasurementsCovariance["RCM_SENSOR"]))
    {
        BiomechanicalAnalysis::log()->error("{} Error getting the 'cov_measurements_RCM_SENSOR' "
                                            "parameter.",
                                            logPrefix);
        return false;
    }

    iDynTree::BerdyOptions berdyOptionsExtWrenches;
    berdyOptionsExtWrenches.berdyVariant
        = iDynTree::BerdyVariants::BERDY_FLOATING_BASE_NON_COLLOCATED_EXT_WRENCHES;
    berdyOptionsExtWrenches.includeAllNetExternalWrenchesAsSensors = true;
    berdyOptionsExtWrenches.includeAllJointTorquesAsSensors = false;
    berdyOptionsExtWrenches.includeAllJointAccelerationsAsSensors = false;
    berdyOptionsExtWrenches.includeAllNetExternalWrenchesAsSensors = true;
    berdyOptionsExtWrenches.includeAllNetExternalWrenchesAsDynamicVariables = true;

    if (!m_extWrenchesHelper.berdyHelper.init(m_kinDyn->getRobotModel(), berdyOptionsExtWrenches))
    {
        BiomechanicalAnalysis::log()->error("{} Error initializing the BerdyHelper object.",
                                            logPrefix);
        return false;
    }

    m_extWrenchesHelper.berdySolver
        = std::make_unique<iDynTree::BerdySparseMAPSolver>(m_extWrenchesHelper.berdyHelper);

    if (!m_extWrenchesHelper.berdySolver->initialize())
    {
        BiomechanicalAnalysis::log()->error("{} Error initializing the BerdySparseMAPSolver "
                                            "object.",
                                            logPrefix);
        return false;
    }

    iDynTree::Triplets measurementsCovarianceMatrixTriplets;
    for (const iDynTree::BerdySensor& berdySensor :
         m_extWrenchesHelper.berdyHelper.getSensorsOrdering())
    {
        switch (berdySensor.type)
        {
        case iDynTree::BerdySensorTypes::NET_EXT_WRENCH_SENSOR: {
            // initialize with default covariance
            iDynTree::Vector6 wrenchCovariance;
            for (int i = 0; i < 6; i++)
                wrenchCovariance.setVal(i, m_mapEstParams.measurementDefaultCovariance);

            // set specific covariance if configured
            auto specificMeasurementsPtr
                = m_mapEstParams.specificMeasurementsCovariance.find(berdySensor.id);
            if (specificMeasurementsPtr != m_mapEstParams.specificMeasurementsCovariance.end())
            {
                for (int i = 0; i < 6; i++)
                    wrenchCovariance.setVal(i, specificMeasurementsPtr->second[i]);
            }

            for (std::size_t i = 0; i < 6; i++)
                measurementsCovarianceMatrixTriplets.setTriplet({berdySensor.range.offset + i,
                                                                 berdySensor.range.offset + i,
                                                                 wrenchCovariance[i]});
        }
        break;
        case iDynTree::BerdySensorTypes::RCM_SENSOR: {
            auto specificMeasurementsPtr = m_mapEstParams.specificMeasurementsCovariance.find("RCM_"
                                                                                              "SENS"
                                                                                              "OR");
            for (std::size_t i = 0; i < 6; i++)
            {
                measurementsCovarianceMatrixTriplets.setTriplet(
                    {berdySensor.range.offset + i,
                     berdySensor.range.offset + i,
                     specificMeasurementsPtr->second[i]});
            }
        }
        break;
        default:
            break;
        }
    }
    iDynTree::SparseMatrix<iDynTree::ColumnMajor> measurementsPriorCovarianceMatrix;
    std::size_t sigmaYSize = m_extWrenchesHelper.berdyHelper.getNrOfSensorsMeasurements();
    measurementsPriorCovarianceMatrix.resize(sigmaYSize, sigmaYSize);
    measurementsPriorCovarianceMatrix.setFromTriplets(measurementsCovarianceMatrixTriplets);
    m_extWrenchesHelper.berdySolver->setMeasurementsPriorCovariance(
        measurementsPriorCovarianceMatrix);

    return true;
}

iDynTree::SpatialForceVector HumanID::computeRCMInBaseFrame()
{
    iDynTree::Vector3 world_gravity;
    iDynTree::SpatialForceVector subjectWeightInCentroidal(world_gravity,
                                                           iDynTree::AngularForceVector3(0.0,
                                                                                         0.0,
                                                                                         0.0));
    subjectWeightInCentroidal = subjectWeightInCentroidal * (-m_humanMass);

    iDynTree::Transform base_H_centroidal;
    base_H_centroidal.setPosition(m_kinDyn->getCenterOfMassPosition()
                                  - m_kinDyn->getWorldBaseTransform().getPosition());
    base_H_centroidal.setRotation(m_kinDyn->getWorldBaseTransform().getRotation().inverse());

    return base_H_centroidal * subjectWeightInCentroidal;
}
