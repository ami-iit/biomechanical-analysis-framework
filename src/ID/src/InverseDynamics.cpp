#include <BiomechanicalAnalysis/ID/InverseDynamics.h>
#include <BiomechanicalAnalysis/Logging/Logger.h>
#include <iDynTree/EigenHelpers.h>
#include <iDynTree/ModelLoader.h>

using namespace BiomechanicalAnalysis::ID;

bool HumanID::initialize(std::weak_ptr<const BipedalLocomotion::ParametersHandler::IParametersHandler> handler,
                         std::shared_ptr<iDynTree::KinDynComputations> kinDyn)
{

    constexpr auto logPrefix = "[HumanID::intizialize]";
    auto ptr = handler.lock();

    if (!ptr->getParameter("humanMass", m_humanMass))
    {
        BiomechanicalAnalysis::log()->error("{} Error getting the 'humanMass' parameter.", logPrefix);
        return false;
    }

    // Check the validity of the kinDyn object
    if ((kinDyn == nullptr) || (!kinDyn->isValid()))
    {
        BiomechanicalAnalysis::log()->error("{} Invalid kinDyn object.", logPrefix);
        return false;
    }

    m_kinDyn = kinDyn;

    // check if the model path is provided and load the model otherwise use the kinDyn object passed
    iDynTree::ModelLoader loader;
    std::string urdfModel;
    if (ptr->getParameter("urdfModel", urdfModel))
    {
        std::optional<std::string> urdfOpt = ResolveRoboticsURICpp::resolveRoboticsURI(urdfModel);
        if (!urdfOpt.has_value())
        {
            BiomechanicalAnalysis::log()->error("Cannot resolve the URDF file");
            return false;
        }
        m_modelPath = urdfOpt.value();
        std::vector<std::string> jointsList;
        if (ptr->getParameter("jointsList", jointsList))
        {
            if (!loader.loadReducedModelFromFile(m_modelPath, jointsList))
            {
                BiomechanicalAnalysis::log()->error("{} Error loading the model from file {}.", logPrefix, m_modelPath);
                return false;
            }
        } else
        {
            BiomechanicalAnalysis::log()->warn("{} Parameter 'jointsList' not found in the "
                                               "configuration file, loading the model with all the "
                                               "joints.",
                                               logPrefix);
            if (!loader.loadModelFromFile(m_modelPath))
            {
                BiomechanicalAnalysis::log()->error("{} Error loading the model from file {}.", logPrefix, m_modelPath);
                return false;
            }
        }

        m_kinDynFullModel = std::make_shared<iDynTree::KinDynComputations>();
        if (!m_kinDynFullModel->loadRobotModel(loader.model()))
        {
            BiomechanicalAnalysis::log()->error("{} Error loading the model from file {}.", logPrefix, m_modelPath);
            return false;
        }
        m_kinDynFullModel->setFloatingBase(m_kinDyn->getFloatingBase());
        m_useFullModel = true;
    } else
    {
        BiomechanicalAnalysis::log()->warn("{} Error getting the 'urdfModel' parameter, using the "
                                           "default model.",
                                           logPrefix);
        m_useFullModel = false;
        m_kinDynFullModel = m_kinDyn;
    }

    // Resize and initialize the KinematicState object
    m_kinState.floatingBaseFrameIndex = m_kinDyn->getFrameIndex(m_kinDyn->getFloatingBase());
    m_kinState.jointsPosition.resize(m_kinDynFullModel->model().getNrOfDOFs());
    m_kinState.jointsPosition.zero();
    m_kinState.jointsVelocity.resize(m_kinDynFullModel->model().getNrOfDOFs());
    m_kinState.jointsVelocity.zero();
    m_kinState.baseAngularVelocity.zero();
    m_jointTorquesHelper.estimatedJointTorques.resize(m_kinDynFullModel->model().getNrOfDOFs());

    // Get the group handlers for initializing the MAPHelper m_jointTorquesHelper object
    auto jointTorquesHandler = ptr->getGroup("JOINT_TORQUES").lock();
    if (jointTorquesHandler == nullptr)
    {
        BiomechanicalAnalysis::log()->error("{} Error getting the JOINT_TORQUES group.", logPrefix);
        return false;
    }

    // Initialize the MAPHelper m_jointTorquesHelper object
    if (!initializeJointTorquesHelper(jointTorquesHandler))
    {
        BiomechanicalAnalysis::log()->error("{} Error initializing the joint torques helper.", logPrefix);
        return false;
    }

    // Get the group handlers for initializing the MAPHelper m_extWrenchesEstimator object
    auto extWrenchesHandler = ptr->getGroup("EXTERNAL_WRENCHES").lock();
    if (extWrenchesHandler == nullptr)
    {
        BiomechanicalAnalysis::log()->error("{} Error getting the EXTERNAL_WRENCHES group.", logPrefix);
        return false;
    }

    // Initialize the MAPHelper m_extWrenchesEstimator object
    if (!initializeExtWrenchesHelper(extWrenchesHandler))
    {
        BiomechanicalAnalysis::log()->error("{} Error initializing the external wrenches helper.", logPrefix);
        return false;
    }

    // Resize the estimated Dynamic Variables and the measurement vectors of the MAPHelper objects
    m_extWrenchesEstimator.estimatedDynamicVariables.resize(m_extWrenchesEstimator.berdyHelper.getNrOfDynamicVariables());
    m_extWrenchesEstimator.measurement.resize(m_extWrenchesEstimator.berdyHelper.getNrOfSensorsMeasurements());
    m_extWrenchesEstimator.measurement.zero();
    m_jointTorquesHelper.estimatedDynamicVariables.resize(m_jointTorquesHelper.berdyHelper.getNrOfDynamicVariables());
    m_jointTorquesHelper.measurement.resize(m_jointTorquesHelper.berdyHelper.getNrOfSensorsMeasurements());

    return true;
}

bool HumanID::updateExtWrenchesMeasurements(const std::unordered_map<std::string, iDynTree::Wrench>& wrenches)
{
    constexpr auto logPrefix = "[HumanID::updateExtWrenchesMeasurements]";
    if (m_useFullModel)
    {
        // if the full model is used, update the kinematic state of the full model
        iDynTree::Transform w_H_b;
        iDynTree::VectorDynSize s;
        s.resize(m_kinDyn->getNrOfDegreesOfFreedom());
        iDynTree::Twist base_velocity;
        iDynTree::VectorDynSize s_dot;
        s_dot.resize(m_kinDyn->getNrOfDegreesOfFreedom());
        iDynTree::Vector3 world_gravity;
        m_kinDyn->getRobotState(w_H_b, s, base_velocity, s_dot, world_gravity);
        m_kinState.jointsPosition.zero();
        m_kinState.jointsVelocity.zero();
        for (int i = 0; i < m_kinDynFullModel->getNrOfDegreesOfFreedom(); i++)
        {
            for (int j = 0; j < m_kinDyn->getNrOfDegreesOfFreedom(); j++)
            {
                if (m_kinDynFullModel->getRobotModel().getJointName(i) == m_kinDyn->getRobotModel().getJointName(j))
                {
                    m_kinState.jointsPosition(i) = s(j);
                    m_kinState.jointsVelocity(i) = s_dot(j);
                    break;
                }
            }
        }
        m_kinDynFullModel->setRobotState(w_H_b, m_kinState.jointsPosition, base_velocity, m_kinState.jointsVelocity, world_gravity);
    }

    // Update the measurement vector with the measurements of the wrenches
    m_extWrenchesEstimator.measurement.zero();
    for (int i = 0; i < m_wrenchSources.size(); i++)
    {
        if (m_wrenchSources[i].type == WrenchSourceType::Dummy)
        {
            m_wrenchSources[i].outputFrameTransform.setRotation(
                m_kinDynFullModel->getWorldTransform(m_wrenchSources[i].outputFrame).getRotation());
        } else if (m_wrenchSources[i].type == WrenchSourceType::Fixed)
        {
            if (wrenches.find(m_wrenchSources[i].outputFrame) == wrenches.end())
            {
                BiomechanicalAnalysis::log()->error("{} Wrench {} not found.", logPrefix, m_wrenchSources[i].outputFrame);
                return false;
            }
            m_wrenchSources[i].wrench = m_wrenchSources[i].outputFrameTransform * wrenches.at(m_wrenchSources[i].outputFrame);
            iDynTree::Wrench wrenchMeas = m_wrenchSources[i].outputFrameTransform * wrenches.at(m_wrenchSources[i].outputFrame);
            iDynTree::LinkIndex linkIndex = m_extWrenchesEstimator.berdyHelper.model().getLinkIndex(m_wrenchSources[i].outputFrame);
            if (linkIndex == iDynTree::LINK_INVALID_INDEX)
            {
                BiomechanicalAnalysis::log()->error("{} Link {} not found.", logPrefix, m_wrenchSources[i].outputFrame);
                return false;
            }

            iDynTree::IndexRange sensorRange
                = m_extWrenchesEstimator.berdyHelper.getRangeLinkSensorVariable(iDynTree::BerdySensorTypes::NET_EXT_WRENCH_SENSOR,
                                                                                m_kinDynFullModel->getFrameIndex(
                                                                                    m_wrenchSources[i].outputFrame));
            for (int j = 0; j < 6; j++)
            {
                m_extWrenchesEstimator.measurement(sensorRange.offset + j) = m_wrenchSources[i].wrench(j);
            }
        }
    }
    iDynTree::SpatialForceVector rcmWrench = computeRCMInBaseFrame();
    iDynTree::IndexRange rcmSensorRange
        = m_extWrenchesEstimator.berdyHelper.getRangeRCMSensorVariable(iDynTree::BerdySensorTypes::RCM_SENSOR);
    for (int i = 0; i < 6; i++)
    {
        m_extWrenchesEstimator.measurement(rcmSensorRange.offset + i) = rcmWrench(i);
    }

    return true;
}

bool HumanID::solve()
{
    constexpr auto logPrefix = "[HumanID::solve]";

    // Update the kinematic state
    m_kinDynFullModel->getJointPos(m_kinState.jointsPosition);
    m_kinDynFullModel->getJointVel(m_kinState.jointsVelocity);
    m_kinState.baseAngularVelocity = m_kinDyn->getBaseTwist().getAngularVec3();

    // Update the berdySolver object with the current kinematic state
    m_extWrenchesEstimator.berdySolver->updateEstimateInformationFloatingBase(m_kinState.jointsPosition,
                                                                              m_kinState.jointsVelocity,
                                                                              m_kinState.floatingBaseFrameIndex,
                                                                              m_kinState.baseAngularVelocity,
                                                                              m_extWrenchesEstimator.measurement);
    // Estimate the external wrenches
    if (!m_extWrenchesEstimator.berdySolver->doEstimate())
    {
        BiomechanicalAnalysis::log()->error("{} Error in the estimation of the dynamics.", logPrefix);
        return false;
    }

    m_extWrenchesEstimator.berdySolver->getLastEstimate(m_extWrenchesEstimator.estimatedDynamicVariables);

    iDynTree::LinkNetExternalWrenches linkExtWrenches(m_kinDynFullModel->getRobotModel());
    m_extWrenchesEstimator.berdyHelper.extractLinkNetExternalWrenchesFromDynamicVariables(m_extWrenchesEstimator.estimatedDynamicVariables,
                                                                                          linkExtWrenches);

    // Extract the estimated external wrenches
    for (std::size_t i = 0; i < m_wrenchSources.size(); i++)
    {
        iDynTree::LinkIndex linkIndex = m_kinDynFullModel->getRobotModel().getLinkIndex(m_wrenchSources[i].outputFrame);
        for (int j = 0; j < 6; j++)
        {
            m_estimatedExtWrenches[i](j) = linkExtWrenches(linkIndex)(j);
        }
    }

    // Update the measurement vector of the m_jointTorquesHelper object
    m_jointTorquesHelper.measurement.zero();
    for (std::size_t i = 0; i < m_wrenchSources.size(); i++)
    {
        iDynTree::IndexRange sensorRange
            = m_jointTorquesHelper.berdyHelper.getRangeLinkSensorVariable(iDynTree::BerdySensorTypes::NET_EXT_WRENCH_SENSOR,
                                                                          m_kinDynFullModel->getFrameIndex(m_wrenchSources[i].outputFrame));
        for (int j = 0; j < 6; j++)
        {
            m_jointTorquesHelper.measurement(sensorRange.offset + j) = m_estimatedExtWrenches[i](j);
        }
    }

    // Update the berdyHelper object with the current kinematic state
    if (!m_jointTorquesHelper.berdyHelper.updateKinematicsFromFloatingBase(m_kinState.jointsPosition,
                                                                           m_kinState.jointsVelocity,
                                                                           m_kinState.floatingBaseFrameIndex,
                                                                           m_kinState.baseAngularVelocity))
    {
        BiomechanicalAnalysis::log()->error("{} Error updating the kinematics from the floating "
                                            "base.",
                                            logPrefix);
        return false;
    }

    // Update the berdySolver object with the current kinematic state
    m_jointTorquesHelper.berdySolver->updateEstimateInformationFloatingBase(m_kinState.jointsPosition,
                                                                            m_kinState.jointsVelocity,
                                                                            m_kinState.floatingBaseFrameIndex,
                                                                            m_kinState.baseAngularVelocity,
                                                                            m_jointTorquesHelper.measurement);
    // Estimate the joint torques
    if (!m_jointTorquesHelper.berdySolver->doEstimate())
    {
        BiomechanicalAnalysis::log()->error("{} Error in the estimation of the dynamics.", logPrefix);
        return false;
    }

    // Extract the estimated dynamic variables
    m_jointTorquesHelper.berdySolver->getLastEstimate(m_jointTorquesHelper.estimatedDynamicVariables);

    // Extract the estimated joint torques
    if (!m_jointTorquesHelper.berdyHelper.extractJointTorquesFromDynamicVariables(m_jointTorquesHelper.estimatedDynamicVariables,
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

void HumanID::getJointTorques(Eigen::Ref<Eigen::VectorXd> jointTorques)
{
    if (jointTorques.size() != m_kinDynFullModel->model().getNrOfDOFs())
    {
        BiomechanicalAnalysis::log()->error("[HumanID::getJointTorques] The size of the input vector "
                                            "is different from the number of DOFs of the model.");
        return;
    }
    jointTorques = iDynTree::toEigen(m_jointTorquesHelper.estimatedJointTorques);
}

std::vector<std::string> HumanID::getJointsList()
{
    std::vector<std::string> jointNames;
    for (int i = 0; i < m_kinDynFullModel->model().getNrOfJoints(); i++)
    {
        jointNames.push_back(m_kinDynFullModel->model().getJointName(i));
    }
    return jointNames;
}

std::vector<iDynTree::Wrench> HumanID::getEstimatedExtWrenches()
{
    return m_estimatedExtWrenches;
}

std::vector<std::string> HumanID::getEstimatedExtWrenchesList()
{
    std::vector<std::string> wrenchSources;
    for (auto& wrench : m_wrenchSources)
    {
        wrenchSources.push_back(wrench.outputFrame);
    }
    return wrenchSources;
}

bool HumanID::initializeJointTorquesHelper(const std::shared_ptr<BipedalLocomotion::ParametersHandler::IParametersHandler> groupHandler)
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
        BiomechanicalAnalysis::log()->error("{} Error getting the 'SENSOR_REMOVAL' group.", logPrefix);
        return false;
    }

    iDynTree::SensorsList sensorList = m_kinDynFullModel->getRobotModel().sensors();

    for (auto& sensor : mapBerdySensorType)
    {
        std::string sensorName;
        if (removeSensorHandler->getParameter(sensor.second, sensorName))
        {
            if (sensorName == "*")
            {
                if (!sensorList.removeAllSensorsOfType(static_cast<iDynTree::SensorType>(sensor.first)))
                {
                    BiomechanicalAnalysis::log()->error("{} Error removing all sensors of type {}.", logPrefix, sensor.second);
                    return false;
                }
            } else
            {
                if (!sensorList.removeSensor(static_cast<iDynTree::SensorType>(sensor.first), sensorName))
                {
                    BiomechanicalAnalysis::log()->error("{} Error removing sensor {}.", logPrefix, sensorName);
                }
            }
        }
    }

    // Initialize the BerdyHelper object
    if (!m_jointTorquesHelper.berdyHelper.init(m_kinDynFullModel->model(), berdyOptions))
    {
        BiomechanicalAnalysis::log()->error("{} Error initializing the BerdyHelper object.", logPrefix);
        return false;
    }
    m_jointTorquesHelper.estimatedDynamicVariables.resize(m_jointTorquesHelper.berdyHelper.getNrOfDynamicVariables());

    // Initialize the BerdySparseMAPSolver object
    m_jointTorquesHelper.berdySolver = std::make_unique<iDynTree::BerdySparseMAPSolver>(m_jointTorquesHelper.berdyHelper);
    if (!m_jointTorquesHelper.berdySolver->initialize())
    {
        BiomechanicalAnalysis::log()->error("{} Error initializing the BerdySparseMAPSolver "
                                            "object.",
                                            logPrefix);
        return false;
    }

    // Get dynamic and measurement variable size
    size_t numberOfDynVariables = m_jointTorquesHelper.berdyHelper.getNrOfDynamicVariables();
    size_t numberOfDynEquations = m_jointTorquesHelper.berdyHelper.getNrOfDynamicEquations();
    size_t numberOfMeasurements = m_jointTorquesHelper.berdyHelper.getNrOfSensorsMeasurements();
    // Regularization priors
    iDynTree::VectorDynSize dynamicsRegularizationExpectedValueVector(numberOfDynVariables); // mu_d
    iDynTree::SparseMatrix<iDynTree::ColumnMajor> dynamicsRegularizationCovarianceMatrix(numberOfDynVariables,
                                                                                         numberOfDynVariables); // sigma_d
    // Dynamic constraint prior
    iDynTree::SparseMatrix<iDynTree::ColumnMajor> dynamicsConstraintsCovarianceMatrix(numberOfDynEquations,
                                                                                      numberOfDynEquations); // sigma_D

    // Measurements prior
    iDynTree::SparseMatrix<iDynTree::ColumnMajor> measurementsCovarianceMatrix(numberOfMeasurements, numberOfMeasurements); // sigma_y

    double dynamicsRegularizationCovarianceValue;
    if (!groupHandler->getParameter("mu_dyn_variables", dynamicsRegularizationCovarianceValue))
    {
        BiomechanicalAnalysis::log()->error("{} Error getting the 'mu_dyn_variables' parameter.", logPrefix);
        return false;
    }
    for (size_t i = 0; i < numberOfDynVariables; i++)
    {
        dynamicsRegularizationExpectedValueVector(i) = dynamicsRegularizationCovarianceValue;
    }

    double priorDynamicsRegularizationCovarianceValue;
    if (!groupHandler->getParameter("cov_dyn_variables", priorDynamicsRegularizationCovarianceValue))
    {
        BiomechanicalAnalysis::log()->error("{} Error getting the 'cov_dyn_variables' parameter.", logPrefix);
        return false;
    }
    for (size_t i = 0; i < numberOfDynVariables; i++)
    {
        dynamicsRegularizationCovarianceMatrix.setValue(i, i, priorDynamicsRegularizationCovarianceValue);
    }

    double priorDynamicsConstraintsCovarianceValue;
    if (!groupHandler->getParameter("cov_dyn_constraints", priorDynamicsConstraintsCovarianceValue))
    {
        BiomechanicalAnalysis::log()->error("{} Error getting the 'cov_dyn_constraints' parameter.", logPrefix);
        return false;
    }
    for (size_t i = 0; i < numberOfDynEquations; i++)
    {
        dynamicsConstraintsCovarianceMatrix.setValue(i, i, priorDynamicsConstraintsCovarianceValue);
    }

    iDynTree::Triplets allSensorsTriplets;
    std::string covMeasurementOptionPrefix = "cov_measurements_";
    for (auto& berdySensor : m_jointTorquesHelper.berdyHelper.getSensorsOrdering())
    {
        // Check that the sensor is a valid berdy sensor
        if (mapBerdySensorType.find(berdySensor.type) == mapBerdySensorType.end())
        {
            BiomechanicalAnalysis::log()->error("Failed to find berdy sensor type. Maybe is a new "
                                                "sensor?");
            return false;
        }

        std::string berdySensorTypeString = mapBerdySensorType.at(berdySensor.type);
        std::vector<double> sensorCovarianceVector;
        double sensorCovarianceValue;
        if (groupHandler->getParameter(covMeasurementOptionPrefix + berdySensorTypeString, sensorCovarianceVector))
        {
            if (sensorCovarianceVector.size() != berdySensor.range.size)
            {
                BiomechanicalAnalysis::log()->error("{} Error in the size of the sensor range.", logPrefix);
                return false;
            }
            for (size_t i = 0; i < sensorCovarianceVector.size(); i++)
            {
                iDynTree::Triplet sensorTriplet(berdySensor.range.offset + i, berdySensor.range.offset + i, sensorCovarianceVector[i]);
                allSensorsTriplets.setTriplet(sensorTriplet);
            }
        } else if (groupHandler->getParameter(covMeasurementOptionPrefix + berdySensorTypeString, sensorCovarianceValue))
        {
            if (berdySensor.range.size != 1)
            {
                BiomechanicalAnalysis::log()->error("{} Error in the size of the sensor range.", logPrefix);
                return false;
            }
            iDynTree::Triplet sensorTriplet(berdySensor.range.offset, berdySensor.range.offset, sensorCovarianceValue);
            allSensorsTriplets.setTriplet(sensorTriplet);
        } else
        {
            BiomechanicalAnalysis::log()->error("{} Error getting the 'cov_measurements_{}' "
                                                "parameter.",
                                                logPrefix,
                                                berdySensorTypeString);
            return false;
        }
    }
    measurementsCovarianceMatrix.setFromTriplets(allSensorsTriplets);

    m_jointTorquesHelper.berdySolver->setDynamicsRegularizationPriorExpectedValue(dynamicsRegularizationExpectedValueVector);
    m_jointTorquesHelper.berdySolver->setDynamicsRegularizationPriorCovariance(dynamicsRegularizationCovarianceMatrix);
    m_jointTorquesHelper.berdySolver->setDynamicsConstraintsPriorCovariance(dynamicsConstraintsCovarianceMatrix);
    m_jointTorquesHelper.berdySolver->setMeasurementsPriorCovariance(measurementsCovarianceMatrix);

    return true;
}

bool HumanID::initializeExtWrenchesHelper(const std::shared_ptr<BipedalLocomotion::ParametersHandler::IParametersHandler> groupHandler)
{
    constexpr auto logPrefix = "[HumanID::intizialize::initializeExtWrenchesHelper]";

    std::vector<std::string> wrenchSource;
    if (!groupHandler->getParameter("wrenchSources", wrenchSource))
    {
        BiomechanicalAnalysis::log()->error("{} Error getting the wrench source parameter.", logPrefix);
        return false;
    }

    for (auto& wrench : wrenchSource)
    {
        WrenchSourceData data;
        auto wrenchHandler = groupHandler->getGroup(wrench).lock();
        if (wrenchHandler == nullptr)
        {
            BiomechanicalAnalysis::log()->error("{} Error getting the wrench group {}.", logPrefix, wrench);
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
                BiomechanicalAnalysis::log()->error("{} Error getting the position parameter.", logPrefix);
                return false;
            }
            iDynTree::Position positionIDynTree;
            positionIDynTree(0) = position[0];
            positionIDynTree(1) = position[1];
            positionIDynTree(2) = position[2];

            if (!wrenchHandler->getParameter("orientation", orientation))
            {
                BiomechanicalAnalysis::log()->error("{} Error getting the orientation parameter.", logPrefix);
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
                BiomechanicalAnalysis::log()->error("{} Error getting the values parameter.", logPrefix);
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

    if (!groupHandler->getParameter("mu_dyn_variables", m_extWrenchesEstimator.params.priorDynamicsRegularizationExpected))
    {
        BiomechanicalAnalysis::log()->error("{} Error getting the 'mu_dyn_variables' parameter.", logPrefix);
        return false;
    }

    if (!groupHandler->getParameter("cov_dyn_variables", m_extWrenchesEstimator.params.priorDynamicsRegularizationCovarianceValue))
    {
        BiomechanicalAnalysis::log()->error("{} Error getting the 'cov_dyn_variables' parameter.", logPrefix);
        return false;
    }

    std::vector<std::string> specificElements;
    if (!groupHandler->getParameter("specificElements", specificElements))
    {
        BiomechanicalAnalysis::log()->error("{} Error getting the 'measurements' parameter.", logPrefix);
        return false;
    }

    for (auto& element : specificElements)
    {
        std::vector<double> covariance;
        if (!groupHandler->getParameter(element, covariance))
        {
            BiomechanicalAnalysis::log()->error("{} Error getting the '{}' parameter.", logPrefix, element);
            return false;
        }
        m_extWrenchesEstimator.params.specificMeasurementsCovariance[element] = covariance;
    }

    if (!groupHandler->getParameter("cov_measurements_RCM_SENSOR",
                                    m_extWrenchesEstimator.params.specificMeasurementsCovariance["RCM_"
                                                                                                 "SENSOR"]))
    {
        BiomechanicalAnalysis::log()->error("{} Error getting the 'cov_measurements_RCM_SENSOR' "
                                            "parameter.",
                                            logPrefix);
        return false;
    }

    if (!groupHandler->getParameter("default_cov_measurements", m_extWrenchesEstimator.params.measurementDefaultCovariance))
    {
        BiomechanicalAnalysis::log()->error("{} Error getting the 'default_cov_measurements' "
                                            "parameter.",
                                            logPrefix);
        return false;
    }

    iDynTree::BerdyOptions berdyOptionsExtWrenches;
    berdyOptionsExtWrenches.berdyVariant = iDynTree::BerdyVariants::BERDY_FLOATING_BASE_NON_COLLOCATED_EXT_WRENCHES;
    berdyOptionsExtWrenches.includeAllNetExternalWrenchesAsSensors = true;
    berdyOptionsExtWrenches.includeAllJointTorquesAsSensors = false;
    berdyOptionsExtWrenches.includeAllJointAccelerationsAsSensors = false;
    berdyOptionsExtWrenches.includeAllNetExternalWrenchesAsSensors = true;
    berdyOptionsExtWrenches.includeAllNetExternalWrenchesAsDynamicVariables = true;

    if (!berdyOptionsExtWrenches.checkConsistency())
    {
        BiomechanicalAnalysis::log()->error("{} Error in the consistency of the BerdyOptions "
                                            "object.",
                                            logPrefix);
        return false;
    }

    if (!m_extWrenchesEstimator.berdyHelper.init(m_kinDynFullModel->getRobotModel(), berdyOptionsExtWrenches))
    {
        BiomechanicalAnalysis::log()->error("{} Error initializing the BerdyHelper object.", logPrefix);
        return false;
    }

    m_extWrenchesEstimator.berdySolver = std::make_unique<iDynTree::BerdySparseMAPSolver>(m_extWrenchesEstimator.berdyHelper);

    if (!m_extWrenchesEstimator.berdySolver->initialize())
    {
        BiomechanicalAnalysis::log()->error("{} Error initializing the BerdySparseMAPSolver "
                                            "object.",
                                            logPrefix);
        return false;
    }

    iDynTree::Triplets measurementsCovarianceMatrixTriplets;
    for (const iDynTree::BerdySensor& berdySensor : m_extWrenchesEstimator.berdyHelper.getSensorsOrdering())
    {
        switch (berdySensor.type)
        {
        case iDynTree::BerdySensorTypes::NET_EXT_WRENCH_SENSOR: {
            // initialize with default covariance
            iDynTree::Vector6 wrenchCovariance;
            for (int i = 0; i < 6; i++)
                wrenchCovariance.setVal(i, m_extWrenchesEstimator.params.measurementDefaultCovariance);

            // set specific covariance if configured
            auto specificMeasurementsPtr = m_extWrenchesEstimator.params.specificMeasurementsCovariance.find(berdySensor.id);
            if (specificMeasurementsPtr != m_extWrenchesEstimator.params.specificMeasurementsCovariance.end())
            {
                for (int i = 0; i < 6; i++)
                    wrenchCovariance.setVal(i, specificMeasurementsPtr->second[i]);
            }

            for (std::size_t i = 0; i < 6; i++)
                measurementsCovarianceMatrixTriplets.setTriplet(
                    {berdySensor.range.offset + i, berdySensor.range.offset + i, wrenchCovariance[i]});
        }
        break;
        case iDynTree::BerdySensorTypes::RCM_SENSOR: {
            auto specificMeasurementsPtr = m_extWrenchesEstimator.params.specificMeasurementsCovariance.find("RCM_"
                                                                                                             "SENS"
                                                                                                             "OR");
            for (std::size_t i = 0; i < 6; i++)
            {
                measurementsCovarianceMatrixTriplets.setTriplet(
                    {berdySensor.range.offset + i, berdySensor.range.offset + i, specificMeasurementsPtr->second[i]});
            }
        }
        break;
        default:
            break;
        }
    }
    iDynTree::SparseMatrix<iDynTree::ColumnMajor> measurementsPriorCovarianceMatrix;
    std::size_t sigmaYSize = m_extWrenchesEstimator.berdyHelper.getNrOfSensorsMeasurements();
    measurementsPriorCovarianceMatrix.resize(sigmaYSize, sigmaYSize);
    measurementsPriorCovarianceMatrix.zero();
    measurementsPriorCovarianceMatrix.setFromTriplets(measurementsCovarianceMatrixTriplets);
    m_extWrenchesEstimator.berdySolver->setMeasurementsPriorCovariance(measurementsPriorCovarianceMatrix);

    // Set mu_d
    iDynTree::VectorDynSize dynamicsRegularizationExpectedValueVector;
    dynamicsRegularizationExpectedValueVector.resize(m_extWrenchesEstimator.berdyHelper.getNrOfDynamicVariables());
    for (std::size_t i = 0; i < dynamicsRegularizationExpectedValueVector.size(); i++)
        dynamicsRegularizationExpectedValueVector.setVal(i, m_extWrenchesEstimator.params.priorDynamicsRegularizationExpected);
    m_extWrenchesEstimator.berdySolver->setDynamicsRegularizationPriorExpectedValue(dynamicsRegularizationExpectedValueVector);

    // set Sigma_d
    iDynTree::Triplets priorDynamicsRegularizationCovarianceMatrixTriplets;
    std::size_t sigmaDSize = m_extWrenchesEstimator.berdyHelper.getNrOfDynamicVariables();
    for (std::size_t i = 0; i < sigmaDSize; i++)
    {
        priorDynamicsRegularizationCovarianceMatrixTriplets.setTriplet(
            {i, i, m_extWrenchesEstimator.params.priorDynamicsRegularizationCovarianceValue});
    }
    iDynTree::SparseMatrix<iDynTree::ColumnMajor> priorDynamicsRegularizationCovarianceMatrix;
    priorDynamicsRegularizationCovarianceMatrix.resize(sigmaDSize, sigmaDSize);
    priorDynamicsRegularizationCovarianceMatrix.setFromTriplets(priorDynamicsRegularizationCovarianceMatrixTriplets);
    m_extWrenchesEstimator.berdySolver->setDynamicsRegularizationPriorCovariance(priorDynamicsRegularizationCovarianceMatrix);

    if (!m_extWrenchesEstimator.berdySolver->isValid())
    {
        BiomechanicalAnalysis::log()->error("{} Error in the initialization of the BerdySolver.", logPrefix);
        return false;
    }

    return true;
}

iDynTree::SpatialForceVector HumanID::computeRCMInBaseFrame()
{
    iDynTree::SpatialForceVector rcmWrench;
    rcmWrench.zero();
    iDynTree::Vector3 world_gravity;
    world_gravity(0) = 0.0;
    world_gravity(1) = 0.0;
    world_gravity(2) = -9.81;
    iDynTree::SpatialForceVector subjectWeightInCentroidal(world_gravity, iDynTree::AngularForceVector3(0.0, 0.0, 0.0));
    subjectWeightInCentroidal = subjectWeightInCentroidal * (-m_humanMass);

    iDynTree::Transform base_H_centroidal;
    base_H_centroidal.setPosition(m_kinDyn->getCenterOfMassPosition() - m_kinDyn->getWorldBaseTransform().getPosition());
    base_H_centroidal.setRotation(m_kinDyn->getWorldBaseTransform().getRotation().inverse());
    rcmWrench = rcmWrench + base_H_centroidal * subjectWeightInCentroidal;

    return rcmWrench;
}
