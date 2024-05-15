/**
 * @file InverseDynamics.h
 * @authors Davide Gorbani <davide.gorbani@iit.it>
 */

#ifndef BIOMECHANICAL_ANALYSIS_INVERSE_DYNAMICS_H
#define BIOMECHANICAL_ANALYSIS_INVERSE_DYNAMICS_H

#include <memory>

#include <iDynTree/BerdyHelper.h>
#include <iDynTree/BerdySparseMAPSolver.h>
#include <iDynTree/Core/SparseMatrix.h>
#include <iDynTree/KinDynComputations.h>

#include <BipedalLocomotion/ParametersHandler/IParametersHandler.h>
#include <BipedalLocomotion/ParametersHandler/StdImplementation.h>

namespace BiomechanicalAnalysis
{
namespace ID
{

struct KinematicState
{
    iDynTree::FrameIndex floatingBaseFrameIndex;
    iDynTree::Vector3 baseAngularVelocity;
    iDynTree::JointPosDoubleArray jointsPosition;
    iDynTree::JointDOFsDoubleArray jointsVelocity;
};

struct MAPHelper
{
    iDynTree::BerdyHelper berdyHelper; /** BerdyHelper object */
    std::unique_ptr<iDynTree::BerdySparseMAPSolver> berdySolver = nullptr; /** BerdySparseMAPSolver
                                                                              object */
    iDynTree::VectorDynSize estimatedDynamicVariables;
    iDynTree::VectorDynSize estimatedJointTorques;
    iDynTree::VectorDynSize measurement;
};

struct MAPEstParams
{
    double priorDynamicsRegularizationExpected; // mu_d
    double priorDynamicsRegularizationCovarianceValue; // Sigma_d
    // measurements params
    double measurementDefaultCovariance;
    std::unordered_map<std::string, std::vector<double>> specificMeasurementsCovariance;
};

enum class WrenchSourceType
{
    Fixed,
    Dummy,
};

struct WrenchSourceData
{
    WrenchSourceType type;
    std::string outputFrame;
    iDynTree::Transform outputFrameTransform;
    iDynTree::Wrench wrench;
};

struct WrenchEstimationStruct
{
    MAPHelper helper;
    std::shared_ptr<iDynTree::KinDynComputations> kinDyn;
    iDynTree::VectorDynSize jointPositions;
    iDynTree::VectorDynSize jointVelocities;
    iDynTree::JointPosDoubleArray jointsPositionArray;
    iDynTree::JointDOFsDoubleArray jointsVelocityArray;
    bool useFullModel;
};

class HumanID
{
private:
    std::shared_ptr<iDynTree::KinDynComputations> m_kinDyn; /** pointer to the KinDynComputations
                                                               object */
    WrenchEstimationStruct m_extWrenchesEstimator;
    MAPHelper m_jointTorquesHelper;
    std::vector<WrenchSourceData> m_wrenchSources;
    KinematicState m_kinState;
    MAPEstParams m_mapEstParams;
    std::vector<iDynTree::Wrench> m_estimatedExtWrenches;
    double m_humanMass;

    bool initializeJointTorquesHelper(
        const std::shared_ptr<BipedalLocomotion::ParametersHandler::IParametersHandler>
            groupHandler);
    bool initializeExtWrenchesHelper(
        const std::shared_ptr<BipedalLocomotion::ParametersHandler::IParametersHandler>
            groupHandler);

    iDynTree::SpatialForceVector computeRCMInBaseFrame();

    const std::unordered_map<iDynTree::BerdySensorTypes, std::string> mapBerdySensorType
        = {{iDynTree::BerdySensorTypes::SIX_AXIS_FORCE_TORQUE_SENSOR,
            "SIX_AXIS_FORCE_TORQUE_SENSOR"},
           {iDynTree::BerdySensorTypes::ACCELEROMETER_SENSOR, "ACCELEROMETER_SENSOR"},
           {iDynTree::BerdySensorTypes::GYROSCOPE_SENSOR, "GYROSCOPE_SENSOR"},
           {iDynTree::BerdySensorTypes::THREE_AXIS_ANGULAR_ACCELEROMETER_SENSOR,
            "THREE_AXIS_ANGULAR_ACCELEROMETER_SENSOR"},
           {iDynTree::BerdySensorTypes::THREE_AXIS_FORCE_TORQUE_CONTACT_SENSOR,
            "THREE_AXIS_FORCE_TORQUE_CONTACT_SENSOR"},
           {iDynTree::BerdySensorTypes::DOF_ACCELERATION_SENSOR, "DOF_ACCELERATION_SENSOR"},
           {iDynTree::BerdySensorTypes::DOF_TORQUE_SENSOR, "DOF_TORQUE_SENSOR"},
           {iDynTree::BerdySensorTypes::NET_EXT_WRENCH_SENSOR, "NET_EXT_WRENCH_SENSOR"},
           {iDynTree::BerdySensorTypes::JOINT_WRENCH_SENSOR, "JOINT_WRENCH_SENSOR"}};

public:
    bool
    initialize(std::weak_ptr<const BipedalLocomotion::ParametersHandler::IParametersHandler> handler,
               std::shared_ptr<iDynTree::KinDynComputations> kinDyn);
    bool
    updateExtWrenchesMeasurements(const std::unordered_map<std::string, iDynTree::Wrench>& wrenches);
    bool solve();
    iDynTree::VectorDynSize getJointTorques();
    std::vector<iDynTree::Wrench> getEstimatedExtWrenches();
};

} // namespace ID

} // namespace BiomechanicalAnalysis

#endif // BIOMECHANICAL_ANALYSIS_INVERSE_DYNAMICS_H
