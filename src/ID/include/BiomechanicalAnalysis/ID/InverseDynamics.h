/**
 * @file InverseDynamics.h
 * @authors Davide Gorbani <davide.gorbani@iit.it>
 */

#ifndef BIOMECHANICAL_ANALYSIS_INVERSE_DYNAMICS_H
#define BIOMECHANICAL_ANALYSIS_INVERSE_DYNAMICS_H

#include <memory>

// iDynTree headers
#include <iDynTree/BerdyHelper.h>
#include <iDynTree/BerdySparseMAPSolver.h>
#include <iDynTree/Core/SparseMatrix.h>
#include <iDynTree/KinDynComputations.h>

// BipedalLocomotion headers
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

struct MAPEstParams
{
    double priorDynamicsRegularizationExpected; // mu_d
    double priorDynamicsRegularizationCovarianceValue; // Sigma_d
    // measurements params
    double measurementDefaultCovariance;
    std::unordered_map<std::string, std::vector<double>> specificMeasurementsCovariance;
};

struct MAPHelper
{
    iDynTree::BerdyHelper berdyHelper; /** BerdyHelper object */
    std::unique_ptr<iDynTree::BerdySparseMAPSolver> berdySolver = nullptr; /** BerdySparseMAPSolver
                                                                              object */
    iDynTree::VectorDynSize estimatedDynamicVariables;
    iDynTree::VectorDynSize estimatedJointTorques;
    iDynTree::VectorDynSize measurement;
    MAPEstParams params;
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

/**
 * @brief Class to compute the inverse dynamics of a human model
 */
class HumanID
{
private:
    std::shared_ptr<iDynTree::KinDynComputations> m_kinDyn; /** pointer to the KinDynComputations
                                                               object passed in the initialize
                                                               function*/
    std::shared_ptr<iDynTree::KinDynComputations> m_kinDynFullModel; /** pointer to the
                                                                        KinDynComputations object
                                                                        with the full list of joints
                                                                      */
    bool m_useFullModel; /** flag to use the full model for the inverse dynamics */
    MAPHelper m_extWrenchesEstimator; /** MAPHelper object for the estimation of the external
                                         wrenches */
    MAPHelper m_jointTorquesHelper; /** MAPHelper object for the estimation of the joint torques */
    std::vector<WrenchSourceData> m_wrenchSources; /** vector of WrenchSourceData objects */
    KinematicState m_kinState; /** KinematicState object */
    std::vector<iDynTree::Wrench> m_estimatedExtWrenches; /** vector of estimated external wrenches
                                                           */
    double m_humanMass; /** mass of the human */
    std::string m_modelPath; /** path to the urdf model file */

    /**
     * @brief Function to initialize the MAPHelper m_jointTorquesHelper object
     * @param groupHandler pointer to the ParametersHandler object
     * @return true if the initialization is successful, false otherwise
     */
    bool initializeJointTorquesHelper(
        const std::shared_ptr<BipedalLocomotion::ParametersHandler::IParametersHandler>
            groupHandler);

    /**
     * @brief Function to initialize the MAPHelper m_extWrenchesEstimator object
     * @param groupHandler pointer to the ParametersHandler object
     * @return true if the initialization is successful, false otherwise
     */
    bool initializeExtWrenchesHelper(
        const std::shared_ptr<BipedalLocomotion::ParametersHandler::IParametersHandler>
            groupHandler);

    /**
     * @brief Function to compute the rate of change of the momentum calculated in the base frame
     * @return true if the initialization is successful, false otherwise
     */
    iDynTree::SpatialForceVector computeRCMInBaseFrame();

    /**
     * Unordered map that maps the BerdySensorTypes to the corresponding string
     */
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
    /**
     * @brief Constructor
     */
    HumanID(){};

    /**
     * @brief Destructor
     */
    ~HumanID(){};

    /**
     * @brief Function to initialize the HumanID object
     * @param handler pointer to the ParametersHandler object
     * @param kinDyn pointer to the KinDynComputations object
     * @return true if the initialization is successful, false otherwise
     * @note an example of the required parameters can be found in
     * https://github.com/ami-iit/biomechanical-analysis-framework/tree/main/src/examples/ID
     */
    bool
    initialize(std::weak_ptr<const BipedalLocomotion::ParametersHandler::IParametersHandler> handler,
               std::shared_ptr<iDynTree::KinDynComputations> kinDyn);

    /**
     * @brief Function to update the measurements of the external wrenches
     * @param wrenches unordered map mapping the name of the wrench source to the wrench
     */
    bool
    updateExtWrenchesMeasurements(const std::unordered_map<std::string, iDynTree::Wrench>& wrenches);

    /**
     * @brief Function to solve the inverse dynamics problem
     * @return true if the solution is successful, false otherwise
     */
    bool solve();

    /**
     * @brief Function to get the estimated joint torques
     * @return vector of joint torques
     */
    iDynTree::VectorDynSize getJointTorques();

    /**
     * @brief Function to get the list of the joints
     * @return vector of joint names
     */
    std::vector<std::string> getJointsList();

    /**
     * @brief Function to get the estimated external wrenches
     * @return vector of external wrenches
     */
    std::vector<iDynTree::Wrench> getEstimatedExtWrenches();
};

} // namespace ID

} // namespace BiomechanicalAnalysis

#endif // BIOMECHANICAL_ANALYSIS_INVERSE_DYNAMICS_H
