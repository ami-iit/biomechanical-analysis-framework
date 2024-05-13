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

class HumanID
{
private:
    std::shared_ptr<iDynTree::KinDynComputations> m_kinDyn; /** pointer to the KinDynComputations
                                                               object */
    MAPHelper m_extWrenchesHelper;
    MAPHelper m_jointTorquesHelper;
    std::vector<WrenchSourceData> m_wrenchSources;
    KinematicState m_kinState;
    MAPEstParams m_mapEstParams;
    std::vector<iDynTree::Wrench> m_estimatedExtWrenches;

    bool initializeJointTorquesHelper();
    bool initializeExtWrenchesHelper(
        const std::shared_ptr<BipedalLocomotion::ParametersHandler::IParametersHandler> taskHandler);

public:
    bool
    initialize(std::weak_ptr<const BipedalLocomotion::ParametersHandler::IParametersHandler> handler,
               std::shared_ptr<iDynTree::KinDynComputations> kinDyn);
    bool
    updateExtWrenchesMeasurements(const std::unordered_map<std::string, iDynTree::Wrench>& wrenches);
    bool solve();
    iDynTree::VectorDynSize getJointTorques();
};

} // namespace ID

} // namespace BiomechanicalAnalysis

#endif // BIOMECHANICAL_ANALYSIS_INVERSE_DYNAMICS_H
