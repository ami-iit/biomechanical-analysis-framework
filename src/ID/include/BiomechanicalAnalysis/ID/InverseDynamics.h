/**
 * @file InverseKinematic.h
 * @authors Davide Gorbani <davide.gorbani@iit.it>
 */

#ifndef BIOMECHANICAL_ANALYSIS_INVERSE_DYNAMICS_H
#define BIOMECHANICAL_ANALYSIS_INVERSE_DYNAMICS_H

#include <memory>

#include <iDynTree/BerdyHelper.h>
#include <iDynTree/BerdySparseMAPSolver.h>
#include <iDynTree/KinDynComputations.h>

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

class HumanID
{
private:
    std::shared_ptr<iDynTree::KinDynComputations> m_kinDyn; /** pointer to the KinDynComputations
                                                               object */
    iDynTree::BerdyHelper m_berdyHelper; /** BerdyHelper object */
    std::unique_ptr<iDynTree::BerdySparseMAPSolver> m_berdySolver
        = nullptr; /** BerdySparseMAPSolver object */
    iDynTree::VectorDynSize m_estimatedDynamicVariables;
    iDynTree::VectorDynSize m_estimatedJointTorques;
    iDynTree::VectorDynSize m_measurement;
    KinematicState m_kinState;

public:
    bool initialize(std::shared_ptr<iDynTree::KinDynComputations> kinDyn);
    bool solve();
    iDynTree::VectorDynSize getJointTorques();
};

} // namespace ID

} // namespace BiomechanicalAnalysis

#endif // BIOMECHANICAL_ANALYSIS_INVERSE_DYNAMICS_H
