/**
 * @file InverseKinematic.h
 * @authors Davide Gorbani <davide.gorbani@iit.it>
 */

#ifndef BIOMECHANICAL_ANALYSIS_INVERSE_DYNAMICS_H
#define BIOMECHANICAL_ANALYSIS_INVERSE_DYNAMICS_H

#include <iDynTree/BerdyHelper.h>
#include <iDynTree/BerdySparseMAPSolver.h>
#include <iDynTree/KinDynComputations.h>
#include <memory>

namespace BiomechanicalAnalysis
{
namespace ID
{

class HumanID
{
private:
    // some private members
    std::shared_ptr<iDynTree::KinDynComputations> m_kinDyn; /** pointer to the KinDynComputations
                                                               object */
public:
    // some public members
    bool intizialize(std::shared_ptr<iDynTree::KinDynComputations> kinDyn);
};

} // namespace ID

} // namespace BiomechanicalAnalysis

#endif // BIOMECHANICAL_ANALYSIS_INVERSE_DYNAMICS_H
