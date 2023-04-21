#ifndef BIOMECHANICAL_ANALYSIS_IK_INTEGRATION_BASED_SOLVER_H
#define BIOMECHANICAL_ANALYSIS_IK_INTEGRATION_BASED_SOLVER_H

#include "Solver.h"

namespace BiomechanicalAnalysis
{
namespace IK
{

template <typename Scalar>
class IntegrationBasedSolver : public virtual Solver<Scalar>
{
public:
    virtual void setStep(const double dt) = 0;
    virtual double getStep() = 0;
};
    
} // namespace IK    
} // namespace BiomechanicalAnalysis

#endif // BIOMECHANICAL_ANALYSIS_IK_INTEGRATION_BASED_SOLVER_H