#ifndef BIOMECHANICAL_ANALYSIS_IK_INTEGRATION_BASED_SOLVER_H
#define BIOMECHANICAL_ANALYSIS_IK_INTEGRATION_BASED_SOLVER_H

#include "Solver.h"

namespace BiomechanicalAnalysis
{
namespace IK
{

template <typename Scalar>
class IntegrationBasedSolver : public Solver<Scalar>
{
public:
    inline virtual void setStep(const double dt){_dt = dt;}

protected:
    double _dt = 0;
};
    
} // namespace IK    
} // namespace BiomechanicalAnalysis

#endif // BIOMECHANICAL_ANALYSIS_IK_INTEGRATION_BASED_SOLVER_H