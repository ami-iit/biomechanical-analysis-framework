#ifndef BIOMECHANICAL_ANALYSIS_IK_SOLVER_H
#define BIOMECHANICAL_ANALYSIS_IK_SOLVER_H

#include <memory>

#include <iDynTree/Model/Model.h>

#include "HumanStateSource.h"

namespace BiomechanicalAnalysis
{
namespace IK
{

template <typename Scalar>
class Solver : public virtual HumanStateSource<Scalar>
{
public:
    virtual bool setModel(std::shared_ptr<iDynTree::Model>& model) = 0;
    virtual bool setBaseFrame(const std::string& baseFrame) = 0;
    virtual bool initialize() = 0;
    virtual bool update() = 0;
};
    
} // namespace IK    
} // namespace BiomechanicalAnalysis

#endif // BIOMECHANICAL_ANALYSIS_IK_SOLVER_H