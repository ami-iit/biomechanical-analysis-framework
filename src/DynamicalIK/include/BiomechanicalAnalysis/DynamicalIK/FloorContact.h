
#ifndef BIOMECHANICAL_ANALYSIS_WEARABLE_TARGETS_H
#define BIOMECHANICAL_ANALYSIS_WEARABLE_TARGETS_H

#include "Target.h"

#include <BiomechanicalAnalysis/DataSources/Force6D.h>

#include <memory>

namespace BiomechanicalAnalysis
{
namespace DynamicalIK
{


// TODO
class FloorContact : public PositionTarget
{
public:

private:
    std::shared_ptr<BiomechanicalAnalysis::DataSources::Force6D<double>> forceSensor;

};

}
}


#endif //BIOMECHANICAL_ANALYSIS_WEARABLE_TARGETS
