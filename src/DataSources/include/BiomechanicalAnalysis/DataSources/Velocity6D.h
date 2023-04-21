
#ifndef BIOMECHANICAL_ANALYSIS_DATA_SOURCES_VELOCITY6D_H
#define BIOMECHANICAL_ANALYSIS_DATA_SOURCES_VELOCITY6D_H

#include <Eigen/Dense>

#include <BiomechanicalAnalysis/DataSources/LinearVelocity.h>
#include <BiomechanicalAnalysis/DataSources/AngularVelocity.h>

namespace BiomechanicalAnalysis
{
namespace DataSources
{

template<typename Scalar>
class Velocity6D : public LinearVelocity<Scalar>, public AngularVelocity<Scalar>{};


} // namespace DataSources
} // namespace BiomechanicalAnalysis

#endif // BIOMECHANICAL_ANALYSIS_DATA_SOURCES_VELOCITY6D_H
