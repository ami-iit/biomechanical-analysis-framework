
#ifndef BIOMECHANICAL_ANALYSIS_DATA_SOURCES_ACCELERATION6D_H
#define BIOMECHANICAL_ANALYSIS_DATA_SOURCES_ACCELERATION6D_H

#include <Eigen/Dense>

#include <BiomechanicalAnalysis/DataSources/LinearAcceleration.h>
#include <BiomechanicalAnalysis/DataSources/AngularAcceleration.h>

namespace BiomechanicalAnalysis
{
namespace DataSources
{

template<typename Scalar>
class Acceleration6D : public virtual LinearAcceleration<Scalar>, virtual AngularAcceleration<Scalar>
{
};


} // namespace DataSources
} // namespace BiomechanicalAnalysis

#endif // BIOMECHANICAL_ANALYSIS_DATA_SOURCES_ACCELERATION6D_H
