
#ifndef BIOMECHANICAL_ANALYSIS_DATA_SOURCES_POSE_H
#define BIOMECHANICAL_ANALYSIS_DATA_SOURCES_POSE_H

#include <Eigen/Dense>

#include <BiomechanicalAnalysis/DataSources/Position.h>
#include <BiomechanicalAnalysis/DataSources/Orientation.h>

namespace BiomechanicalAnalysis
{
namespace DataSources
{

template<typename Scalar>
class Pose : public Position<Scalar>, public Orientation<Scalar>
{

};


} // namespace DataSources
} // namespace BiomechanicalAnalysis

#endif // BIOMECHANICAL_ANALYSIS_DATA_SOURCES_POSE_H
