
#ifndef BIOMECHANICAL_ANALYSIS_DATA_SOURCES_LINEAR_VELOCITY_H
#define BIOMECHANICAL_ANALYSIS_DATA_SOURCES_LINEAR_VELOCITY_H

#include <memory>

#include <Eigen/Dense>

namespace BiomechanicalAnalysis
{
namespace DataSources
{

template<typename Scalar>
class LinearVelocity
{

public:
    virtual Eigen::Ref<Eigen::Vector3<Scalar>> getLinearVelocity() const = 0;
};




} // namespace DataSources
} // namespace BiomechanicalAnalysis

#endif // BIOMECHANICAL_ANALYSIS_DATA_SOURCES_LINEAR_VELOCITY_H
