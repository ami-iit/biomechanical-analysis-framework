
#ifndef BIOMECHANICAL_ANALYSIS_DATA_SOURCES_LINEAR_ACCELERATION_H
#define BIOMECHANICAL_ANALYSIS_DATA_SOURCES_LINEAR_ACCELERATION_H

#include <memory>

#include <Eigen/Dense>

namespace BiomechanicalAnalysis
{
namespace DataSources
{

template<typename Scalar>
class LinearAcceleration
{

public:
    virtual Eigen::Ref<Eigen::Vector3<Scalar>> getLinearAcceleration() const = 0;
};

} // namespace DataSources
} // namespace BiomechanicalAnalysis

#endif // BIOMECHANICAL_ANALYSIS_DATA_SOURCES_LINEAR_ACCELERATION_H
