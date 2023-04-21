
#ifndef BIOMECHANICAL_ANALYSIS_DATA_SOURCES_ANGULAR_ACCELERATION_H
#define BIOMECHANICAL_ANALYSIS_DATA_SOURCES_ANGULAR_ACCELERATION_H

#include <memory>

#include <Eigen/Core>


namespace BiomechanicalAnalysis
{
namespace DataSources
{

template<typename Scalar>
class AngularAcceleration
{

public:
    virtual bool getAngularAcceleration(Eigen::Ref<Eigen::Vector3<Scalar>> angularAcceleration) const = 0;
};

} // namespace DataSources
} // namespace BiomechanicalAnalysis

#endif // BIOMECHANICAL_ANALYSIS_DATA_SOURCES_ANGULAR_ACCELERATION_H
