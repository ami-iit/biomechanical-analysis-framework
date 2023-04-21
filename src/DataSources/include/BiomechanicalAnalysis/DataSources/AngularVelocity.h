
#ifndef BIOMECHANICAL_ANALYSIS_DATA_SOURCES_ANGULAR_VELOCITY_H
#define BIOMECHANICAL_ANALYSIS_DATA_SOURCES_ANGULAR_VELOCITY_H

#include <memory>

#include <Eigen/Dense>

namespace BiomechanicalAnalysis
{
namespace DataSources
{

template<typename Scalar>
class AngularVelocity
{

public:
    virtual bool getAngularVelocity(Eigen::Ref<Eigen::Vector3<Scalar>> angularVelocity) const = 0;
};


} // namespace DataSources
} // namespace BiomechanicalAnalysis

#endif // BIOMECHANICAL_ANALYSIS_DATA_SOURCES_ANGULAR_VELOCITY_H
