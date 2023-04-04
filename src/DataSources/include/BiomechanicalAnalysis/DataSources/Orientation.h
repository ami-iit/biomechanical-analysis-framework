
#ifndef BIOMECHANICAL_ANALYSIS_DATA_SOURCES_ORIENTATION_H
#define BIOMECHANICAL_ANALYSIS_DATA_SOURCES_ORIENTATION_H

#include <memory>

#include <Eigen/Core>

namespace BiomechanicalAnalysis
{
namespace DataSources
{

template<typename Scalar>
class Orientation
{

public:
    virtual Eigen::Ref<Eigen::Matrix3<Scalar>> getOrientation() const = 0;
};

} // namespace DataSources
} // namespace BiomechanicalAnalysis

#endif // BIOMECHANICAL_ANALYSIS_DATA_SOURCES_ORIENTATION_H
