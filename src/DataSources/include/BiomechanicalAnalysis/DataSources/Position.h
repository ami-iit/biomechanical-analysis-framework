
#ifndef BIOMECHANICAL_ANALYSIS_DATA_SOURCES_POSITION_H
#define BIOMECHANICAL_ANALYSIS_DATA_SOURCES_POSITION_H

#include <memory>

#include <Eigen/Core>

namespace BiomechanicalAnalysis
{
namespace DataSources
{

template<typename Scalar>
class Position
{
public:
    virtual bool getPosition(Eigen::Ref<Eigen::Vector3<Scalar>> position) const = 0;
};

} // namespace DataSources
} // namespace BiomechanicalAnalysis

#endif // BIOMECHANICAL_ANALYSIS_DATA_SOURCES_POSITION_H
