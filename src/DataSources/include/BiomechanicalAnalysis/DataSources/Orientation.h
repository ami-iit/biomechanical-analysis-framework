
#ifndef BIOMECHANICAL_ANALYSIS_DATA_SOURCES_ORIENTATION_H
#define BIOMECHANICAL_ANALYSIS_DATA_SOURCES_ORIENTATION_H

#include <memory>

#include <Eigen/Geometry> 

namespace BiomechanicalAnalysis
{
namespace DataSources
{

template<typename Scalar>
class Orientation
{

public:
    virtual bool getOrientation(Eigen::Quaternion<Scalar>& orientation) const = 0;
};


template<typename Scalar>
class ConstOrientation final : public Orientation<Scalar>
{

public:

    ConstOrientation(const Eigen::Quaternion<Scalar>& quat) : _orientation{quat} {}

    bool getOrientation(Eigen::Quaternion<Scalar>& orientation) const override
    {
        orientation = _orientation;
        return true;
    }

private:
    Eigen::Quaternion<Scalar> _orientation;
    
};


} // namespace DataSources
} // namespace BiomechanicalAnalysis

#endif // BIOMECHANICAL_ANALYSIS_DATA_SOURCES_ORIENTATION_H
