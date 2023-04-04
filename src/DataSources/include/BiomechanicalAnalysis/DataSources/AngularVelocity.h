
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
    virtual Eigen::Ref<Eigen::Vector3<Scalar>> getAngularVelocity() const = 0;
};
// 
// template<typename Scalar>
// class StdAngularVelocity : public AngularVelocity<Scalar>
// {
// public:
// 
    // StdAngularVelocity(){};
// 
    // StdAngularVelocity(Eigen::Ref<Eigen::Vector3<Scalar>> angularVelocity)
    // {
        // _angularVelocity = angularVelocity;
    // }
// 
    // void setAngularVelocity(Eigen::Ref<Eigen::Vector3<Scalar>> angularVelocity)
    // {
        // _angularVelocity = angularVelocity;
    // }
// 
    // bool getAngularVelocity(Eigen::Ref<Eigen::Vector3<Scalar>> angularVelocity) const override
    // {
        // angularVelocity = _angularVelocity;
        // return true;
    // }

// private:
//     Eigen::Vector3<Scalar> _angularVelocity;
// };

} // namespace DataSources
} // namespace BiomechanicalAnalysis

#endif // BIOMECHANICAL_ANALYSIS_DATA_SOURCES_ANGULAR_VELOCITY_H
