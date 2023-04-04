
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
    virtual Eigen::Ref<Eigen::Vector3<Scalar>> getAngularAcceleration() const = 0;
};

// template<typename Scalar>
// class StdAngularAcceleration : public AngularAcceleration<Scalar>
// {
// public:

//     StdAngularAcceleration(){};

//     StdAngularAcceleration(Eigen::Ref<Eigen::Vector3<Scalar>> angularAcceleration)
//     {
//         _angularAcceleration = angularAcceleration;
//     }

//     void setAngularAcceleration(Eigen::Ref<Eigen::Vector3<Scalar>> angularAcceleration)
//     {
//         _angularAcceleration = angularAcceleration;
//     }

//     bool getAngularAcceleration(Eigen::Ref<Eigen::Vector3<Scalar>> angularAcceleration) const override
//     {
//         angularAcceleration = _angularAcceleration;
//         return true;
//     }

// private:
//     Eigen::Vector3<Scalar> _angularAcceleration;
// };

//template<typename Scalar>
//class ConstAngularAcceleration : public virtual AngularAcceleration<Scalar>
//{
//public:
//    ConstAngularAcceleration(const Eigen::Ref<const Eigen::Vector3<Scalar>> angularAcceleration) : angularAcceleration{angularAcceleration}{};
//
//private:
//    const Eigen::Vector3<Scalar> angularAcceleration;
//};


} // namespace DataSources
} // namespace BiomechanicalAnalysis

#endif // BIOMECHANICAL_ANALYSIS_DATA_SOURCES_ANGULAR_ACCELERATION_H
