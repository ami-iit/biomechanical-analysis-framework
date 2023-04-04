
#ifndef BIOMECHANICAL_ANALYSIS_DATA_SOURCES_WEARABLES_VLINK_H
#define BIOMECHANICAL_ANALYSIS_DATA_SOURCES_WEARABLES_VLINK_H

#include <memory>

#include <Wearable/IWear/IWear.h>

#include <BiomechanicalAnalysis/DataSources/Pose.h>
#include <BiomechanicalAnalysis/DataSources/Velocity6D.h>
#include <BiomechanicalAnalysis/DataSources/Acceleration6D.h>

namespace BiomechanicalAnalysis
{
namespace DataSources
{
namespace Wearables
{

class VirtualLinkSensor : public Pose<double>, Velocity6D<double>, Acceleration6D<double>
{
public:
    VirtualLinkSensor(const std::shared_ptr<wearable::sensor::IVirtualLinkKinSensor>& virtualLinkSensor) : _virtualLinkSensor{virtualLinkSensor}
    {}

    virtual Eigen::Ref<Eigen::Vector3<double>> getPosition() const override;
    virtual Eigen::Ref<Eigen::Matrix3<double>> getOrientation() const override;
    virtual Eigen::Ref<Eigen::Vector3<double>> getLinearVelocity() const override;
    virtual Eigen::Ref<Eigen::Vector3<double>> getAngularVelocity() const override;
    virtual Eigen::Ref<Eigen::Vector3<double>> getLinearAcceleration() const override;
    virtual Eigen::Ref<Eigen::Vector3<double>> getAngularAcceleration() const override;

    bool update();

private:
    std::shared_ptr<wearable::sensor::IVirtualLinkKinSensor> _virtualLinkSensor;
    Eigen::Vector3<double> _position = Eigen::Vector3<double>::Zero();
    Eigen::Ref<Eigen::Vector3<double>> _positionRef = _position.block<3,1>(0,0);
    Eigen::Matrix3<double> _orientation;
    Eigen::Ref<Eigen::Matrix3<double>> _orientationRef = _orientation.block<3,3>(0,0);

    Eigen::Matrix<double,6,1> _velocity = Eigen::Matrix<double,6,1>::Zero();
    Eigen::Ref<Eigen::Vector3<double>> _linVelocityRef = _velocity.block<3,1>(0,0);
    Eigen::Ref<Eigen::Vector3<double>> _angVelocityRef = _velocity.block<3,1>(3,0);

    Eigen::Matrix<double,6,1> _acceleration = Eigen::Matrix<double,6,1>::Zero();
    Eigen::Ref<Eigen::Vector3<double>> _linAccelerationRef = _acceleration.block<3,1>(0,0);
    Eigen::Ref<Eigen::Vector3<double>> _angAccelerationRef = _acceleration.block<3,1>(0,0);
    
};

} // namespace DataSources
} // namespace Wearables
} // namespace BiomechanicalAnalysis

#endif // BIOMECHANICAL_ANALYSIS_DATA_SOURCES_WEARABLES_VLINK_H
