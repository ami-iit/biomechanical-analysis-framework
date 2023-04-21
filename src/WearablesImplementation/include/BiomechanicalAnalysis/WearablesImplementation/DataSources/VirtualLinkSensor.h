
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

class VirtualLinkSensor : public Pose<double>, public Velocity6D<double>, public Acceleration6D<double>
{
public:
    VirtualLinkSensor(const std::shared_ptr<const wearable::sensor::IVirtualLinkKinSensor>& virtualLinkSensor);

    bool getPosition(Eigen::Ref<Eigen::Vector3<double>> position) const override;
    bool getOrientation(Eigen::Quaterniond& orientation) const override;
    bool getLinearVelocity(Eigen::Ref<Eigen::Vector3<double>> linearVelocity) const override;
    bool getAngularVelocity(Eigen::Ref<Eigen::Vector3<double>> angularVelocity) const override;
    bool getLinearAcceleration(Eigen::Ref<Eigen::Vector3<double>> linearAcceleration) const override;
    bool getAngularAcceleration(Eigen::Ref<Eigen::Vector3<double>> angularAcceleration) const override;

    bool update();

private:
    std::shared_ptr<const wearable::sensor::IVirtualLinkKinSensor> _virtualLinkSensor;
    wearable::Vector3 _wPos;
    wearable::Vector3 _wLinVel;
    wearable::Vector3 _wAngVel;
    wearable::Vector3 _wAngAcc;
    wearable::Vector3 _wLinAcc;
    Eigen::Quaterniond _orientation;    
};

} // namespace DataSources
} // namespace Wearables
} // namespace BiomechanicalAnalysis

#endif // BIOMECHANICAL_ANALYSIS_DATA_SOURCES_WEARABLES_VLINK_H
