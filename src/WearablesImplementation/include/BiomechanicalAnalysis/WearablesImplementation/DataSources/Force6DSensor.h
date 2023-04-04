
#ifndef BIOMECHANICAL_ANALYSIS_DATA_SOURCES_WEARABLES_FORCE6D_H
#define BIOMECHANICAL_ANALYSIS_DATA_SOURCES_WEARABLES_FORCE6D_H

#include <Wearable/IWear/IWear.h>

#include <BiomechanicalAnalysis/DataSources/Force6D.h>

namespace BiomechanicalAnalysis
{
namespace DataSources
{
namespace Wearables
{

class Force6DSensor final : public virtual Force6D<double>
{
public:
    Force6DSensor(const std::shared_ptr<wearable::sensor::IForceTorque6DSensor>& force6DSensor) : _force6DSensor{force6DSensor}
    {}

    Eigen::Ref<Eigen::Matrix<double,6,1>> getForce6D() const override;


    bool update();

private:
    Eigen::Matrix<double,6,1> _wrench = Eigen::Matrix<double,6,1>::Zero();
    Eigen::Ref<Eigen::Matrix<double,6,1>> _wrenchRef = _wrench.block<6,1>(0,0);
    std::shared_ptr<wearable::sensor::IForceTorque6DSensor> _force6DSensor;
};

} // namespace Wearables
} // namespace DataSources
} // namespace BiomechanicalAnalysis

#endif // BIOMECHANICAL_ANALYSIS_DATA_SOURCES_WEARABLES_FORCE6D_H
