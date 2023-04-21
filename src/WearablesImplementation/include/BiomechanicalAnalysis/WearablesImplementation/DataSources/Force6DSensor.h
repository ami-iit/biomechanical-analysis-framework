
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

class Force6DSensor final : public Force6D<double>
{
public:
    Force6DSensor(const std::shared_ptr<wearable::sensor::IForceTorque6DSensor>& force6DSensor);

    bool getForce6D(Eigen::Ref<Eigen::Matrix<double,6,1>> wrench) const override;

    bool update();

private:
    wearable::Vector6 _wVector;
    std::shared_ptr<wearable::sensor::IForceTorque6DSensor> _force6DSensor;
};

} // namespace Wearables
} // namespace DataSources
} // namespace BiomechanicalAnalysis

#endif // BIOMECHANICAL_ANALYSIS_DATA_SOURCES_WEARABLES_FORCE6D_H
