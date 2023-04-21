
#include <BiomechanicalAnalysis/WearablesImplementation/DataSources/Force6DSensor.h>

using namespace BiomechanicalAnalysis::DataSources::Wearables;

Force6DSensor::Force6DSensor(const std::shared_ptr<wearable::sensor::IForceTorque6DSensor>& force6DSensor): _force6DSensor{force6DSensor}
{
    _wVector.fill(0);
}

bool Force6DSensor::getForce6D(Eigen::Ref<Eigen::Matrix<double,6,1>> wrench) const 
{
    // fill vector
    wrench << _wVector[0], _wVector[1], _wVector[2], _wVector[3], _wVector[4], _wVector[5];
    return true;
}

bool Force6DSensor::update()
{
    // get wearable data
    if(!_force6DSensor->getForceTorque6D(_wVector))
    {
        return false;
    }

    return true;
}
