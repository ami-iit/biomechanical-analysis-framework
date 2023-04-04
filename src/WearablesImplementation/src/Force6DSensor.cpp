
#include <BiomechanicalAnalysis/WearablesImplementation/DataSources/Force6DSensor.h>

using namespace BiomechanicalAnalysis::DataSources::Wearables;

Eigen::Ref<Eigen::Matrix<double,6,1>> Force6DSensor::getForce6D() const 
{
    return _wrenchRef;
}

bool Force6DSensor::update()
{
    // get wearable data
    wearable::Vector6 wVector;
    if(!_force6DSensor->getForceTorque6D(wVector))
    {
        return false;
    }

    // fill vector
    _wrench << wVector[0], wVector[1], wVector[2], wVector[3], wVector[4], wVector[5]; 

    return true;
}
