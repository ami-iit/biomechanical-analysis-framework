#include <BiomechanicalAnalysis/WearablesImplementation/DataSources/VirtualLinkSensor.h>

#include <Eigen/Geometry>

using namespace BiomechanicalAnalysis::DataSources::Wearables;

VirtualLinkSensor::VirtualLinkSensor(const std::shared_ptr<const wearable::sensor::IVirtualLinkKinSensor>& virtualLinkSensor) : _virtualLinkSensor{virtualLinkSensor}
{
    _wPos.fill(0);
    _wAngAcc.fill(0);
    _wAngVel.fill(0);
    _wLinAcc.fill(0);
    _wLinVel.fill(0);
    _orientation.setIdentity();
}

bool VirtualLinkSensor::update()
{
    // get wearables position
    if(!_virtualLinkSensor->getLinkPosition(_wPos)){
        return false;
    }

    if(!_virtualLinkSensor->getLinkLinearVelocity(_wLinVel)){
        return false;
    }

    wearable::Quaternion wQuat;
    if(!_virtualLinkSensor->getLinkOrientation(wQuat)){
        return false;
    }

    if(!_virtualLinkSensor->getLinkAngularVelocity(_wAngVel)){
        return false;
    }

    if(!_virtualLinkSensor->getLinkAngularAcceleration(_wAngAcc)){
        return false;
    }

    if(!_virtualLinkSensor->getLinkLinearAcceleration(_wLinAcc)){
        return false;
    }

    // get quaternion
    _orientation.coeffs() << wQuat[0],wQuat[1], wQuat[2], wQuat[3];

    return true;
}

bool VirtualLinkSensor::getPosition(Eigen::Ref<Eigen::Vector3<double>> position) const
{
    position << _wPos[0], _wPos[1], _wPos[2];
    return true;
}

bool VirtualLinkSensor::getOrientation(Eigen::Quaternion<double>& orientation) const
{
    orientation = _orientation;
    return true;
}


bool VirtualLinkSensor::getLinearVelocity(Eigen::Ref<Eigen::Vector3<double>> linearVelocity) const
{
    linearVelocity << _wLinVel[0], _wLinVel[1], _wLinVel[2];
    return true;
}

bool VirtualLinkSensor::getAngularVelocity(Eigen::Ref<Eigen::Vector3<double>> angularVelocity) const
{
    angularVelocity << _wAngVel[0], _wAngVel[1], _wAngVel[2];
    return true;
}

bool VirtualLinkSensor::getLinearAcceleration(Eigen::Ref<Eigen::Vector3<double>> linearAcceleration) const
{
    linearAcceleration << _wLinAcc[0], _wLinAcc[1], _wLinAcc[2];
    return true;
}

bool VirtualLinkSensor::getAngularAcceleration(Eigen::Ref<Eigen::Vector3<double>> angularAcceleration) const
{
    angularAcceleration << _wAngAcc[0], _wAngAcc[1], _wAngAcc[2];
    return true;
}
