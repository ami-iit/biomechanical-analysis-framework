#include <BiomechanicalAnalysis/WearablesImplementation/DataSources/VirtualLinkSensor.h>

#include <Eigen/Geometry>

using namespace BiomechanicalAnalysis::DataSources::Wearables;


bool VirtualLinkSensor::update()
{
    // get wearables position
    wearable::Vector3 wPos;
    if(!_virtualLinkSensor->getLinkPosition(wPos))
    {
        return false;
    }

    wearable::Vector3 wLinVel;
    if(!_virtualLinkSensor->getLinkLinearVelocity(wLinVel))
    {
        return false;
    }

    wearable::Quaternion wQuat;
    if(!_virtualLinkSensor->getLinkOrientation(wQuat))
    {
        return false;
    }

    wearable::Vector3 wAngVel;
    if(!_virtualLinkSensor->getLinkAngularVelocity(wAngVel))
    {
        return false;
    }

    wearable::Vector3 wAngAcc;
    if(!_virtualLinkSensor->getLinkAngularAcceleration(wAngAcc))
    {
        return false;
    }

    wearable::Vector3 wLinAcc;
    if(!_virtualLinkSensor->getLinkLinearAcceleration(wLinAcc))
    {
        return false;
    }

    // fill objects

    _position << wPos[0], wPos[1] ,wPos[2];

    // get quaternion
    Eigen::Quaternion<double> quat(wQuat[0],wQuat[1], wQuat[2], wQuat[3]);    
    // get rotation matrix
    _orientationRef << quat.toRotationMatrix();

    _linVelocityRef << wLinVel[0], wLinVel[1], wLinVel[2];

    _angVelocityRef << wAngVel[0], wAngVel[1], wAngVel[2];

    _linAccelerationRef << wLinAcc[0], wLinAcc[1], wLinAcc[2];

    _angAccelerationRef << wAngAcc[0], wAngAcc[1], wAngAcc[2];

    return true;
}

Eigen::Ref<Eigen::Vector3<double>> VirtualLinkSensor::getPosition() const
{
    return _positionRef;
}

Eigen::Ref<Eigen::Matrix3<double>> VirtualLinkSensor::getOrientation() const
{
    return _orientationRef;
}


Eigen::Ref<Eigen::Vector3<double>> VirtualLinkSensor::getLinearVelocity() const
{
    return _linVelocityRef;
}

Eigen::Ref<Eigen::Vector3<double>> VirtualLinkSensor::getAngularVelocity() const
{
    return _angVelocityRef;
}

Eigen::Ref<Eigen::Vector3<double>> VirtualLinkSensor::getLinearAcceleration() const
{

    return _linAccelerationRef;;
}

Eigen::Ref<Eigen::Vector3<double>> VirtualLinkSensor::getAngularAcceleration() const
{
    
    return _angAccelerationRef;
}
