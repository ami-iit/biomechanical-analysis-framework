#include <BiomechanicalAnalysis/DynamicalIK/StdTarget.h>

using namespace BiomechanicalAnalysis::DynamicalIK;

StdPositionTarget::StdPositionTarget(const std::string& name)
{
    _name = name;
    _frame = "";

    _position.setZero();
    _linearVelocity.setZero();
    _positionSource = nullptr;
    _linearVelocitySource = nullptr;
    _controlledAxis = {true, true, true};
}

bool StdPositionTarget::update()
{
    if(_positionSource!=nullptr && !_positionSource->getPosition(_position)){
        return false;
    }

    if(_linearVelocitySource!=nullptr && !_linearVelocitySource->getLinearVelocity(_linearVelocity)){
        return false;
    }

    return true;
}

void StdPositionTarget::setPositionSource(std::shared_ptr<DataSources::Position<double>> positionSource)
{
    _positionSource = positionSource;
}

void StdPositionTarget::setLinearVelocitySource(std::shared_ptr<DataSources::LinearVelocity<double>> linearVelocitySource)
{
    _linearVelocitySource = linearVelocitySource;
}

const Eigen::Vector3d& StdPositionTarget::getPosition() const
{
    return _position;
}

const Eigen::Vector3d& StdPositionTarget::getLinearVelocity() const
{
    return _linearVelocity;
}

const std::vector<bool>& StdPositionTarget::getControlledAxis() const
{
    return _controlledAxis;
}

bool StdPositionTarget::setControlledAxis(const std::vector<bool>& controlledAxis)
{
    if(controlledAxis.size()!=3){
        return false;
    }

    _controlledAxis = controlledAxis;
    return true;
}

void StdPositionTarget::setFeedbackGain(double feedbackGain)
{
    _feedbackGain = feedbackGain;
}

double StdPositionTarget::getFeedbackGain() const
{
    return _feedbackGain;
}

// const Eigen::Vector3d& StdTarget::getPositionWeight()
// {

// };

// const Eigen::Vector3d& StdTarget::getRotationWeight()
// {

// }
void StdPositionTarget::setFrame(const std::string& frame)
{
    _frame = frame;
}


const std::string& StdPositionTarget::getFrame() const
{
    return _frame;
}

const std::string& StdPositionTarget::getName() const
{
    return _name;
}


StdOrientationTarget::StdOrientationTarget(const std::string& name)
{
    _name = name;

    _orientation.setIdentity();
    _angularVelocity.setZero();
    _orientationSource = nullptr;
    _angularVelocitySource = nullptr;
    _controlledAxis = {true, true, true};
}

bool StdOrientationTarget::update()
{
    if(_orientationSource!=nullptr && !_orientationSource->getOrientation(_orientation)){
        return false;
    }

    if(_angularVelocitySource!=nullptr && !_angularVelocitySource->getAngularVelocity(_angularVelocity)){
        return false;
    }

    return true;
}

void StdOrientationTarget::setOrientationSource(std::shared_ptr<DataSources::Orientation<double>> orientationSource)
{
    _orientationSource = orientationSource;    
}

void StdOrientationTarget::setAngularVelocitySource(std::shared_ptr<DataSources::AngularVelocity<double>> angularVelocitySource)
{ 
    _angularVelocitySource = angularVelocitySource;
}

const Eigen::Quaterniond& StdOrientationTarget::getOrientation() const
{
    return _orientation;
}

const Eigen::Vector3d& StdOrientationTarget::getAngularVelocity() const
{
    return _angularVelocity;
}

void StdOrientationTarget::setFrame(const std::string& frame)
{
    _frame = frame;
}

const std::string& StdOrientationTarget::getFrame() const
{
    return _frame;
}

const std::string& StdOrientationTarget::getName() const
{
    return _name;
}

const std::vector<bool>& StdOrientationTarget::getControlledAxis() const
{
    return _controlledAxis;
}

bool StdOrientationTarget::setControlledAxis(const std::vector<bool>& controlledAxis)
{
    if(controlledAxis.size()!=3){
        return false;
    }

    _controlledAxis = controlledAxis;
    return true;
}

void StdOrientationTarget::setFeedbackGain(double feedbackGain)
{
    _feedbackGain = feedbackGain;
}

double StdOrientationTarget::getFeedbackGain() const
{
    return _feedbackGain;
}
