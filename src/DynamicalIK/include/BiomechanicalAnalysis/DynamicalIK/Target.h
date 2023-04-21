
#ifndef BIOMECHANICAL_ANALYSIS_IK_TARGET_H
#define BIOMECHANICAL_ANALYSIS_IK_TARGET_H

//#include <iDynTree/Core/Position.h>
//#include <iDynTree/Core/Rotation.h>
#include <Eigen/Dense>

namespace BiomechanicalAnalysis
{
namespace DynamicalIK
{

class PositionTarget
{
public:
    virtual const Eigen::Vector3d& getPosition() const = 0;
    virtual const Eigen::Vector3d& getLinearVelocity() const = 0;
    
    virtual double getFeedbackGain() const = 0;

    virtual const std::string& getFrame() const = 0;
    virtual const std::string& getName() const = 0;  

    virtual const std::vector<bool>& getControlledAxis() const = 0;
};

class OrientationTarget
{
public:
    virtual const Eigen::Quaterniond& getOrientation() const = 0;
    virtual const Eigen::Vector3d& getAngularVelocity() const = 0;

    virtual double getFeedbackGain() const = 0;

    virtual const std::string& getFrame() const = 0;
    virtual const std::string& getName() const = 0;  

    virtual const std::vector<bool>& getControlledAxis() const = 0;
};


} // namespace IK

} // namespace BiomechanicalAnalysis

#endif
