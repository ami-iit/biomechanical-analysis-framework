#ifndef BIOMECHANICAL_ANALYSIS_DYNAMICAL_IK_GAIN_TARGET_H
#define BIOMECHANICAL_ANALYSIS_DYNAMICAL_IK_GAIN_TARGET_H

#include <iDynTree/KinDynComputations.h>

#include <BiomechanicalAnalysis/DataSources/AngularVelocity.h>
#include <BiomechanicalAnalysis/DataSources/LinearVelocity.h>
#include <BiomechanicalAnalysis/DataSources/Orientation.h>
#include <BiomechanicalAnalysis/DataSources/Position.h>

#include <BiomechanicalAnalysis/DynamicalIK/Target.h>

namespace BiomechanicalAnalysis
{
namespace DynamicalIK
{

class StdPositionTarget : public PositionTarget
{
public:
    StdPositionTarget(const std::string& name);

    void setPositionSource(std::shared_ptr<DataSources::Position<double>> positionSource);
    void setLinearVelocitySource(std::shared_ptr<DataSources::LinearVelocity<double>> linearVelocitySource);
    bool setControlledAxis(const std::vector<bool>& controlledAxis);
    void setFrame(const std::string& frame);
    void setFeedbackGain(double feedbackGain);

    const Eigen::Vector3d& getPosition() const override;
    const Eigen::Vector3d& getLinearVelocity() const override;
    
    double getFeedbackGain() const override;

    const std::string& getFrame() const override;
    const std::string& getName() const override;  

    const std::vector<bool>& getControlledAxis() const override;


    bool update();

private:
    std::string _name;
    std::string _frame;

    std::shared_ptr<DataSources::LinearVelocity<double>> _linearVelocitySource;
    std::shared_ptr<DataSources::Position<double>> _positionSource;

    Eigen::Vector3d _position;
    Eigen::Vector3d _linearVelocity;

    double _feedbackGain = 1.0;
    std::vector<bool> _controlledAxis;

};

class StdOrientationTarget : public OrientationTarget
{
public:

    StdOrientationTarget(const std::string& name);

    void setOrientationSource(std::shared_ptr<DataSources::Orientation<double>> orientationSource);
    void setAngularVelocitySource(std::shared_ptr<DataSources::AngularVelocity<double>> angularVelocitySource);
    bool setControlledAxis(const std::vector<bool>& controlledAxis);
    void setFrame(const std::string& frame);
    void setFeedbackGain(double feedbackGain);

    const Eigen::Quaterniond& getOrientation() const override;
    const Eigen::Vector3d& getAngularVelocity() const override;
    double getFeedbackGain() const override;

    const std::string& getFrame() const override;
    const std::string& getName() const override; 

    const std::vector<bool>& getControlledAxis() const override;


    bool update();

private:
    std::string _name;
    std::string _frame;

    std::shared_ptr<DataSources::Orientation<double>> _orientationSource;
    std::shared_ptr<DataSources::AngularVelocity<double>> _angularVelocitySource;

    Eigen::Quaterniond _orientation;
    Eigen::Vector3d _angularVelocity;

    double _feedbackGain = 1.0;
    std::vector<bool> _controlledAxis;
};


} // namespace DynamicalIK
} // namespace BiomechanicalAnalysis

#endif // BIOMECHANICAL_ANALYSIS_DYNAMICAL_IK_GAIN_TARGET_H
