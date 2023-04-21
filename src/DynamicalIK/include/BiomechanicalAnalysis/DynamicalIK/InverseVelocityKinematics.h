#ifndef BIOMECHANICAL_ANALYSIS_FRAMEWORK_DYNAMICAL_IK_INVERSE_VELOCITY_KIN_H
#define BIOMECHANICAL_ANALYSIS_FRAMEWORK_DYNAMICAL_IK_INVERSE_VELOCITY_KIN_H

#include <memory>

#include <BiomechanicalAnalysis/IK/IntegrationBasedSolver.h>
#include "Target.h"
#include "Constraint.h"

namespace BiomechanicalAnalysis
{
namespace DynamicalIK
{

/**
 * @brief Class for inverse velocity kinematics
 * 
 */
class InverseVelocityKinematics : public BiomechanicalAnalysis::IK::IntegrationBasedSolver<double>
{
public:

    InverseVelocityKinematics();
    ~InverseVelocityKinematics();

    bool addTarget(std::shared_ptr<PositionTarget> target);
    bool addTarget(std::shared_ptr<OrientationTarget> target);
    bool addConstraint(std::shared_ptr<Constraint>& constraint);

    // IntegrationBasedSolver methods
    bool setModel(std::shared_ptr<iDynTree::Model>& model) final override;
    bool setBaseFrame(const std::string& baseFrame) final override;
    void setStep(const double dt) final override;
    double getStep() final override;
    bool initialize() final override;
    bool update() final override;

    std::vector<std::string> getJointNames() const final override;
    std::string getBaseFrame() const final override;
    size_t getNumberOfDofs() const final override;

    const Eigen::VectorX<double>& getJointPositions() const final override;
    const Eigen::VectorX<double>& getJointVelocities() const final override;
    const Eigen::Matrix<double,4,4>& getBasePose() const final override;
    const Eigen::Vector3<double>& getBaseVelocity() const final override;
    const Eigen::Vector3<double>& getCoMPosition() const final override;
    const Eigen::Vector3<double>& getCoMVelocity() const final override;

    bool isValid() const final override;
private:
    class Impl;
    std::unique_ptr<Impl> impl;
};

} // namespace DynamicalIK
} // namespace BiomechanicalAnalysis

#endif // BIOMECHANICAL_ANALYSIS_FRAMEWORK_DYNAMICAL_IK_INVERSE_VELOCITY_KIN_H
