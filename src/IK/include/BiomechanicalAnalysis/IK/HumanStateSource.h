#ifndef BIOMECHANICAL_ANALYSIS_IK_HUMAN_STATE_H
#define BIOMECHANICAL_ANALYSIS_IK_HUMAN_STATE_H

#include <vector>

#include <Eigen/Dense>

namespace BiomechanicalAnalysis
{
namespace IK
{

template <typename Scalar>
class HumanStateSource
{
public:
    virtual std::vector<std::string> getJointNames() const = 0;
    virtual std::string getBaseFrame() const = 0;
    virtual size_t getNumberOfDofs() const = 0;

    virtual const Eigen::VectorX<Scalar>& getJointPositions() const = 0;
    virtual const Eigen::VectorX<Scalar>& getJointVelocities() const = 0;

    virtual const Eigen::Matrix<Scalar,4,4>& getBasePose() const = 0;
    
    virtual const Eigen::Vector3<Scalar>& getBaseVelocity() const = 0;

    virtual const Eigen::Vector3<Scalar>& getCoMPosition() const = 0;
    virtual const Eigen::Vector3<Scalar>& getCoMVelocity() const = 0;

    virtual bool isValid() const = 0;
};
    
} // namespace IK
} // namespace BiomechanicalAnalysis



#endif // BIOMECHANICAL_ANALYSIS_IK_HUMAN_STATE_H