/**
 * @file CommonConversions.h
 * @authors Davide Gorbani <davide.gorbani@iit.it>
 */

#ifndef BIOMECHANICAL_ANALYSIS_COMMON_CONVERSIONS_H
#define BIOMECHANICAL_ANALYSIS_COMMON_CONVERSIONS_H

#include <Eigen/Dense>
#include <iDynTree/Rotation.h>

namespace BiomechanicalAnalysis
{
namespace Conversions
{

/**
 * @brief This function converts the quaternion from iDynTree to Eigen::Quaterniond.
 * @param rot The object of iDynTree::Rotation.
 * @return The rotation in Eigen::Quaterniond format.
 */
inline Eigen::Quaterniond fromiDynTreeToEigenQuatConversion(const iDynTree::Rotation& rot)
{
    Eigen::Quaterniond quat;
    quat.x() = rot.asQuaternion()[1];
    quat.y() = rot.asQuaternion()[2];
    quat.z() = rot.asQuaternion()[3];
    quat.w() = rot.asQuaternion()[0];
    return quat;
}

} // namespace Conversions
} // namespace BiomechanicalAnalysis

#endif // BIOMECHANICAL_ANALYSIS_COMMON_CONVERSIONS_H
