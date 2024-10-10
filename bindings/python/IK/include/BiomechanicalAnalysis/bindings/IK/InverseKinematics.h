/**
 * @file InverseKinematics.h
 * @authors Evelyn D'Elia
 * @copyright 2024 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the BSD-3-Clause license.
 */

#ifndef BIOMECHANICAL_ANALYSIS_BINDINGS_IK_INVERSE_KINEMATICS_H
#define BIOMECHANICAL_ANALYSIS_BINDINGS_IK_INVERSE_KINEMATICS_H

#include <pybind11/pybind11.h>

namespace BiomechanicalAnalysis
{
namespace bindings
{
namespace IK
{

void CreateInverseKinematics(pybind11::module& module);

} // namespace IK
} // namespace bindings
} // namespace BiomechanicalAnalysis

#endif // BIOMECHANICAL_ANALYSIS_BINDINGS_IK_INVERSE_KINEMATICS_H
