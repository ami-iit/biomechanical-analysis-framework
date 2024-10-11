/**
 * @file InverseDynamics.h
 * @authors Davide Gorbani
 * @copyright 2024 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the BSD-3-Clause license.
 */

#ifndef BIOMECHANICAL_ANALYSIS_BINDINGS_ID_INVERSE_DYNAMICS_H
#define BIOMECHANICAL_ANALYSIS_BINDINGS_ID_INVERSE_DYNAMICS_H

#include <pybind11/pybind11.h>

namespace BiomechanicalAnalysis
{
namespace bindings
{
namespace ID
{

void CreateInverseDynamics(pybind11::module& module);

} // namespace ID
} // namespace bindings
} // namespace BiomechanicalAnalysis

#endif // BIOMECHANICAL_ANALYSIS_BINDINGS_ID_INVERSE_DYNAMICS_H
