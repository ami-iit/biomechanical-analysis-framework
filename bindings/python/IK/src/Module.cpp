/**
 * @file Module.cpp
 * @copyright 2024 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the BSD-3-Clause license.
 */

#include <pybind11/pybind11.h>

#include <BiomechanicalAnalysis/bindings/IK/InverseKinematics.h>
#include <BiomechanicalAnalysis/bindings/IK/Module.h>

namespace BiomechanicalAnalysis
{
namespace bindings
{
namespace IK
{

void CreateModule(pybind11::module& module)
{
    module.doc() = "IK module.";

    CreateInverseKinematics(module);
}
} // namespace IK
} // namespace bindings
} // namespace BiomchanicalAnalysis
