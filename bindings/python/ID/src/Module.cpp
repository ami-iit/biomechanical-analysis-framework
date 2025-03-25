/**
 * @file Module.cpp
 * @copyright 2024 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the BSD-3-Clause license.
 */

#include <pybind11/pybind11.h>

#include <BiomechanicalAnalysis/bindings/ID/InverseDynamics.h>
#include <BiomechanicalAnalysis/bindings/ID/Module.h>

namespace BiomechanicalAnalysis
{
namespace bindings
{
namespace ID
{

void CreateModule(pybind11::module& module)
{
    module.doc() = "ID module.";

    CreateInverseDynamics(module);
}
} // namespace ID
} // namespace bindings
} // namespace BiomechanicalAnalysis
