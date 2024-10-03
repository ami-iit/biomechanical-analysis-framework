/**
 * @file InverseKinematics.cpp
 * @authors Evelyn D'Elia
 * @copyright 2024 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the BSD-3-Clause license.
 */

#include <pybind11/chrono.h>
#include <pybind11/eigen.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include <BiomechanicalAnalysis/IK/InverseKinematics.h>
#include <BipedalLocomotion/ParametersHandler/IParametersHandler.h>
#include <BiomechanicalAnalysis/bindings/type_caster/swig.h>

namespace BiomechanicalAnalysis
{
namespace bindings
{
namespace IK
{

void CreateInverseKinematics(pybind11::module& module)
{
    namespace py = ::pybind11;

    using namespace BiomechanicalAnalysis::IK;
    using namespace BipedalLocomotion::ParametersHandler;

    py::class_<HumanIK>(module, "HumanIK")
        .def(py::init())
        .def(
            "initialize",
            [](HumanIK& ik,
                std::shared_ptr<const IParametersHandler> handler, py::object& obj) -> bool {
                std::shared_ptr<iDynTree::KinDynComputations>* cls
                    = py::detail::swig_wrapped_pointer_to_pybind<
                        std::shared_ptr<iDynTree::KinDynComputations>>(obj);

                if (cls == nullptr)
                {
                    throw ::pybind11::value_error("Invalid input for the function. Please provide "
                                                  "an iDynTree::KinDynComputations object.");
                }

                return ik.initialize(handler, *cls);
            },
            py::arg("param_handler"),
            py::arg("kin_dyn"));

}

} // namespace IK
} // namespace bindings
} // namespace BiomechanicalAnalysis
