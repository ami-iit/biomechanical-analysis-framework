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

#include <BiomechanicalAnalysis/ID/InverseDynamics.h>
#include <BiomechanicalAnalysis/bindings/type_caster/swig.h>
#include <BipedalLocomotion/ParametersHandler/IParametersHandler.h>
#include <iDynTree/KinDynComputations.h>

namespace BiomechanicalAnalysis
{
namespace bindings
{
namespace ID
{

void CreateInverseDynamics(pybind11::module& module)
{
    namespace py = ::pybind11;

    using namespace BiomechanicalAnalysis::ID;
    using namespace BipedalLocomotion::ParametersHandler;

    py::class_<HumanID>(module, "HumanID")
        .def(py::init())
        .def(
            "initialize",
            [](HumanID& id, std::shared_ptr<const IParametersHandler> handler, py::object& obj) -> bool {
                std::shared_ptr<iDynTree::KinDynComputations>* cls
                    = py::detail::swig_wrapped_pointer_to_pybind<std::shared_ptr<iDynTree::KinDynComputations>>(obj);

                if (cls == nullptr)
                {
                    throw ::pybind11::value_error("Invalid input for the function. Please provide "
                                                  "an iDynTree::KinDynComputations object.");
                }

                return id.initialize(handler, *cls);
            },
            py::arg("param_handler"),
            py::arg("kin_dyn"))
        .def(
            "updateExtWrenchesMeasurements",
            [](HumanID& id, const std::unordered_map<std::string, Eigen::VectorXd>& wrenchesEigen) -> bool {
                std::unordered_map<std::string, iDynTree::Wrench> wrenches;
                for (const auto& [key, value] : wrenchesEigen)
                {
                    if (value.size() != 6)
                    {
                        throw ::pybind11::value_error("Invalid input for the function. The wrenches "
                                                      "must have 6 elements.");
                    }

                    iDynTree::Wrench w;
                    w.setLinearVec3(iDynTree::GeomVector3(value(0), value(1), value(2)));
                    w.setAngularVec3(iDynTree::GeomVector3(value(3), value(4), value(5)));

                    wrenches[key] = w;
                }
                return id.updateExtWrenchesMeasurements(wrenches);
            },
            py::arg("wrenches"))
        .def("solve", &HumanID::solve)
        .def("getJointTorques",
             [](HumanID& id) -> Eigen::VectorXd {
                 Eigen::VectorXd jointTorques(id.getJointTorques().size());
                 id.getJointTorques(jointTorques);
                 return jointTorques;
             })
        .def("getJointsList", &HumanID::getJointsList)
        .def("getEstimatedExtWrenches",
             [](HumanID& id) -> std::vector<Eigen::VectorXd> {
                 std::vector<Eigen::VectorXd> wrenches;
                 const auto& estimatedWrenches = id.getEstimatedExtWrenches();
                 for (int i = 0; i < estimatedWrenches.size(); i++)
                 {
                     Eigen::VectorXd w(6);
                     w << estimatedWrenches[i].getLinearVec3()(0), estimatedWrenches[i].getLinearVec3()(1),
                         estimatedWrenches[i].getLinearVec3()(2), estimatedWrenches[i].getAngularVec3()(0),
                         estimatedWrenches[i].getAngularVec3()(1), estimatedWrenches[i].getAngularVec3()(2);
                     wrenches.push_back(w);
                 }
                 return wrenches;
             })
        .def("getEstimatedExtWrenchesList", &HumanID::getEstimatedExtWrenchesList);
}

} // namespace ID
} // namespace bindings
} // namespace BiomechanicalAnalysis
