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
#include <BiomechanicalAnalysis/bindings/type_caster/swig.h>
#include <BipedalLocomotion/ParametersHandler/IParametersHandler.h>

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

    py::class_<nodeData>(module, "nodeData")
        .def(py::init<>())
        .def_readwrite("I_R_IMU", &nodeData::I_R_IMU)
        .def_readwrite("I_omega_IMU", &nodeData::I_omega_IMU);

    py::class_<positionData>(module, "positionData")
        .def(py::init<>())
        .def_readwrite("S_p_M", &positionData::S_p_M)
        .def_readwrite("S_v_M", &positionData::S_v_M);

    py::class_<poseData>(module, "poseData")
        .def(py::init<>())
        .def_readwrite("S_H_M", &poseData::S_H_M)
        .def_readwrite("S_v_M", &poseData::S_v_M);

    py::class_<HumanIK>(module, "HumanIK")
        .def(py::init())
        .def(
            "initialize",
            [](HumanIK& ik, std::shared_ptr<const IParametersHandler> handler, py::object& obj) -> bool {
                std::shared_ptr<iDynTree::KinDynComputations>* cls
                    = py::detail::swig_wrapped_pointer_to_pybind<std::shared_ptr<iDynTree::KinDynComputations>>(obj);

                if (cls == nullptr)
                {
                    throw ::pybind11::value_error("Invalid input for the function. Please provide "
                                                  "an iDynTree::KinDynComputations object.");
                }

                return ik.initialize(handler, *cls);
            },
            py::arg("param_handler"),
            py::arg("kin_dyn"))
        .def("setDt", &HumanIK::setDt, py::arg("dt"))
        .def("getDt", &HumanIK::getDt)
        .def("getDoFsNumber", &HumanIK::getDoFsNumber)
        .def("updateOrientationTask", &HumanIK::updateOrientationTask, py::arg("node"), py::arg("I_R_IMU"), py::arg("I_omega_IMU"))
        .def("updateGravityTask", &HumanIK::updateGravityTask, py::arg("node"), py::arg("I_R_IMU"))
        .def("updateFloorContactTask", &HumanIK::updateFloorContactTask, py::arg("node"), py::arg("verticalForce"), py::arg("linkHeight"))
        .def("clearCalibrationMatrices", &HumanIK::clearCalibrationMatrices)
        .def("calibrateWorldYaw", &HumanIK::calibrateWorldYaw, py::arg("nodeStruct"))
        .def("calibrateAllWithWorld", &HumanIK::calibrateAllWithWorld, py::arg("nodeStruct"), py::arg("frameName"))
        .def("updateOrientationGravityTasks", &HumanIK::updateOrientationAndGravityTasks, py::arg("nodeStruct"))
        .def("updateFloorContactTasks", &HumanIK::updateFloorContactTasks, py::arg("wrenchMap"), py::arg("linkHeight"))
        .def("updateJointRegularizationTask", py::overload_cast<>(&HumanIK::updateJointRegularizationTask))
        .def("updateJointRegularizationTask",
             py::overload_cast<const Eigen::VectorXd&>(&HumanIK::updateJointRegularizationTask),
             py::arg("jointPositionSetPoint"))
        .def("updateJointConstraintsTask", &HumanIK::updateJointConstraintsTask)
        .def("updatePositionTask",
             py::overload_cast<const int, const positionData&>(&HumanIK::updatePositionTask),
             py::arg("node"),
             py::arg("data"))
        .def("updatePositionTasks", &HumanIK::updatePositionTasks, py::arg("positionMap"))
        .def("updatePoseTask", py::overload_cast<const int, const poseData&>(&HumanIK::updatePoseTask), py::arg("node"), py::arg("data"))
        .def("updatePoseTasks", &HumanIK::updatePoseTasks, py::arg("poseMap"))
        .def("advance", &HumanIK::advance)
        .def("getJointPositions",
             [](HumanIK& ik) {
                 Eigen::VectorXd jointPositions(ik.getDoFsNumber());
                 bool ok = ik.getJointPositions(jointPositions);
                 return std::make_tuple(ok, jointPositions);
             })
        .def("getJointVelocities",
             [](HumanIK& ik) {
                 Eigen::VectorXd jointVelocities(ik.getDoFsNumber());
                 bool ok = ik.getJointVelocities(jointVelocities);
                 return std::make_tuple(ok, jointVelocities);
             })
        .def("getBasePosition",
             [](HumanIK& ik) {
                 Eigen::Vector3d basePosition;
                 bool ok = ik.getBasePosition(basePosition);
                 return std::make_tuple(ok, basePosition);
             })
        .def("getBaseOrientation",
             [](HumanIK& ik) {
                 Eigen::Matrix3d baseOrientation;
                 bool ok = ik.getBaseOrientation(baseOrientation);
                 return std::make_tuple(ok, baseOrientation);
             })
        .def("getBaseLinearVelocity",
             [](HumanIK& ik) {
                 Eigen::Vector3d baseVelocity;
                 bool ok = ik.getBaseLinearVelocity(baseVelocity);
                 return std::make_tuple(ok, baseVelocity);
             })
        .def("getBaseAngularVelocity",
             [](HumanIK& ik) {
                 Eigen::Vector3d baseAngularVelocity;
                 bool ok = ik.getBaseAngularVelocity(baseAngularVelocity);
                 return std::make_tuple(ok, baseAngularVelocity);
             })
        .def(
            "getOrientationTaskSetPoint",
            [](HumanIK& ik, int node) {
                manif::SO3d W_R_link;
                Eigen::Vector3d W_omega_link;
                bool ok = ik.getOrientationTaskSetPoint(node, W_R_link, W_omega_link);
                return std::make_tuple(ok, W_R_link, W_omega_link);
            },
            py::arg("node"))
        .def(
            "getGravityTaskSetPoint",
            [](HumanIK& ik, int node) {
                Eigen::Vector3d gravityDirection;
                bool ok = ik.getGravityTaskSetPoint(node, gravityDirection);
                return std::make_tuple(ok, gravityDirection);
            },
            py::arg("node"))
        .def(
            "getPositionTaskSetPoint",
            [](HumanIK& ik, int node) {
                Eigen::Vector3d W_p_frame;
                Eigen::Vector3d W_v_frame;
                bool ok = ik.getPositionTaskSetPoint(node, W_p_frame, W_v_frame);
                return std::make_tuple(ok, W_p_frame, W_v_frame);
            },
            py::arg("node"))
        .def(
            "getPoseTaskSetPoint",
            [](HumanIK& ik, int node) {
                manif::SE3d W_H_frame;
                manif::SE3d::Tangent W_v_frame;
                bool ok = ik.getPoseTaskSetPoint(node, W_H_frame, W_v_frame);
                return std::make_tuple(ok, W_H_frame, W_v_frame);
            },
            py::arg("node"));
}

} // namespace IK
} // namespace bindings
} // namespace BiomechanicalAnalysis
