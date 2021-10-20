/**
 * @file IntegrationBasedIK.cpp
 * @authors Paolo Maria Viceconte
 * @copyright 2020 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the GNU Lesser General Public License v2.1 or any later version.
 */

#include <pybind11/eigen.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include <BipedalLocomotion/IK/IntegrationBasedIK.h>
#include <BipedalLocomotion/System/ILinearTaskSolver.h>
#include <BipedalLocomotion/System/Source.h>
#include <BipedalLocomotion/bindings/IK/IntegrationBasedIK.h>

namespace BipedalLocomotion
{
namespace bindings
{
namespace IK
{

void CreateIntegrationBasedIK(pybind11::module& module)
{
    namespace py = ::pybind11;
    using namespace BipedalLocomotion::IK;
    using namespace BipedalLocomotion::System;

    py::class_<IntegrationBasedIKState>(module, "IntegrationBasedIKState")
        .def(py::init())
        .def_readwrite("joint_velocity", &IntegrationBasedIKState::jointVelocity)
        .def_readwrite("base_velocity", &IntegrationBasedIKState::baseVelocity);

    py::class_<Source<IntegrationBasedIKState>>(module, "IntegrationBasedIKStateSource");

    py::class_<ILinearTaskSolver<IKLinearTask, IntegrationBasedIKState>,
               Source<IntegrationBasedIKState>>(module, "ILinearTaskSolverIK")
        .def("add_task",
             &ILinearTaskSolver<IKLinearTask, IntegrationBasedIKState>::addTask,
             py::arg("task"),
             py::arg("task_name"),
             py::arg("priority"),
             py::arg("weight") = Eigen::VectorXd())
        .def("get_task_names",
             &ILinearTaskSolver<IKLinearTask, IntegrationBasedIKState>::getTaskNames)
        .def("finalize",
             &ILinearTaskSolver<IKLinearTask, IntegrationBasedIKState>::finalize,
             py::arg("handler"))
        .def("advance", &ILinearTaskSolver<IKLinearTask, IntegrationBasedIKState>::advance)
        .def("get_output", &ILinearTaskSolver<IKLinearTask, IntegrationBasedIKState>::getOutput)
        .def("is_output_valid",
             &ILinearTaskSolver<IKLinearTask, IntegrationBasedIKState>::isOutputValid)
        .def(
            "initialize",
            [](ILinearTaskSolver<IKLinearTask, IntegrationBasedIKState>& impl,
               std::shared_ptr<const BipedalLocomotion::ParametersHandler::IParametersHandler>
                   handler) -> bool { return impl.initialize(handler); },
            py::arg("handler"));

    py::class_<IntegrationBasedIK, //
               ILinearTaskSolver<IKLinearTask, IntegrationBasedIKState>>(module,
                                                                         "IntegrationBasedIK");
}

} // namespace IK
} // namespace bindings
} // namespace BipedalLocomotion
