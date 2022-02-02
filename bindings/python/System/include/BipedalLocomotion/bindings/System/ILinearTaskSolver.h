/**
 * @file ILinearTaskSolver.h
 * @authors Giulio Romualdi
 * @copyright 2021 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the GNU Lesser General Public License v2.1 or any later version.
 */

#ifndef BIPEDAL_LOCOMOTION_BINDINGS_ILINEAR_TASK_SOLVER_H
#define BIPEDAL_LOCOMOTION_BINDINGS_ILINEAR_TASK_SOLVER_H

#include <string>

#include <pybind11/eigen.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include <BipedalLocomotion/System/ILinearTaskSolver.h>

namespace BipedalLocomotion
{
namespace bindings
{
namespace System
{

template <class _Task, class _State>
void CreateILinearTaskSolver(pybind11::module& module, const std::string& pythonClassName)
{
    namespace py = ::pybind11;

    py::class_<::BipedalLocomotion::System::ILinearTaskSolver<_Task, _State>,
               ::BipedalLocomotion::System::Source<_State>>(module, pythonClassName.c_str())
        .def("add_task",
             &::BipedalLocomotion::System::ILinearTaskSolver<_Task, _State>::addTask,
             py::arg("task"),
             py::arg("task_name"),
             py::arg("priority"),
             py::arg("weight") = Eigen::VectorXd())
        .def("set_task_weight_provider",
             &::BipedalLocomotion::System::ILinearTaskSolver<_Task, _State>::setTaskWeightProvider,
             py::arg("task_name"),
             py::arg("weight_provider"))
        .def("get_task_weight_provider",
            [](const ::BipedalLocomotion::System::ILinearTaskSolver<_Task, _State>& impl,
               const std::string& taskName) {

                auto provider = impl.getTaskWeightProvider(taskName).lock();

                if (provider == nullptr)
                {
                    const std::string msg = "Failed to get the weight Provider for the task named "
                        + taskName + ".";
                    throw py::value_error(msg);
                }

                return provider;
            },
            py::arg("task_name"))
        .def("get_task_names",
             &::BipedalLocomotion::System::ILinearTaskSolver<_Task, _State>::getTaskNames)
        .def("finalize",
             &::BipedalLocomotion::System::ILinearTaskSolver<_Task, _State>::finalize,
             py::arg("handler"))
        .def("advance", &::BipedalLocomotion::System::ILinearTaskSolver<_Task, _State>::advance)
        .def("get_output",
             &::BipedalLocomotion::System::ILinearTaskSolver<_Task, _State>::getOutput)
        .def("is_output_valid",
             &::BipedalLocomotion::System::ILinearTaskSolver<_Task, _State>::isOutputValid)
        .def(
            "initialize",
            [](::BipedalLocomotion::System::ILinearTaskSolver<_Task, _State>& impl,
               std::shared_ptr<const ::BipedalLocomotion::ParametersHandler::IParametersHandler>
                   handler) -> bool { return impl.initialize(handler); },
            py::arg("handler"))
        .def("__repr__", &::BipedalLocomotion::System::ILinearTaskSolver<_Task, _State>::toString)
        .def("get_raw_solution",
             &::BipedalLocomotion::System::ILinearTaskSolver<_Task, _State>::getRawSolution);
}

} // namespace System
} // namespace bindings
} // namespace BipedalLocomotion

#endif // BIPEDAL_LOCOMOTION_BINDINGS_SYSTEM_ILINEAR_TASK_SOLVER_H
