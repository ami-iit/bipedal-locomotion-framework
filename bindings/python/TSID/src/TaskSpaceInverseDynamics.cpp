/**
 * @file TaskSpaceInverseDynamics.cpp
 * @authors Giulio Romualdi
 * @copyright 2021 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the GNU Lesser General Public License v2.1 or any later version.
 */

#include <pybind11/eigen.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include <BipedalLocomotion/TSID/TSIDLinearTask.h>
#include <BipedalLocomotion/TSID/TaskSpaceInverseDynamics.h>
#include <BipedalLocomotion/bindings/TSID/TaskSpaceInverseDynamics.h>

namespace BipedalLocomotion
{
namespace bindings
{
namespace TSID
{

void CreateTaskSpaceInverseDynamics(pybind11::module& module)
{
    namespace py = ::pybind11;
    using namespace BipedalLocomotion::TSID;
    using namespace BipedalLocomotion::System;

    py::class_<TSIDState>(module, "TSIDState")
        .def(py::init())
        .def_readwrite("base_acceleration", &TSIDState::baseAcceleration)
        .def_readwrite("joint_accelerations", &TSIDState::jointAccelerations)
        .def_readwrite("joint_torques", &TSIDState::jointTorques)
        .def_readwrite("contact_wrenches", &TSIDState::contactWrenches);

    py::class_<Source<TSIDState>>(module, "TSIDStateSource");

    py::class_<ILinearTaskSolver<TSIDLinearTask, TSIDState>,
               Source<TSIDState>>(module, "ILinearTaskSolverTSID")
        .def("add_task",
             &ILinearTaskSolver<TSIDLinearTask, TSIDState>::addTask,
             py::arg("task"),
             py::arg("task_name"),
             py::arg("priority"),
             py::arg("weight") = Eigen::VectorXd())
        .def("set_task_weight",
             &ILinearTaskSolver<TSIDLinearTask, TSIDState>::setTaskWeight,
             py::arg("task_name"),
             py::arg("weight"))
        .def(
            "get_task_weight",
            [](const ILinearTaskSolver<TSIDLinearTask, TSIDState>& impl, const std::string& name) {
                auto task = impl.getTask("name").lock();
                if(task == nullptr)
                {
                    const std::string msg = "Failed to get the weight for the task named " + name + ".";
                    throw py::value_error(msg);
                }
                Eigen::VectorXd weight(task->size());

                if (!impl.getTaskWeight(name, weight))
                {
                    const std::string msg = "Failed to get the weight for the task named " + name + ".";
                    throw py::value_error(msg);
                }
                return weight;
            },
            py::arg("task_name"))
        .def("get_task_names", &ILinearTaskSolver<TSIDLinearTask, TSIDState>::getTaskNames)
        .def("finalize",
             &ILinearTaskSolver<TSIDLinearTask, TSIDState>::finalize,
             py::arg("handler"))
        .def("advance", &ILinearTaskSolver<TSIDLinearTask, TSIDState>::advance)
        .def("get_output", &ILinearTaskSolver<TSIDLinearTask, TSIDState>::getOutput)
        .def("is_output_valid", &ILinearTaskSolver<TSIDLinearTask, TSIDState>::isOutputValid)
        .def(
            "initialize",
            [](ILinearTaskSolver<TSIDLinearTask, TSIDState>& impl,
               std::shared_ptr<const BipedalLocomotion::ParametersHandler::IParametersHandler>
                   handler) -> bool { return impl.initialize(handler); },
            py::arg("handler"))
        .def("__repr__", &ILinearTaskSolver<TSIDLinearTask, TSIDState>::toString);

    py::class_<TaskSpaceInverseDynamics, //
               ILinearTaskSolver<TSIDLinearTask, TSIDState>>(module, "TaskSpaceInverseDynamics");
}

} // namespace TSID
} // namespace bindings
} // namespace BipedalLocomotion
