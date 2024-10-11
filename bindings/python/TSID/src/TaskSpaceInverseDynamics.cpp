/**
 * @file TaskSpaceInverseDynamics.cpp
 * @authors Giulio Romualdi
 * @copyright 2021 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the BSD-3-Clause license.
 */

#include <pybind11/eigen.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include <BipedalLocomotion/TSID/TSIDLinearTask.h>
#include <BipedalLocomotion/TSID/TaskSpaceInverseDynamics.h>

#include <BipedalLocomotion/bindings/System/Advanceable.h>
#include <BipedalLocomotion/bindings/System/ILinearTaskSolver.h>
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

    py::class_<TSIDState>(module, "TSIDState")
        .def(py::init())
        .def_readwrite("base_acceleration", &TSIDState::baseAcceleration)
        .def_readwrite("joint_accelerations", &TSIDState::jointAccelerations)
        .def_readwrite("joint_torques", &TSIDState::jointTorques)
        .def_readwrite("contact_wrenches", &TSIDState::contactWrenches);

    BipedalLocomotion::bindings::System::CreateSource<TSIDState>(module, "ILinearTaskSolverTSID");

    BipedalLocomotion::bindings::System::CreateILinearTaskSolver<TSIDLinearTask,
                                                                 TSIDState> //
        (module, "ILinearTaskSolverTSID");

    py::class_<TaskSpaceInverseDynamics, //
               ::BipedalLocomotion::System::ILinearTaskSolver<TSIDLinearTask, TSIDState>> //
        (module, "TaskSpaceInverseDynamics");

    py::class_<TaskSpaceInverseDynamicsProblem>(module, "TaskSpaceInverseDynamicsProblem")
        .def_readwrite("variables_handler", &TaskSpaceInverseDynamicsProblem::variablesHandler)
        .def_readwrite("weights", &TaskSpaceInverseDynamicsProblem::weights)
        .def_property_readonly(
            "tsid",
            [](const TaskSpaceInverseDynamicsProblem& impl) { return impl.tsid.get(); },
            py::return_value_policy::reference_internal)
        .def("__getitem__",
             [](TaskSpaceInverseDynamicsProblem& impl, int index) -> py::object {
                 if (index == 0)
                 {
                     return py::cast(impl.variablesHandler);
                 }
                 if (index == 1)
                 {
                     return py::cast(impl.weights);
                 }
                 if (index == 2)
                 {
                     // in this case the ownership of the object is taken py python.
                     // Python will call the destructor and delete operator when the object’s
                     // reference count reaches zero. Undefined behavior ensues when the C++ side
                     // does the same, or when the data was not dynamically allocated. (This will
                     // not happen since the reasle is called)
                     return py::cast(impl.tsid.release(), py::return_value_policy::take_ownership);
                 }

                 throw pybind11::index_error();
             })
        .def("is_valid", &TaskSpaceInverseDynamicsProblem::isValid);
}

} // namespace TSID
} // namespace bindings
} // namespace BipedalLocomotion
