/**
 * @file IntegrationBasedIK.cpp
 * @authors Paolo Maria Viceconte
 * @copyright 2020 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the BSD-3-Clause license.
 */

#include <pybind11/eigen.h>
#include <pybind11/pybind11.h>
#include <pybind11/pytypes.h>
#include <pybind11/stl.h>

#include <BipedalLocomotion/IK/IntegrationBasedIK.h>
#include <BipedalLocomotion/System/Source.h>

#include <BipedalLocomotion/bindings/IK/IntegrationBasedIK.h>
#include <BipedalLocomotion/bindings/System/Advanceable.h>
#include <BipedalLocomotion/bindings/System/ILinearTaskSolver.h>

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

    BipedalLocomotion::bindings::System::CreateSource<IntegrationBasedIKState> //
        (module, "IntegrationBasedIK");

    BipedalLocomotion::bindings::System::CreateILinearTaskSolver<IKLinearTask,
                                                                 IntegrationBasedIKState> //
        (module, "ILinearTaskSolverIK");

    py::class_<IntegrationBasedIK, //
               ILinearTaskSolver<IKLinearTask, IntegrationBasedIKState>>(module,
                                                                         "IntegrationBasedIK");

    py::class_<IntegrationBasedIKProblem>(module, "IntegrationBasedIKProblem")
        .def_readwrite("variables_handler", &IntegrationBasedIKProblem::variablesHandler)
        .def_readwrite("weights", &IntegrationBasedIKProblem::weights)
        .def_property_readonly(
            "ik",
            [](const IntegrationBasedIKProblem& impl) { return impl.ik.get(); },
            py::return_value_policy::reference_internal)
        .def("__getitem__",
             [](IntegrationBasedIKProblem& impl, int index) -> py::object {
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
                     // in this case the onwership of the object is taken py python.
                     // Python will call the destructor and delete operator when the object’s
                     // reference count reaches zero. Undefined behavior ensues when the C++ side
                     // does the same, or when the data was not dynamically allocated. (This will
                     // not happen since the reasle is called)
                     return py::cast(impl.ik.release(), py::return_value_policy::take_ownership);
                 }

                 throw pybind11::index_error();
             })
        .def("is_valid", &IntegrationBasedIKProblem::isValid);
}

} // namespace IK
} // namespace bindings
} // namespace BipedalLocomotion
