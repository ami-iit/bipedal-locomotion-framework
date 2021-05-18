/**
 * @file SO3Task.cpp
 * @authors Paolo Maria Viceconte
 * @copyright 2021 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the GNU Lesser General Public License v2.1 or any later version.
 */

#include <pybind11/eigen.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include <BipedalLocomotion/IK/SO3Task.h>
#include <BipedalLocomotion/System/LinearTask.h>
#include <BipedalLocomotion/bindings/IK/SO3Task.h>

namespace BipedalLocomotion
{
namespace bindings
{
namespace IK
{

void CreateSO3Task(pybind11::module& module)
{
    namespace py = ::pybind11;
    using namespace BipedalLocomotion::IK;
    using namespace BipedalLocomotion::System;

    py::class_<SO3Task, std::shared_ptr<SO3Task>, LinearTask>(module, "SO3Task")
        .def(py::init())
        .def(
            "initialize",
            [](SO3Task& impl,
               std::shared_ptr<BipedalLocomotion::ParametersHandler::IParametersHandler>
                   paramHandler) -> bool { return impl.initialize(paramHandler); },
            py::arg("param_handler"))
        .def("set_kin_dyn", &SO3Task::setKinDyn, py::arg("kin_dyn"))
        .def("set_variables_handler", &SO3Task::setVariablesHandler, py::arg("variables_handler"))
        .def("set_set_point", &SO3Task::setSetPoint, py::arg("I_R_F"), py::arg("angular_velocity"));
}

} // namespace IK
} // namespace bindings
} // namespace BipedalLocomotion
