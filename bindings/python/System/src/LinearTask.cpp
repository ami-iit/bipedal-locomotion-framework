/**
 * @file LinearTask.cpp
 * @authors Paolo Maria Viceconte
 * @copyright 2021 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the GNU Lesser General Public License v2.1 or any later version.
 */

#include <pybind11/cast.h>
#include <pybind11/eigen.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include <BipedalLocomotion/System/LinearTask.h>
#include <BipedalLocomotion/bindings/System/LinearTask.h>

namespace BipedalLocomotion
{
namespace bindings
{
namespace System
{
class LinearTaskPublicist : public BipedalLocomotion::System::LinearTask
{
public:
    using BipedalLocomotion::System::LinearTask::m_A;
    using BipedalLocomotion::System::LinearTask::m_b;
    using BipedalLocomotion::System::LinearTask::m_description;
};

void CreateLinearTask(pybind11::module& module)
{
    namespace py = ::pybind11;

    using namespace BipedalLocomotion::System;

    py::class_<LinearTask, //
               LinearTaskTrampoline<>,
               std::shared_ptr<LinearTask>>
        linearTask(module, "LinearTask");

    py::enum_<LinearTask::Type>(linearTask, "Type")
        .value("equality", LinearTask::Type::equality)
        .value("inequality", LinearTask::Type::inequality)
        .export_values();

    linearTask.def(py::init<>())
        .def(
            "initialize",
            [](LinearTask& impl,
               std::shared_ptr<const BipedalLocomotion::ParametersHandler::IParametersHandler>
                   handler) -> bool { return impl.initialize(handler); },
            py::arg("param_handler"))
        .def("set_variables_handler",
             &LinearTask::setVariablesHandler,
             py::arg("variables_handler"))
        .def("update", &LinearTask::update)
        .def("get_A", &LinearTask::getA)
        .def("get_b", &LinearTask::getB)
        .def("size", &LinearTask::size)
        .def("get_description", &LinearTask::getDescription)
        .def("type", &LinearTask::type)
        .def("is_valid", &LinearTask::isValid)
        .def("__repr__", &LinearTask::getDescription)
        .def("__len__", &LinearTask::size)
        .def_readwrite("_A", &LinearTaskPublicist::m_A)
        .def_readwrite("_b", &LinearTaskPublicist::m_b)
        .def_readwrite("_description", &LinearTaskPublicist::m_description);
}

} // namespace System
} // namespace bindings
} // namespace BipedalLocomotion
