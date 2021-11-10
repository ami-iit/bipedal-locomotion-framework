/**
 * @file ITaskControllerManager.cpp
 * @authors Giulio Romualdi
 * @copyright 2021 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the GNU Lesser General Public License v2.1 or any later version.
 */

#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include <BipedalLocomotion/System/ITaskControllerManager.h>
#include <BipedalLocomotion/bindings/System/ITaskControllerManager.h>

namespace BipedalLocomotion
{
namespace bindings
{
namespace System
{

void CreateITaskControllerManager(pybind11::module& module)
{
    namespace py = ::pybind11;

    using namespace BipedalLocomotion::System;

    py::class_<ITaskControllerManager, std::shared_ptr<ITaskControllerManager>>
        ITaskControllerManager(module, "ITaskControllerManager");
    py::enum_<ITaskControllerManager::Mode>(ITaskControllerManager, "ITaskControllerManagerMode")
        .value("Enable", ITaskControllerManager::Mode::Enable)
        .value("Disable", ITaskControllerManager::Mode::Disable)
        .export_values();

    ITaskControllerManager
        .def("set_task_controller_mode",
             &ITaskControllerManager::setTaskControllerMode,
             py::arg("mode"))
        .def("get_task_controller_mode", &ITaskControllerManager::getTaskControllerMode)
        .def_property("task_controller_mode",
                      &ITaskControllerManager::getTaskControllerMode,
                      &ITaskControllerManager::setTaskControllerMode);
}

} // namespace System
} // namespace bindings
} // namespace BipedalLocomotion
