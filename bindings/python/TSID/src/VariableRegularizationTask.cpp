/**
 * @file VariableRegularizationTask.cpp
 * @authors Giulio Romualdi
 * @copyright 2021 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the BSD-3-Clause license.
 */

#include <pybind11/eigen.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include <BipedalLocomotion/TSID/TSIDLinearTask.h>
#include <BipedalLocomotion/TSID/VariableRegularizationTask.h>
#include <BipedalLocomotion/bindings/TSID/VariableRegularizationTask.h>

namespace BipedalLocomotion
{
namespace bindings
{
namespace TSID
{

void CreateVariableRegularizationTask(pybind11::module& module)
{
    namespace py = ::pybind11;
    using namespace ::BipedalLocomotion::TSID;

    py::class_<VariableRegularizationTask,
               std::shared_ptr<VariableRegularizationTask>,
               TSIDLinearTask>(module, "VariableRegularizationTask")
        .def(py::init())
        .def("set_set_point", &VariableRegularizationTask::setSetPoint, py::arg("set_point"));
}

} // namespace TSID
} // namespace bindings
} // namespace BipedalLocomotion
