/**
 * @file VariableFeasibleRegionTask.cpp
 * @authors Roberto Mauceri
 * @copyright 2025 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the BSD-3-Clause license.
 */

#include <pybind11/eigen.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include <BipedalLocomotion/TSID/TSIDLinearTask.h>
#include <BipedalLocomotion/TSID/VariableFeasibleRegionTask.h>
#include <BipedalLocomotion/bindings/TSID/VariableFeasibleRegionTask.h>

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

    py::class_<VariableFeasibleRegionTask,
               std::shared_ptr<VariableFeasibleRegionTask>,
               TSIDLinearTask>(module, "VariableFeasibleRegionTask")
        .def(py::init())
        .def("set_feasible_region", &VariableFeasibleRegionTask::setFeasibleRegion, py::arg("C"), py::arg("l"), py::arg("u"));
}

} // namespace TSID
} // namespace bindings
} // namespace BipedalLocomotion
