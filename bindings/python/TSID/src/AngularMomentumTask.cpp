/**
 * @file AngularMomentumTask.cpp
 * @authors Giulio Romualdi
 * @copyright 2021 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the BSD-3-Clause license.
 */

#include <pybind11/eigen.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include <BipedalLocomotion/TSID/TSIDLinearTask.h>
#include <BipedalLocomotion/TSID/AngularMomentumTask.h>
#include <BipedalLocomotion/bindings/TSID/AngularMomentumTask.h>

namespace BipedalLocomotion
{
namespace bindings
{
namespace TSID
{

void CreateAngularMomentumTask(pybind11::module& module)
{
    namespace py = ::pybind11;
    using namespace BipedalLocomotion::TSID;

    py::class_<AngularMomentumTask, std::shared_ptr<AngularMomentumTask>, TSIDLinearTask>(module, "AngularMomentumTask")
        .def(py::init())
        .def("set_kin_dyn", &AngularMomentumTask::setKinDyn, py::arg("kin_dyn"))
        .def("set_set_point",
             &AngularMomentumTask::setSetPoint,
             py::arg("angular_momentum"),
             py::arg("angular_momentum_derivative"));
}

} // namespace TSID
} // namespace bindings
} // namespace BipedalLocomotion
