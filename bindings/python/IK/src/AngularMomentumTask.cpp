/**
 * @file AngularMomentumTask.cpp
 * @authors Giulio Romualdi
 * @copyright 2021 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the BSD-3-Clause license.
 */

#include <pybind11/eigen.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include <BipedalLocomotion/IK/IKLinearTask.h>
#include <BipedalLocomotion/IK/AngularMomentumTask.h>

#include <BipedalLocomotion/bindings/IK/AngularMomentumTask.h>
#include <BipedalLocomotion/bindings/System/LinearTask.h>

namespace BipedalLocomotion
{
namespace bindings
{
namespace IK
{

void CreateAngularMomentumTask(pybind11::module& module)
{
    namespace py = ::pybind11;
    using namespace BipedalLocomotion::IK;

    py::class_<AngularMomentumTask, //
               std::shared_ptr<AngularMomentumTask>,
               IKLinearTask>(module, "AngularMomentumTask")
        .def(py::init())
        .def("set_kin_dyn",
             BipedalLocomotion::bindings::System::setKinDyn<AngularMomentumTask>,
             py::arg("kin_dyn"))
        .def("set_set_point", &AngularMomentumTask::setSetPoint, py::arg("angular_momentum"));
}

} // namespace IK
} // namespace bindings
} // namespace BipedalLocomotion
