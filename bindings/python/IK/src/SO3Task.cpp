/**
 * @file SO3Task.cpp
 * @authors Paolo Maria Viceconte
 * @copyright 2021 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the BSD-3-Clause license.
 */

#include <pybind11/eigen.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include <BipedalLocomotion/IK/IKLinearTask.h>
#include <BipedalLocomotion/IK/SO3Task.h>

#include <BipedalLocomotion/bindings/IK/SO3Task.h>
#include <BipedalLocomotion/bindings/System/LinearTask.h>

#include <manif/manif.h>

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

    py::class_<SO3Task, std::shared_ptr<SO3Task>, IKLinearTask>(module, "SO3Task")
        .def(py::init())
        .def("set_set_point",
             &SO3Task::setSetPoint,
             py::arg("I_R_F"),
             py::arg("angular_velocity") = manif::SO3d::Tangent::Zero());
}

} // namespace IK
} // namespace bindings
} // namespace BipedalLocomotion
