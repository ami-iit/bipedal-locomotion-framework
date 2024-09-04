/**
 * @file JointVelocityLimitsTask.cpp
 * @authors Davide Gorbani
 * @copyright 2023 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the BSD-3-Clause license.
 */

#include <pybind11/eigen.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include <BipedalLocomotion/IK/IKLinearTask.h>
#include <BipedalLocomotion/IK/JointVelocityLimitsTask.h>

#include <BipedalLocomotion/bindings/IK/JointVelocityLimitsTask.h>
#include <BipedalLocomotion/bindings/System/LinearTask.h>

namespace BipedalLocomotion
{
namespace bindings
{
namespace IK
{

void CreateJointVelocityLimitsTask(pybind11::module& module)
{
    namespace py = ::pybind11;
    using namespace BipedalLocomotion::IK;

    py::class_<JointVelocityLimitsTask,
               std::shared_ptr<JointVelocityLimitsTask>,
               IKLinearTask>(module, "JointVelocityLimitsTask")
        .def(py::init());
}

} // namespace IK
} // namespace bindings
} // namespace BipedalLocomotion
