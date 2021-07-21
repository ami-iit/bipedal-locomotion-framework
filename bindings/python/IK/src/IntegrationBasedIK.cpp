/**
 * @file IntegrationBasedIK.cpp
 * @authors Paolo Maria Viceconte
 * @copyright 2020 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the GNU Lesser General Public License v2.1 or any later version.
 */

#include <pybind11/eigen.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include <BipedalLocomotion/IK/IntegrationBasedIK.h>
#include <BipedalLocomotion/System/Source.h>
#include <BipedalLocomotion/bindings/IK/IntegrationBasedIK.h>

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

    py::class_<Source<IntegrationBasedIKState>>(module, "IntegrationBasedIKStateSource");

    py::class_<IntegrationBasedIK, Source<IntegrationBasedIKState>>(module, "IntegrationBasedIK");
}

} // namespace IK
} // namespace bindings
} // namespace BipedalLocomotion
