/**
 * @file QPInverseKinematics.cpp
 * @authors Paolo Maria Viceconte
 * @copyright 2021 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the GNU Lesser General Public License v2.1 or any later version.
 */

#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include <BipedalLocomotion/IK/IntegrationBasedIK.h>
#include <BipedalLocomotion/IK/QPInverseKinematics.h>
#include <BipedalLocomotion/System/Source.h>
#include <BipedalLocomotion/bindings/IK/QPInverseKinematics.h>

namespace BipedalLocomotion
{
namespace bindings
{
namespace IK
{

void CreateQPInverseKinematics(pybind11::module& module)
{
    namespace py = ::pybind11;
    using namespace BipedalLocomotion::IK;

    py::class_<QPInverseKinematics, IntegrationBasedIK>(module, "QPInverseKinematics")
        .def(py::init());
}

} // namespace IK
} // namespace bindings
} // namespace BipedalLocomotion
