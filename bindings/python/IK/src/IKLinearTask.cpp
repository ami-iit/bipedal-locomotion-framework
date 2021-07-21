/**
 * @file SE3Task.cpp
 * @authors Paolo Maria Viceconte
 * @copyright 2021 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the GNU Lesser General Public License v2.1 or any later version.
 */

#include <pybind11/eigen.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include <BipedalLocomotion/IK/IKLinearTask.h>
#include <BipedalLocomotion/System/LinearTask.h>
#include <BipedalLocomotion/bindings/IK/IKLinearTask.h>

namespace BipedalLocomotion
{
namespace bindings
{
namespace IK
{

void CreateIKLinearTask(pybind11::module& module)
{
    namespace py = ::pybind11;
    using namespace BipedalLocomotion::IK;
    using namespace BipedalLocomotion::System;

    py::class_<IKLinearTask, std::shared_ptr<IKLinearTask>, LinearTask>(module, "IKLinearTask");
}

} // namespace IK
} // namespace bindings
} // namespace BipedalLocomotion
