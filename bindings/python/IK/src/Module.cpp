/**
 * @file Module.cpp
 * @authors Paolo Maria Viceconte
 * @copyright 2021 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the GNU Lesser General Public License v2.1 or any later version.
 */

#include <pybind11/pybind11.h>

#include <BipedalLocomotion/bindings/IK/Module.h>
#include <BipedalLocomotion/bindings/IK/CoMTask.h>
#include <BipedalLocomotion/bindings/IK/SE3Task.h>
#include <BipedalLocomotion/bindings/IK/SO3Task.h>
#include <BipedalLocomotion/bindings/IK/JointTrackingTask.h>

namespace BipedalLocomotion
{
namespace bindings
{
namespace IK
{
void CreateModule(pybind11::module& module)
{
    module.doc() = "IK module.";

    CreateCoMTask(module);
    CreateSE3Task(module);
    CreateSO3Task(module);
    CreateJointTrackingTask(module);
}
} // namespace IK
} // namespace bindings
} // namespace BipedalLocomotion
