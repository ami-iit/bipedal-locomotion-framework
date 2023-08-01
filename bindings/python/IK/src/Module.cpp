/**
 * @file Module.cpp
 * @authors Paolo Maria Viceconte
 * @copyright 2021 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the BSD-3-Clause license.
 */

#include <pybind11/pybind11.h>

#include <BipedalLocomotion/bindings/IK/AngularMomentumTask.h>
#include <BipedalLocomotion/bindings/IK/CoMTask.h>
#include <BipedalLocomotion/bindings/IK/DistanceTask.h>
#include <BipedalLocomotion/bindings/IK/GravityTask.h>
#include <BipedalLocomotion/bindings/IK/IKLinearTask.h>
#include <BipedalLocomotion/bindings/IK/IntegrationBasedIK.h>
#include <BipedalLocomotion/bindings/IK/JointLimitsTask.h>
#include <BipedalLocomotion/bindings/IK/JointTrackingTask.h>
#include <BipedalLocomotion/bindings/IK/Module.h>
#include <BipedalLocomotion/bindings/IK/QPInverseKinematics.h>
#include <BipedalLocomotion/bindings/IK/R3Task.h>
#include <BipedalLocomotion/bindings/IK/SE3Task.h>
#include <BipedalLocomotion/bindings/IK/SO3Task.h>

namespace BipedalLocomotion
{
namespace bindings
{
namespace IK
{
void CreateModule(pybind11::module& module)
{
    module.doc() = "IK module.";

    CreateIKLinearTask(module);
    CreateCoMTask(module);
    CreateSE3Task(module);
    CreateSO3Task(module);
    CreateR3Task(module);
    CreateJointTrackingTask(module);
    CreateJointLimitsTask(module);
    CreateAngularMomentumTask(module);
    CreateDistanceTask(module);
    CreateGravityTask(module);
    CreateIntegrationBasedIK(module);
    CreateQPInverseKinematics(module);
}
} // namespace IK
} // namespace bindings
} // namespace BipedalLocomotion
