/**
 * @file Module.cpp
 * @authors Giulio Romualdi
 * @copyright 2021 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the BSD-3-Clause license.
 */

#include <pybind11/pybind11.h>

#include <BipedalLocomotion/bindings/TSID/AngularMomentumTask.h>
#include <BipedalLocomotion/bindings/TSID/BaseDynamicsTask.h>
#include <BipedalLocomotion/bindings/TSID/CoMTask.h>
#include <BipedalLocomotion/bindings/TSID/FeasibleContactWrenchTask.h>
#include <BipedalLocomotion/bindings/TSID/JointDynamicsTask.h>
#include <BipedalLocomotion/bindings/TSID/JointTrackingTask.h>
#include <BipedalLocomotion/bindings/TSID/Module.h>
#include <BipedalLocomotion/bindings/TSID/QPTSID.h>
#include <BipedalLocomotion/bindings/TSID/R3Task.h>
#include <BipedalLocomotion/bindings/TSID/SE3Task.h>
#include <BipedalLocomotion/bindings/TSID/SO3Task.h>
#include <BipedalLocomotion/bindings/TSID/TSIDLinearTask.h>
#include <BipedalLocomotion/bindings/TSID/TaskSpaceInverseDynamics.h>
#include <BipedalLocomotion/bindings/TSID/VariableRegularizationTask.h>

namespace BipedalLocomotion
{
namespace bindings
{
namespace TSID
{
void CreateModule(pybind11::module& module)
{
    module.doc() = "TSID module.";

    CreateTSIDLinearTask(module);
    CreateCoMTask(module);
    CreateSE3Task(module);
    CreateSO3Task(module);
    CreateR3Task(module);
    CreateJointTrackingTask(module);
    CreateBaseDynamicsTask(module);
    CreateJointDynamicsTask(module);
    CreateFeasibleContactWrenchTask(module);
    CreateTaskSpaceInverseDynamics(module);
    CreateVariableRegularizationTask(module);
    CreateAngularMomentumTask(module);
    CreateQPTSID(module);
    CreateQPFixedBaseTSID(module);
}
} // namespace TSID
} // namespace bindings
} // namespace BipedalLocomotion
