/**
 * @file Module.cpp
 * @authors Giulio Romualdi
 * @copyright 2021 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the GNU Lesser General Public License v2.1 or any later version.
 */

#include <pybind11/pybind11.h>

#include <BipedalLocomotion/bindings/TSID/BaseDynamicsTask.h>
#include <BipedalLocomotion/bindings/TSID/CoMTask.h>
#include <BipedalLocomotion/bindings/TSID/FeasibleContactWrenchTask.h>
#include <BipedalLocomotion/bindings/TSID/JointDynamicsTask.h>
#include <BipedalLocomotion/bindings/TSID/JointTrackingTask.h>
#include <BipedalLocomotion/bindings/TSID/Module.h>
#include <BipedalLocomotion/bindings/TSID/QPTSID.h>
#include <BipedalLocomotion/bindings/TSID/SE3Task.h>
#include <BipedalLocomotion/bindings/TSID/SO3Task.h>
#include <BipedalLocomotion/bindings/TSID/TSIDLinearTask.h>
#include <BipedalLocomotion/bindings/TSID/TaskSpaceInverseDynamics.h>

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
    CreateJointTrackingTask(module);
    CreateBaseDynamicsTask(module);
    CreateJointDynamicsTask(module);
    CreateFeasibleContactWrenchTask(module);
    CreateTaskSpaceInverseDynamics(module);
    CreateQPTSID(module);
    CreateQPFixedBaseTSID(module);
}
} // namespace TSID
} // namespace bindings
} // namespace BipedalLocomotion
