/**
 * @file Module.cpp
 * @authors Giulio Romualdi
 * @copyright 2021 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the BSD-3-Clause license.
 */

#include <pybind11/pybind11.h>

#include <BipedalLocomotion/bindings/System/Advanceable.h>
#include <BipedalLocomotion/bindings/System/Clock.h>
#include <BipedalLocomotion/bindings/System/IClock.h>
#include <BipedalLocomotion/bindings/System/ITaskControllerManager.h>
#include <BipedalLocomotion/bindings/System/LinearTask.h>
#include <BipedalLocomotion/bindings/System/Module.h>
#include <BipedalLocomotion/bindings/System/VariablesHandler.h>
#include <BipedalLocomotion/bindings/System/WeightProvider.h>

namespace BipedalLocomotion
{
namespace bindings
{
namespace System
{
void CreateModule(pybind11::module& module)
{
    module.doc() = "System module";

    CreateCommonDataStructure(module);

    CreateVariablesHandler(module);
    CreateLinearTask(module);
    CreateITaskControllerManager(module);
    CreateIClock(module);
    CreateClockFactory(module);
    CreateClockBuilder(module);

    CreateWeightProvider(module);
    CreateConstantWeightProvider(module);
}
} // namespace System
} // namespace bindings
} // namespace BipedalLocomotion
