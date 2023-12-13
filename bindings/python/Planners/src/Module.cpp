/**
 * @file Module.cpp
 * @authors Giulio Romualdi, Diego Ferigo
 * @copyright 2021 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the BSD-3-Clause license.
 */

#include <pybind11/pybind11.h>

#include <BipedalLocomotion/bindings/Planners/DCMPlanner.h>
#include <BipedalLocomotion/bindings/Planners/Module.h>
#include <BipedalLocomotion/bindings/Planners/Spline.h>
#include <BipedalLocomotion/bindings/Planners/SwingFootPlanner.h>
#include <BipedalLocomotion/bindings/Planners/TimeVaryingDCMPlanner.h>

namespace BipedalLocomotion
{
namespace bindings
{
namespace Planners
{
void CreateModule(pybind11::module& module)
{
    module.doc() = "Planners module.";

    CreateDCMPlanner(module);
    CreateTimeVaryingDCMPlanner(module);
    CreateSwingFootPlanner(module);
    CreateSpline(module);
    CreateCubicSpline(module);
    CreateQuinticSpline(module);
}
} // namespace Planners
} // namespace bindings
} // namespace BipedalLocomotion
