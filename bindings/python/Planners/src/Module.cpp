/**
 * @file ParametersHandler.cpp
 * @authors Giulio Romualdi, Diego Ferigo
 * @copyright 2021 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the GNU Lesser General Public License v2.1 or any later version.
 */

#include <pybind11/pybind11.h>

#include <BipedalLocomotion/bindings/Planners/DCMPlanner.h>
#include <BipedalLocomotion/bindings/Planners/Module.h>
#include <BipedalLocomotion/bindings/Planners/QuinticSpline.h>
#include <BipedalLocomotion/bindings/Planners/SwingFootPlanner.h>
#include <BipedalLocomotion/bindings/Planners/TimeVaryingDCMPlanner.h>
#include <BipedalLocomotion/bindings/Planners/UnicyclePlanner.h>

namespace BipedalLocomotion
{
namespace bindings
{
namespace Planners
{
void CreateModule(pybind11::module& module)
{
    module.doc() = "Planners module.";

    CreateQuinticSpline(module);
    CreateDCMPlanner(module);
    CreateTimeVaryingDCMPlanner(module);
    CreateSwingFootPlanner(module);
    CreateUnicyclePlanner(module);
}
} // namespace Planners
} // namespace bindings
} // namespace BipedalLocomotion
