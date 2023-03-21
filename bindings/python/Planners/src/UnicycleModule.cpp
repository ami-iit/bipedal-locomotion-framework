/**
 * @file UnicycleModule.cpp
 * @authors Giulio Romualdi, Diego Ferigo
 * @copyright 2021 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the BSD-3-Clause license.
 */

#include <pybind11/pybind11.h>

#include <BipedalLocomotion/bindings/Planners/UnicycleModule.h>
#include <BipedalLocomotion/bindings/Planners/UnicyclePlanner.h>

namespace BipedalLocomotion
{
namespace bindings
{
namespace Planners
{
void CreateUnicycleModule(pybind11::module& module)
{
    CreateUnicyclePlanner(module);
}
} // namespace Planners
} // namespace bindings
} // namespace BipedalLocomotion
