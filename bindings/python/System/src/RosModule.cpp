/**
 * @file RosModule.cpp
 * @authors Giulio Romualdi
 * @copyright 2022 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the BSD-3-Clause license.
 */

#include <pybind11/pybind11.h>

#include <BipedalLocomotion/bindings/System/RosClock.h>
#include <BipedalLocomotion/bindings/System/RosModule.h>

namespace BipedalLocomotion
{
namespace bindings
{
namespace System
{
void CreateRosModule(pybind11::module& module)
{
    CreateRosClock(module);
    CreateRosClockFactory(module);
}
} // namespace System
} // namespace bindings
} // namespace BipedalLocomotion
