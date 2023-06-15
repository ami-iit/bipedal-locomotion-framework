/**
 * @file YarpModule.cpp
 * @authors Giulio Romualdi
 * @copyright 2022 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the BSD-3-Clause license.
 */

#include <pybind11/pybind11.h>

#include <BipedalLocomotion/bindings/System/YarpClock.h>
#include <BipedalLocomotion/bindings/System/YarpModule.h>

namespace BipedalLocomotion
{
namespace bindings
{
namespace System
{
void CreateYarpModule(pybind11::module& module)
{
    CreateYarpClock(module);
    CreateYarpClockFactory(module);
}
} // namespace System
} // namespace bindings
} // namespace BipedalLocomotion
