/**
 * @file Module.cpp
 * @authors Giulio Romualdi
 * @copyright 2023 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the BSD-3-Clause license.
 */

#include <pybind11/pybind11.h>

#include <BipedalLocomotion/bindings/SimplifiedModelControllers/CoMZMPController.h>

namespace BipedalLocomotion
{
namespace bindings
{
namespace SimplifiedModelControllers
{
void CreateModule(pybind11::module& module)
{
    module.doc() = "Simplified Model Controller module.";

    CreateCoMZMPController(module);
}
} // namespace Contacts
} // namespace bindings
} // namespace SimplifiedModelControllers
