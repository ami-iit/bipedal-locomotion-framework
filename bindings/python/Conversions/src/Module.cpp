/**
 * @file Module.cpp
 * @authors Giulio Romualdi
 * @copyright 2021 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the BSD-3-Clause license.
 */

#include <pybind11/pybind11.h>

#include <BipedalLocomotion/bindings/Conversions/ManifConversions.h>

namespace BipedalLocomotion
{
namespace bindings
{
namespace Conversions
{
void CreateModule(pybind11::module& module)
{
    module.doc() = "Conversions module.";

    CreateManifConversions(module);
}
} // namespace Conversions
} // namespace bindings
} // namespace BipedalLocomotion
