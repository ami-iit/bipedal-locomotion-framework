/**
 * @file TomlModule.cpp
 * @authors Giulio Romualdi
 * @copyright 2021 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the BSD-3-Clause license.
 */

#include <pybind11/pybind11.h>

#include <BipedalLocomotion/bindings/ParametersHandler/TomlModule.h>
#include <BipedalLocomotion/bindings/ParametersHandler/TomlParametersHandler.h>

namespace BipedalLocomotion
{
namespace bindings
{
namespace ParametersHandler
{
void CreateTomlModule(pybind11::module& module)
{
    CreateTomlParameterHandler(module);
}
} // namespace ParametersHandler
} // namespace bindings
} // namespace BipedalLocomotion
