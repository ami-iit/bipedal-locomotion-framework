/**
 * @file YarpModule.cpp
 * @authors Giulio Romualdi
 * @copyright 2021 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the BSD-3-Clause license.
 */

#include <pybind11/pybind11.h>

#include <BipedalLocomotion/bindings/ParametersHandler/YarpModule.h>
#include <BipedalLocomotion/bindings/ParametersHandler/YarpParametersHandler.h>

namespace BipedalLocomotion
{
namespace bindings
{
namespace ParametersHandler
{
void CreateYarpModule(pybind11::module& module)
{
    CreateYarpParameterHandler(module);
}
} // namespace ParametersHandler
} // namespace bindings
} // namespace BipedalLocomotion
