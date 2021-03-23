/**
 * @file VariablesHandler.cpp
 * @authors Giulio Romualdi
 * @copyright 2021 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the GNU Lesser General Public License v2.1 or any later version.
 */

#include <pybind11/pybind11.h>

#include <BipedalLocomotion/bindings/System/Module.h>
#include <BipedalLocomotion/bindings/System/VariablesHandler.h>

namespace BipedalLocomotion
{
namespace bindings
{
namespace System
{
void CreateModule(pybind11::module& module)
{
    module.doc() = "System module";

    CreateVariablesHandler(module);
}
} // namespace System
} // namespace bindings
} // namespace BipedalLocomotion
