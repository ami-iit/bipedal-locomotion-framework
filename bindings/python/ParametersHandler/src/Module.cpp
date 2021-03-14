/**
 * @file ParametersHandler.cpp
 * @authors Giulio Romualdi, Diego Ferigo
 * @copyright 2021 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the GNU Lesser General Public License v2.1 or any later version.
 */

#include <pybind11/pybind11.h>

#include <BipedalLocomotion/bindings/ParametersHandler/Module.h>
#include <BipedalLocomotion/bindings/ParametersHandler/ParametersHandler.h>

namespace BipedalLocomotion
{
namespace bindings
{
namespace ParametersHandler
{
void CreateModule(pybind11::module& module)
{
    module.doc() = "Parameter handler module";

    CreateIParameterHandler(module);
    CreateStdParameterHandler(module);
}
} // namespace ParametersHandler
} // namespace bindings
} // namespace BipedalLocomotion
