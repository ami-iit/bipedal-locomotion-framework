/**
 * @file Module.cpp
 * @authors Giulio Romualdi, Diego Ferigo
 * @copyright 2021 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the GNU Lesser General Public License v2.1 or any later version.
 */

#include <pybind11/pybind11.h>

#include <BipedalLocomotion/bindings/manif/Module.h>

#include <BipedalLocomotion/bindings/manif/BaseTypes.h>

namespace BipedalLocomotion
{
namespace bindings
{
namespace manif
{
void CreateModule(pybind11::module& module)
{
    module.doc() = "manif module contains the bindings for manif";

    CreateBaseTypes(module);
}
} // namespace manif
} // namespace bindings
} // namespace BipedalLocomotion
