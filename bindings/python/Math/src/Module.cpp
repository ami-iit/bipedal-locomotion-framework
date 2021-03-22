/**
 * @file Module.cpp
 * @authors Giulio Romualdi
 * @copyright 2021 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the GNU Lesser General Public License v2.1 or any later version.
 */

#include <pybind11/pybind11.h>

#include <BipedalLocomotion/bindings/Math/Module.h>

#include <BipedalLocomotion/bindings/Math/Constants.h>

namespace BipedalLocomotion
{
namespace bindings
{
namespace Math
{
void CreateModule(pybind11::module& module)
{
    module.doc() = "Math module contains the bindings for BipedalLocomotion::Math";

    CreateConstants(module);
}
} // namespace Math
} // namespace bindings
} // namespace BipedalLocomotion
