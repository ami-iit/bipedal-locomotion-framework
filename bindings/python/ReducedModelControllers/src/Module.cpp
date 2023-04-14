/**
 * @file Module.cpp
 * @authors Carlotta Sartore
 * @copyright 2021 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the BSD-3-Clause license.
 */

#include <pybind11/pybind11.h>

#include <BipedalLocomotion/bindings/ReducedModelControllers/CentroidalMPC.h>

namespace BipedalLocomotion
{
namespace bindings
{
namespace ReducedModelControllers
{
void CreateModule(pybind11::module& module)
{
    module.doc() = "Reduced Model Controllers module.";

    CreateCentroidalMPC(module);
}
} // namespace Contacts
} // namespace bindings
} // namespace ReducedModelControllers
