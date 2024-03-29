/**
 * @file Module.cpp
 * @authors Giulio Romualdi
 * @copyright 2021 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the BSD-3-Clause license.
 */

#include <pybind11/pybind11.h>

#include <BipedalLocomotion/bindings/FloatingBaseEstimators/FloatingBaseEstimators.h>
#include <BipedalLocomotion/bindings/FloatingBaseEstimators/LeggedOdometry.h>

#include <BipedalLocomotion/bindings/FloatingBaseEstimators/Module.h>

namespace BipedalLocomotion
{
namespace bindings
{
namespace FloatingBaseEstimators
{
void CreateModule(pybind11::module& module)
{
    module.doc() = "Floating base estimators module.";

    CreateFloatingBaseEstimator(module);
    CreateLeggedOdometry(module);
}
} // namespace FloatingBaseEstimators
} // namespace bindings
} // namespace BipedalLocomotion
