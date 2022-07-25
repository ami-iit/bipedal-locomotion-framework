/**
 * @file Constants.cpp
 * @authors Giulio Romualdi
 * @copyright 2021 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the BSD-3-Clause license.
 */

#include <BipedalLocomotion/Math/Constants.h>
#include <BipedalLocomotion/bindings/Math/Constants.h>

namespace BipedalLocomotion
{
namespace bindings
{
namespace Math
{

void CreateConstants(pybind11::module& module)
{
    using namespace BipedalLocomotion::Math;

    module.attr("StandardAccelerationOfGravitation")
        = pybind11::float_(StandardAccelerationOfGravitation);
}
} // namespace Math
} // namespace bindings
} // namespace BipedalLocomotion
