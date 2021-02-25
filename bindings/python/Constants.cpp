/**
 * @file Constants.cpp
 * @authors Giulio Romualdi
 * @copyright 2021 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the GNU Lesser General Public License v2.1 or any later version.
 */

#include "BipedalLocomotion/Math/Constants.h"

#include "bipedal_locomotion_framework.h"

void BipedalLocomotion::bindings::CreateConstants(pybind11::module& module)
{
    using namespace BipedalLocomotion::Math;

    module.attr("StandardAccelerationOfGravitation")
        = pybind11::float_(StandardAccelerationOfGravitation);
}
