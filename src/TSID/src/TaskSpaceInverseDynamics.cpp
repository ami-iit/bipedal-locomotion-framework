/**
 * @file TaskSpaceInverseDynamics.cpp
 * @authors Giulio Romualdi
 * @copyright 2024 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the BSD-3-Clause license.
 */

#include <BipedalLocomotion/TSID/TaskSpaceInverseDynamics.h>

using namespace BipedalLocomotion::TSID;

bool TaskSpaceInverseDynamicsProblem::isValid() const
{
    return this->tsid != nullptr;
}
