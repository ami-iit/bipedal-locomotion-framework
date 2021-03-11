/**
 * @file IntegrationBasedIK.cpp
 * @authors Giulio Romualdi
 * @copyright 2021 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the GNU Lesser General Public License v2.1 or any later version.
 */

#include <BipedalLocomotion/IK/IntegrationBasedIK.h>

using namespace BipedalLocomotion::IK;

bool IntegrationBasedIK::initialize(std::weak_ptr<ParametersHandler::IParametersHandler> handler)
{
    return true;
};
