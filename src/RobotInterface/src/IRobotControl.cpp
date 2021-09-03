/**
 * @file IRobotControl.cpp
 * @authors Giulio Romualdi
 * @copyright 2020 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the GNU Lesser General Public License v2.1 or any later version.
 */

#include <BipedalLocomotion/RobotInterface/IRobotControl.h>

using namespace BipedalLocomotion::RobotInterface;

bool IRobotControl::initialize(std::weak_ptr<const ParametersHandler::IParametersHandler> handler)
{
    return true;
}
