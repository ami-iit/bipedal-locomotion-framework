/**
 * @file Frame.cpp
 * @authors Giulio Romualdi
 * @copyright 2019 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the GNU Lesser General Public License v2.1 or any later version.
 */

#include <BipedalLocomotionControllers/OptimalControlUtilities/Frame.h>

using namespace BipedalLocomotionControllers::OptimalControlUtilities;

std::string& FrameNames::label()
{
    return this->first;
}
const std::string& FrameNames::label() const
{
    return this->first;
}
std::string& FrameNames::nameInModel()
{
    return this->second;
}
const std::string& FrameNames::nameInModel() const
{
    return this->second;
}
