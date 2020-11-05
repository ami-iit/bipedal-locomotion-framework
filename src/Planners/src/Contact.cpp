/**
 * @file Contact.cpp
 * @authors Diego Ferigo
 * @copyright 2020 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the GNU Lesser General Public License v2.1 or any later version.
 */

#include "BipedalLocomotion/Planners/Contact.h"

using namespace BipedalLocomotion::Planners;

bool Contact::operator==(const Contact& other) const
{
    bool eq = true;
    eq = eq && this->activationTime == other.activationTime;
    eq = eq && this->name == other.name;
    eq = eq && this->type == other.type;
    eq = eq && this->pose.coeffs() == other.pose.coeffs();
    eq = eq && this->activationTime == other.activationTime;
    eq = eq && this->deactivationTime == other.deactivationTime;
    return eq;
}
