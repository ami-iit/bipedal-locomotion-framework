/**
 * @file Contact.cpp
 * @authors Giulio Romualdi
 * @copyright 2021 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the GNU Lesser General Public License v2.1 or any later version.
 */

#include <BipedalLocomotion/Contacts/Contact.h>

using namespace BipedalLocomotion::Contacts;

bool PlannedContact::operator==(const PlannedContact& other) const
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

std::pair<bool, double> EstimatedContact::getContactDetails() const
{
    return std::make_pair(isActive, switchTime);
}

void EstimatedContact::setContactStateStamped(const std::pair<bool, double>& pair)
{
    isActive = pair.first;
    switchTime = pair.second;
}
