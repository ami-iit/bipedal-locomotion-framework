/**
 * @file Contact.cpp
 * @authors Giulio Romualdi
 * @copyright 2021 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the BSD-3-Clause license.
 */

#include <chrono>

#include <BipedalLocomotion/Contacts/Contact.h>

using namespace BipedalLocomotion::Contacts;

bool PlannedContact::operator==(const PlannedContact& other) const
{
    bool eq = this->activationTime == other.activationTime;
    eq = eq && this->name == other.name;
    eq = eq && this->type == other.type;
    eq = eq && this->pose.coeffs() == other.pose.coeffs();
    eq = eq && this->activationTime == other.activationTime;
    eq = eq && this->deactivationTime == other.deactivationTime;
    return eq;
}

bool PlannedContact::operator!=(const PlannedContact& other) const
{
    return !this->operator==(other);
}

bool PlannedContact::isContactActive(const std::chrono::nanoseconds& t) const
{
    return (this->activationTime <= t) && (t < this->deactivationTime);
}

std::string PlannedContact::toString() const
{
    std::stringstream ss;
    ss << "Contact name: " << name << " activation time: "
       << std::chrono::duration_cast<std::chrono::milliseconds>(activationTime).count()
       << "ms deactivation time: "
       << std::chrono::duration_cast<std::chrono::milliseconds>(deactivationTime).count()
       << "ms pose: " << pose.coeffs().transpose();
    return ss.str();
}

std::pair<bool, std::chrono::nanoseconds> EstimatedContact::getContactDetails() const
{
    return std::make_pair(isActive, switchTime);
}

void EstimatedContact::setContactStateStamped(const std::pair<bool, std::chrono::nanoseconds>& pair)
{
    isActive = pair.first;
    switchTime = pair.second;
}

ContactWrench DiscreteGeometryContact::getContactWrench() const
{

    ContactWrench contactWrench;
    contactWrench.index = this->index;
    contactWrench.name = this->name;
    contactWrench.pose = this->pose;
    contactWrench.type = this->type;
    contactWrench.wrench = BipedalLocomotion::Math::Wrenchd::Zero();
    for (const auto& corner : this->corners)
    {
        contactWrench.wrench.force() += corner.force;
        contactWrench.wrench.torque() += corner.position.cross(corner.force);
    }

    return contactWrench;
}