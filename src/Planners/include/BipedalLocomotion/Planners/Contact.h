/**
 * @file Contact.h
 * @authors Stefano Dafarra
 * @copyright 2020 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the GNU Lesser General Public License v2.1 or any later version.
 */

#ifndef BIPEDAL_LOCOMOTION_PLANNERS_CONTACT_H
#define BIPEDAL_LOCOMOTION_PLANNERS_CONTACT_H

#include <manif/SE3.h>

#include <string>

namespace BipedalLocomotion
{
namespace Planners
{

/**
 * @brief Definition of the type of contact
 */
enum class ContactType
{
    /**
    * The contact impose a full pose constraint.
    */
    FULL,

    /**
    * The contact impose a position constraint.
    */
    POINT
};

struct Contact
{
    /**
     * Pose of the contact.
     */
    manif::SE3d pose{manif::SE3d::Identity()};

    /**
     * Instant from which the contact can be considered active.
     */
    double activationTime {0.0};

    /**
     * Instant after which the contact is no more active.
     */
    double deactivationTime {0.0};

    /**
     * Name of the contact.
     */
    std::string name {"Contact"};

    /**
     * Type of contact.
     */
    ContactType type {ContactType::FULL};

    /**
     * @brief The equality operator.
     *
     * @param other The other object used for the comparison.
     * @return True if the contacts are the same, false otherwise.
     */
    bool operator==(const Contact& other) const
    {
        auto doubleEq = [](const double d1, const double d2) {
            return std::abs(d1 - d2) < std::numeric_limits<double>::epsilon();
        };
        bool eq = true;
        eq = eq && this->name == other.name;
        eq = eq && this->type == other.type;
        eq = eq && this->pose.coeffs() == other.pose.coeffs();
        eq = eq && doubleEq(this->activationTime, other.activationTime);
        eq = eq && doubleEq(this->deactivationTime, other.deactivationTime);
        return eq;
    }
};

}
}

#endif // BIPEDAL_LOCOMOTION_PLANNERS_CONTACT_H
