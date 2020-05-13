/**
 * @file Contact.h
 * @authors Stefano Dafarra
 * @copyright 2020 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the GNU Lesser General Public License v2.1 or any later version.
 */

#ifndef BIPEDAL_LOCOMOTION_PLANNERS_CONTACT_H
#define BIPEDAL_LOCOMOTION_PLANNERS_CONTACT_H

#include <iDynTree/Core/Transform.h>
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
    iDynTree::Transform pose;

    /**
     * Instant from which the contact can be considered active.
     */
    double activationTime;

    /**
     * Instant after which the contact is no more active.
     */
    double deactivationTime;

    /**
     * Name of the contact.
     */
    std::string name;

    /**
     * Type of contact.
     */
    ContactType type;
};

}
}

#endif // BIPEDAL_LOCOMOTION_PLANNERS_CONTACT_H
