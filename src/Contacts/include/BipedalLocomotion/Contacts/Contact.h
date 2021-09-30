/**
 * @file Contact.h
 * @authors Stefano Dafarra, Prashanth Ramadoss, Giulio Romualdi
 * @copyright 2020 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the GNU Lesser General Public License v2.1 or any later version.
 */

#ifndef BIPEDAL_LOCOMOTION_CONTACTS_CONTACT_H
#define BIPEDAL_LOCOMOTION_CONTACTS_CONTACT_H

#include <BipedalLocomotion/Math/Wrench.h>

#include <manif/SE3.h>

#include <string>

namespace BipedalLocomotion
{
namespace Contacts
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

/**
 * @brief Definition of the type of contact base class
 */
struct ContactBase
{
    /**
     * Pose of the contact.
     */
    manif::SE3d pose{manif::SE3d::Identity()};

    /**
     * Name of the contact.
     */
    std::string name{"Contact"};

    /**
     * Frame index of the contact
     */
    int index{-1};

    /**
     * Type of contact.
     */
    ContactType type{ContactType::FULL};
};

/**
 * @brief Definition of a Planned Contact structure
 */
struct PlannedContact : ContactBase
{
    /**
     * Instant from which the contact can be considered active.
     */
    double activationTime{0.0};

    /**
     * Instant after which the contact is no more active.
     */
    double deactivationTime{0.0};

    /**
     * @brief The equality operator.
     *
     * @param other The other object used for the comparison.
     * @return True if the contacts are the same, false otherwise.
     */
    bool operator==(const PlannedContact& other) const;
};

/**
 * @brief Definition of an Estimated Contact structure
 */
struct EstimatedContact : ContactBase
{

    /**
     * Instant at which the contact state was toggled.
     */
    double switchTime{0.0};

    /**
     * Current state of contact
     */
    bool isActive{false};

    /**
     * Time at which the contact details were last updated
     * This field helps in forgetting contacts
     */
    double lastUpdateTime{0.0};

    std::pair<bool, double> getContactDetails() const;

    void setContactStateStamped(const std::pair<bool, double>& pair);
};

/**
 * @brief Definition of an ContactWrench structure
 */
struct ContactWrench : public ContactBase
{
    BipedalLocomotion::Math::Wrenchd wrench{BipedalLocomotion::Math::Wrenchd::Zero()};
};

using EstimatedLandmark = EstimatedContact;

/**
 * @brief Definition of a corner
 */
struct Corner
{
    Eigen::Vector3d position;
    Eigen::Vector3d force;
};

struct ContactWithCorners : ContactBase
{
    std::vector<Corner> corners;
};

}
}

#endif // BIPEDAL_LOCOMOTION_CONTACTS_CONTACT_H
