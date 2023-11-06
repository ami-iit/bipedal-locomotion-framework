/**
 * @file Contact.h
 * @authors Stefano Dafarra, Prashanth Ramadoss, Giulio Romualdi
 * @copyright 2020 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the BSD-3-Clause license.
 */

#ifndef BIPEDAL_LOCOMOTION_CONTACTS_CONTACT_H
#define BIPEDAL_LOCOMOTION_CONTACTS_CONTACT_H

#include <cmath>
#include <ratio>
#include <string>
#include <vector>
#include <chrono>

#include <Eigen/Dense>
#include <manif/SE3.h>

#include <BipedalLocomotion/Math/Wrench.h>

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
    std::chrono::nanoseconds activationTime{std::chrono::nanoseconds::zero()};

    /**
     * Instant after which the contact is no more active.
     */
    std::chrono::nanoseconds deactivationTime{std::chrono::nanoseconds::zero()};

    /**
     * @brief The equality operator.
     *
     * @param other The other object used for the comparison.
     * @return True if the contacts are the same, false otherwise.
     */
    bool operator==(const PlannedContact& other) const;

    /**
     * @brief The inequality operator.
     *
     * @param other The other object used for the comparison.
     * @return True if the contacts are the different, false otherwise.
     */
    bool operator!=(const PlannedContact& other) const;

    /**
     * @brief Check if the contact is active at a give time instant
     *
     * @param t time instant at which we check if the contact is active.
     * @return True if `activationTime <= t < deactivationTime`.
     */
    [[nodiscard]] bool isContactActive(const std::chrono::nanoseconds& t) const;
};

/**
 * @brief Definition of an Estimated Contact structure
 */
struct EstimatedContact : ContactBase
{
    /**
     * Instant at which the contact state was toggled.
     */
    std::chrono::nanoseconds switchTime{std::chrono::nanoseconds::zero()};

    /**
     * Current state of contact
     */
    bool isActive{false};

    /**
     * Time at which the contact details were last updated
     * This field helps in forgetting contacts
     */
    std::chrono::nanoseconds lastUpdateTime{std::chrono::nanoseconds::zero()};

    std::pair<bool, std::chrono::nanoseconds> getContactDetails() const;

    void setContactStateStamped(const std::pair<bool, std::chrono::nanoseconds>& pair);
};

/**
 * @brief Definition of an ContactWrench structure
 */
struct ContactWrench : public ContactBase
{
    /**< Wrench acting on the contact */
    BipedalLocomotion::Math::Wrenchd wrench{BipedalLocomotion::Math::Wrenchd::Zero()};
};

using EstimatedLandmark = EstimatedContact;

/**
 * @brief Definition of a corner
 */
struct Corner
{
    Eigen::Vector3d position; /**< Position of the corner with respect to the main frame associated
                                 to the contact. */
    Eigen::Vector3d force; /**< Pure force applied to the contact corner. */
};

/**
 * @brief DiscreteGeometryContact is a contact which is represented by a set of corners that
 * exchanges a pure force whith the enviroment.
 */
struct DiscreteGeometryContact : public ContactBase
{
    std::vector<Corner> corners; /**< List of corners associated to the Contact. */
};

} // namespace Contacts
} // namespace BipedalLocomotion

#endif // BIPEDAL_LOCOMOTION_CONTACTS_CONTACT_H
