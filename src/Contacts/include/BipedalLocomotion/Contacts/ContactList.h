/**
 * @file ContactList.h
 * @authors Stefano Dafarra
 * @copyright 2020 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the BSD-3-Clause license.
 */

#ifndef BIPEDAL_LOCOMOTION_CONTACTS_CONTACTLIST_H
#define BIPEDAL_LOCOMOTION_CONTACTS_CONTACTLIST_H

// BipedalLocomotion
#include <BipedalLocomotion/Contacts/Contact.h>

#include <manif/manif.h>

// std
#include <chrono>
#include <functional>
#include <set>
#include <string>
#include <unordered_map>

namespace BipedalLocomotion
{
namespace Contacts
{

/**
 * @brief Class containing a list of contacts.
 * The contact are added such that the activation time is strictly growing. In addition, contacts
 * cannot be overlapping. It represents a series of contact activations and deactivations of a
 * single entity.
 */
class ContactList
{
    /**
     * @brief Struct used for inserting new contacts in the set.
     */
    struct ContactCompare
    {
        bool operator()(const BipedalLocomotion::Contacts::PlannedContact& lhs,
                        const BipedalLocomotion::Contacts::PlannedContact& rhs) const;
    };

    std::set<BipedalLocomotion::Contacts::PlannedContact, ContactCompare> m_contacts; /** Data
                                                     structure for inserting and ordering
                                                     contacts. **/
    std::string m_defaultName{"ContactList"}; /** Default name for the contact list. **/
    BipedalLocomotion::Contacts::ContactType m_defaultContactType{
        BipedalLocomotion::Contacts::ContactType::FULL}; /** Default contact type. **/
    int m_defaultIndex{-1}; /**< Default Frame index of the contact */

public:
    using const_iterator
        = std::set<BipedalLocomotion::Contacts::PlannedContact, ContactCompare>::const_iterator;
    using const_reverse_iterator = std::set<BipedalLocomotion::Contacts::PlannedContact,
                                            ContactCompare>::const_reverse_iterator;

    /**
     * @brief Set the default name.
     * @param defaultName the default name.
     */
    void setDefaultName(const std::string& defaultName);

    /**
     * @brief Get the default name.
     * @return the default name.
     */
    const std::string& defaultName() const;

    /**
     * @brief Set the default contact type.
     * @param The default contact type.
     */
    void setDefaultContactType(const BipedalLocomotion::Contacts::ContactType& type);

    /**
     * @brief Set the default frame index of the contact
     * @param defaultIndex the default index.
     */
    void setDefaultIndex(int defaultIndex);

    /**
     * @brief Get the default frame index of the contact
     * @return the default index.
     */
    int defaultIndex() const;

    /**
     * @brief Get the default contact type.
     * @return the default contact type.
     */
    const BipedalLocomotion::Contacts::ContactType& defaultContactType() const;

    /**
     * @brief Add a new contact to the list.
     * @param newContact The new contact
     * @return false if it was not possible to insert the contact.
     * Possible failures: the activation time is greater than the deactivation time, or the new
     * contact overlaps with an existing contact.
     */
    bool addContact(const BipedalLocomotion::Contacts::PlannedContact& newContact);

    /**
     * @brief Add a new contact to the list.
     *
     * It uses the defaultName and the defaultContactType for the missing informations.
     * @param newTransform The contact pose.
     * @param activationTime The activation time.
     * @param deactivationTime The deactivation time.
     * @return false if it was not possible to insert the contact.
     * Possible failures: the activation time is greater than the deactivation time, or the new
     * contact overlaps with an existing contact.
     */
    bool addContact(const manif::SE3d& newTransform,
                    const std::chrono::nanoseconds& activationTime,
                    const std::chrono::nanoseconds& deactivationTime);

    /**
     * @brief Erase a contact
     * @param iterator to the contact to erase.
     * @return an iterator to the contact that follows the one removed.
     */
    const_iterator erase(const_iterator iterator);

    /**
     * @brief Return a const iterator to the begin of the contacts.
     */
    const_iterator begin() const;

    /**
     * @brief Return a const iterator to the begin of the contacts.
     */
    const_iterator cbegin() const;

    /**
     * @brief Return a const reverse iterator to the contacts (basically starting from the last
     * contact going backward).
     */
    const_reverse_iterator rbegin() const;

    /**
     * @brief Return a const reverse iterator to the contacts (basically starting from the last
     * contact going backward).
     */
    const_reverse_iterator crbegin() const;

    /**
     * @brief Return a const iterator to the end of the list.
     *
     * This is only a placeholder, it does not reference any contact.
     */
    const_iterator end() const;

    /**
     * @brief Return a const iterator to the end of the list.
     *
     * This is only a placeholder, it does not reference any contact.
     */
    const_iterator cend() const;

    /**
     * @brief Return a const reverse iterator to the end of the list.
     *
     * This is only a placeholder, it does not reference any contact.
     */
    const_reverse_iterator rend() const;

    /**
     * @brief Return a const reverse iterator to the end of the list.
     *
     * This is only a placeholder, it does not reference any contact.
     */
    const_reverse_iterator crend() const;

    /**
     * @brief Access contacts by index.
     * @warning This method in a for loop is much less efficient than using iterators.
     * @param index of the phase to be accessed.
     * @return A const reference to the desired contact.
     */
    const BipedalLocomotion::Contacts::PlannedContact& operator[](size_t index) const;

    /**
     * @brief Get the size of the list.
     * @return The number of contacts.
     */
    size_t size() const;

    /**
     * @brief Iterator pointing to the first contact.
     */
    const_iterator firstContact() const;

    /**
     * @brief Iterator pointing to the last contact.
     */
    const_iterator lastContact() const;

    /**
     * @brief Edit an existing contact.
     * @param element Iterator to the element to edit.
     * @param newContact The new contact
     * @return false if the element is not valid or if the new contact timing would require a
     * reordering of the list.
     */
    bool editContact(const_iterator element,
                     const BipedalLocomotion::Contacts::PlannedContact& newContact);

    /**
     * @brief Get the active contact given the time.
     *
     * It returns the active contact (i.e., the highest activation time lower than time and the
     * deactivation time strictly higher than time). If no contacts are active at the given time, it
     * returns an iterator to the end.
     * @param time The present time.
     * @return an iterator to the last contact having an activation time lower than time.
     * If no contact satisfies this condition, it returns a pointer to the end.
     * @note Differently from getPresentContact the contact needs to be active.
     */
    const_iterator getActiveContact(const std::chrono::nanoseconds& time) const;

    /**
     * @brief Get the next contact given the time.
     *
     * It returns the contact with the lowest activation time higher than time.
     * If no contacts have an activation time higher than time, it returns an iterator to the end.
     * @param time The present time.
     * @return an iterator to the first contact having an activation time higher than time.
     * If no contact satisfies this condition, it returns a pointer to the end.
     */
    const_iterator getNextContact(const std::chrono::nanoseconds& time) const;

    /**
     * @brief Get the contact given the time.
     *
     * It returns the contact with the highest activation time lower than time.
     * If no contacts have an activation time lower than time, it returns an iterator to the end.
     * Notice that the contact may not be active, i.e. the deactivationTime may be lower than time.
     * @param time The present time.
     * @return an iterator to the last contact having an activation time lower than time.
     * If no contact satisfies this condition, it returns a pointer to the end.
     */
    const_iterator getPresentContact(const std::chrono::nanoseconds& time) const;

    /**
     * @brief Get the contact given the time.
     *
     * It returns the contact with the highest deactivation time lower than time.
     * If no contacts have a deactivation time lower than time, it returns an iterator to the end.
     * @param time The present time.
     * @return an iterator to the last contact having an activation time lower than time.
     * If no contact satisfies this condition, it returns a pointer to the end.
     */
    const_iterator getPreviousContact(const std::chrono::nanoseconds& time) const;

    /**
     * @brief Clear all the steps, except the one returned by getPresentContact
     * @param time The present time.
     * @return false if no contact is available at this time.
     */
    bool keepOnlyPresentContact(const std::chrono::nanoseconds& time);

    /**
     * @brief Clear the contacts.
     */
    void clear();

    /**
     * @brief Remove only the last contact.
     */
    void removeLastContact();

    /**
     * @brief Convert the contact list to a string.
     *
     * @return A string containing the information of the contact list.
     */
    [[nodiscard]] std::string toString() const;

    /**
     * @brief Force the sample time of the contact list.
     * @param dt The new sample time.
     * @return true if the contact list has been correctly resampled.
     * @note the activation time is rounded down to the nearest multiple of dt, while the
     * deactivation time is rounded up to the nearest multiple of dt.
     * @note If the deactivation time of a contact in the list is equal to
     * `std::chrono::nanoseconds::max()` it will not be rounded up.
     * @warning This will change the sample time of all the contacts in the list. All the
     * iterators to the contacts will be invalidated.
     */
    bool forceSampleTime(const std::chrono::nanoseconds& dt);

    /**
     * @brief Check if the contacts are sampled with the given sample time.
     * @param dt The sample time.
     * @return true if the contacts are sampled with the given sample time.
     * @note If the deactivation time of a contact in the list is equal to
     * `std::chrono::nanoseconds::max()` it will consider it as sampled with the given sample time.
     */
    [[nodiscard]] bool areContactsSampled(const std::chrono::nanoseconds& dt) const;
};

/**
 * @brief Utility alias to a map of ContacLists.
 * @note The key of the map is a user define label representing the contact name.
 */
using ContactListMap = std::unordered_map<std::string, ContactList>;

} // namespace Contacts
} // namespace BipedalLocomotion

#endif // BIPEDAL_LOCOMOTION_CONTACTS_CONTACTLIST_H
