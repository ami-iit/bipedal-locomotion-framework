/**
 * @file UnicycleUtilities.h
 * @authors Lorenzo Moretti,Giulio Romualdi, Stefano Dafarra
 * @copyright 2024 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the BSD-3-Clause license.
 */

#ifndef BIPEDAL_LOCOMOTION_PLANNERS_UNICYCLE_UTILITIES_H
#define BIPEDAL_LOCOMOTION_PLANNERS_UNICYCLE_UTILITIES_H

#include <BipedalLocomotion/Contacts/ContactList.h>
#include <UnicyclePlanner.h>

namespace BipedalLocomotion::Planners::UnicycleUtilities
{
/**
 * It populates the contact list.
 * @param initTime the initial time of the trajectory.
 * @param dt the time step.
 * @param inContact a vector containing the contact status.
 * @param steps a deque containing the steps.
 * @param contactFrameIndex the index of the contact frame.
 * @param contactName the name of the contact.
 * @param contactList the contact list to be populated. It should be empty or contain only the first
 *                    step (i.e., the current one already in contact at time initTime).
 */
bool getContactList(const std::chrono::nanoseconds& initTime,
                    const std::chrono::nanoseconds& dt,
                    const std::vector<bool>& inContact,
                    const std::deque<Step>& steps,
                    const int& contactFrameIndex,
                    const std::string& contactName,
                    BipedalLocomotion::Contacts::ContactList& contactList);

} // namespace BipedalLocomotion::Planners::UnicycleUtilities

#endif // BIPEDAL_LOCOMOTION_PLANNERS_UNICYCLE_UTILITIES_H
