/**
 * @file ContactDetector.h
 * @authors Prashanth Ramadoss
 * @copyright 2020 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the BSD-3-Clause license.
 */

#ifndef BIPEDAL_LOCOMOTION_CONTACT_DETECTORS_CONTACT_DETECTOR_H
#define BIPEDAL_LOCOMOTION_CONTACT_DETECTORS_CONTACT_DETECTOR_H

#include <unordered_map>

#include <BipedalLocomotion/Contacts/Contact.h>
#include <BipedalLocomotion/ParametersHandler/IParametersHandler.h>
#include <BipedalLocomotion/System/Source.h>

namespace BipedalLocomotion
{
namespace Contacts
{

using EstimatedContactList = std::unordered_map<std::string, Contacts::EstimatedContact>;

class ContactDetector : public BipedalLocomotion::System::Source<EstimatedContactList>
{
public:
    virtual ~ContactDetector() = default;

    /**
     * Determines the validity of the object retrieved with get()
     * @return True in case of success, false otherwise.
     */
    bool isOutputValid() const final;

    /**
     * Get contact states
     * @return container of contacts
     */
    const EstimatedContactList& getOutput() const final;

    /**
     * Reset the internal contact states of the detector to false
     * @return True in case of success, false otherwise.
     */
    bool resetContacts();

    /**
     * Get state of specific contact
     * @param[in] contactName name of contact
     * @param[out] contact estimated contact
     * @return true if contact exists, false otherwise
     */
    bool get(const std::string& contactName, Contacts::EstimatedContact& contact) const;

    /**
     * Get state of specific contact
     * @param[in] contactName name of contact
     * @return contact state if contact exists, a dummy contact otherwise
     */
    Contacts::EstimatedContact get(const std::string& contactName) const;

protected:
    /**
     * Enumerator used to determine the running state of the estimator
     */
    enum class State
    {
        NotInitialized, /**< The estimator is not initialized yet call
                           ContactDetector::initialze method to initialize it*/
        Initialized, /**< The estimator is initialized and ready to be used */
        Running /**< The estimator is running */
    };

    State m_detectorState{State::NotInitialized}; /**< State of the estimator */
    EstimatedContactList m_contactStates;
};

} // namespace Contacts
} // namespace BipedalLocomotion

#endif // BIPEDAL_LOCOMOTION_CONTACT_DETECTORS_CONTACT_DETECTOR_H
