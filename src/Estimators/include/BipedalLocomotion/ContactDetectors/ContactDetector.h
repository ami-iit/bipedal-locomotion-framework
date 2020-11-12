/**
 * @file ContactDetector.h
 * @authors Prashanth Ramadoss
 * @copyright 2020 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the GNU Lesser General Public License v2.1 or any later version.
 */

#ifndef BIPEDAL_LOCOMOTION_CONTACT_DETECTOR_H
#define BIPEDAL_LOCOMOTION_CONTACT_DETECTOR_H

#include <BipedalLocomotion/System/Advanceable.h>
#include <BipedalLocomotion/ParametersHandler/IParametersHandler.h>
#include <BipedalLocomotion/Planners/Contact.h>

#include <iostream>
#include <unordered_map>

namespace BipedalLocomotion
{
namespace Estimators
{

using ContactStates = std::unordered_map<std::string, BipedalLocomotion::Planners::Contact>;

class ContactDetector : public BipedalLocomotion::System::Advanceable<ContactStates>
{
public:
    virtual ~ContactDetector() { };

    /**
    * Configure generic parameters
    * @param[in] handler configure the generic parameters for the estimator
    * @return True in case of success, false otherwise.
    */
    bool initialize(std::weak_ptr<BipedalLocomotion::ParametersHandler::IParametersHandler> handler);

    /**
    * Compute one step of the detector
    * The derived class must implement its own methods for setting measurements
    * and update states to be called within the advance() method
    * @return True in case of success, false otherwise.
    */
    virtual bool advance() final;

    /**
     * Reset the internal contact states of the detector to false
     * @return True in case of success, false otherwise.
     *
     */
    virtual bool resetContacts() final;

    /**
    * Get contact states
    * @return container of contacts
    */
    virtual const ContactStates& get() const final;

    /**
     * Get state of specific contact
     * @param[in] contactName name of contact
     * @return contact state if contact exists, false otherwise
     */
    BipedalLocomotion::Planners::Contact get(const std::string& contactName);

    /**
    * Determines the validity of the object retrieved with get()
    * @return True in case of success, false otherwise.
    */
    virtual bool isValid() const final;

protected:
    /**
    * These custom parameter specifications should be specified by the derived class.
    * @param[in] handler configure the custom parameters for the estimator
    * @return True if success, false otherwise
    */
    virtual bool customInitialization(std::weak_ptr<BipedalLocomotion::ParametersHandler::IParametersHandler> handler);

    /**
    * The derived class must implement the contact detection technique to update the contact states
    * @return True if success, false otherwise
    */
    virtual bool updateContactStates();


    /**
    * Enumerator used to determine the running state of the estimator
    */
    enum class State
    {
        NotInitialized, /**< The estimator is not initialized yet call FloatingBaseEstimator::initialze
                           method to initialize it*/
        Initialized,    /**< The estimator is initialized and ready to be used */
        Running         /**< The estimator is running */
    };

    State m_detectorState{State::NotInitialized}; /**< State of the estimator */
    ContactStates m_contactStates;
};


} // namespace Estimators
} // namespace BipedalLocomotion

#endif // BIPEDAL_LOCOMOTION_CONTACT_DETECTOR_H

