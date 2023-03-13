/**
 * @file SchmittTriggerDetector.h
 * @authors Prashanth Ramadoss, Giulio Romualdi
 * @copyright 2020-2023 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the BSD-3-Clause license.
 */

#ifndef BIPEDAL_LOCOMOTION_CONTACT_DETECTORS_SCHMITT_TRIGGER_DETECTOR_H
#define BIPEDAL_LOCOMOTION_CONTACT_DETECTORS_SCHMITT_TRIGGER_DETECTOR_H

#include <unordered_map>

#include <BipedalLocomotion/ContactDetectors/ContactDetector.h>
#include <BipedalLocomotion/Math/SchmittTrigger.h>

namespace BipedalLocomotion
{
namespace Contacts
{

/**
 * Schmitt Trigger thresholding based contact detector that maintains and updates the contact states
 * for a prescribed set of contacts.
 */
class SchmittTriggerDetector : public ContactDetector
{
public:

    /**
     * Constructor.
     * It is required by the pimpl idiom.
     */
    SchmittTriggerDetector();

    /**
     * Destructor.
     * It is required by the pimpl idiom.
     */
    ~SchmittTriggerDetector();

    /**
    * Initialize the SchmittTriggerDetector witn a parameters handler.
    * @param[in] handler configure the custom parameters for the detector.
    * @note The following parameters are required
    * |        Parameter Name        |       Type       |                                                                                             Description                                                                                             | Mandatory |
    * |:----------------------------:|:----------------:|:---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------:|:---------:|
    * |          `contacts`          | `vector<string>` |                               Vector containing the names of the contact. The name can be used to retrieve the current status of a contact with `ContactDetector::get`.                             |    Yes    |
    * |   `contact_make_thresholds`  | `vector<double>` | Vector containing High-value thresholds to initiate an ON state switch after switchOnAfter time-units. For each contact specified in `contacts` the user needs to specify an element of the vector. |    Yes    |
    * |  `contact_break_thresholds`  | `vector<double>` | Vector containing Low-value thresholds to initiate an ON state switch after switchOnAfter time-units. For each contact specified in `contacts`  the user needs to specify an element of the vector. |    Yes    |
    * |  `contact_make_switch_times` | `vector<double>` |           Time units to wait for before switching to `contact` state from `no-contact` state. For each contact specified in `contacts` the user needs to specify an element of the vector.          |    Yes    |
    * | `contact_break_switch_times` | `vector<double>` |          Time units to wait for before switching to `no-contact`  state from `contact`  state. For each contact specified in `contacts` the user needs to specify an element of the vector.         |    Yes    |
    * @return True in case of success, false otherwise.
    */
    bool initialize(std::weak_ptr<const ParametersHandler::IParametersHandler> handler) override;

    /**
     * Set trigger input and time stamp for an existing SchmittTrigger unit
     * @param[in] contactName name of the contact
     * @param[in] input the imput of the trigger containing the contact force and the time instant
     * @return True in case of success, false otherwise.
     */
    bool setTimedTriggerInput(const std::string& contactName, //
                              const Math::SchmittTriggerInput& input);

    /**
     * Set trigger input and time stamp for existing units
     * @param[in] timedInputs container of timed trigger inputs, pair(first, second): (time, force)
     * @note any unit names in the input container that does not already exist will be ignored
     * @return True in case of success, false otherwise.
     */
    bool setTimedTriggerInputs(const std::unordered_map<std::string, //
                                                        Math::SchmittTriggerInput>& timedInputs);

    /**
     * Add a contact whose contact state need to be tracked
     * @param[in] contactName name of the contact
     * @param[in] initialState initial contact state
     * @param[in] params Schmitt Trigger parameters
     * @param[in] time_now current time instant to initialize the Schmitt trigger timers
     * @note this method does not reset the state and parameters if the contact already exists
     * @return True in case of success, false otherwise.
     */
    bool addContact(const std::string& contactName,
                    const BipedalLocomotion::Math::SchmittTriggerState& initialState,
                    const BipedalLocomotion::Math::SchmittTrigger::Params& params);

    /**
     * Reset a contact's state
     * @param[in] contactName name of the contact
     * @param[in] initialState contact state
     * @return True in case of success, false if contact does not exist/otherwise.
     */
    bool resetState(const std::string& contactName, const bool& state);

    /**
     * Reset a contact's parameters
     * @param[in] contactName name of the contact
     * @param[in] initialState initial contact state
     * @param[in] params Schmitt Trigger parameters
     * @return True in case of success, false if contact does not exist/otherwise.
     */
    bool resetContact(const std::string& contactName,
                      const bool state,
                      const BipedalLocomotion::Math::SchmittTrigger::Params& params);

    /**
     * Remove contact from the Detector
     * @param[in] contactName name of the contact
     * @return True in case of success, false if does not exist/otherwise.
     */
    bool removeContact(const std::string& contactName);

    /**
     * Compute one step of the detector.
     * This method uses all the inputs and evaluate the contact status considering the Schmitt
     * trigger output.
     * @return True in case of success, false otherwise.
     */
    bool advance() final;

private:

    /**
    * Private implementation of the class
    */
    class Impl;
    std::unique_ptr<Impl> m_pimpl; /**< Pointer to implementation */
};

} // namespace Contacts
} // namespace BipedalLocomotion

#endif // BIPEDAL_LOCOMOTION_CONTACT_DETECTORS_SCHMITT_TRIGGER_DETECTOR_H
