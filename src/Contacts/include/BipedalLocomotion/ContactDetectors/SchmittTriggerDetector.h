/**
 * @file SchmittTriggerDetector.h
 * @authors Prashanth Ramadoss
 * @copyright 2020 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the GNU Lesser General Public License v2.1 or any later version.
 */

#ifndef BIPEDAL_LOCOMOTION_CONTACT_DETECTORS_SCHMITT_TRIGGER_DETECTOR_H
#define BIPEDAL_LOCOMOTION_CONTACT_DETECTORS_SCHMITT_TRIGGER_DETECTOR_H

#include <BipedalLocomotion/ContactDetectors/ContactDetector.h>

#include <iostream>
#include <unordered_map>

namespace BipedalLocomotion
{
namespace Contacts
{

struct SchmittTriggerParams;
struct SchmittTriggerInput;

/**
 * Schmitt Trigger thresholding based contact detector
 * that maintains and updates the contact states for
 * a prescribed set of contacts
 */
class SchmittTriggerDetector : public ContactDetector
{
public:
    SchmittTriggerDetector();
    ~SchmittTriggerDetector();

    /**
     * Set trigger input and time stamp for an existing SchmittTrigger unit
     * @param[in] contactName name of the contact
     * @param[in] time time of measurement
     * @param[in] force contact force intensity (typically contact normal force)
     * @return True in case of success, false otherwise.
     */
    bool setTimedTriggerInput(const std::string& contactName,
                              const double& time,
                              const double& triggerInput);

    /**
     * Set trigger input and time stamp for existing units
     * @param[in] timedInputs container of timed trigger inputs, pair(first, second): (time, force)
     * @note any unit names in the input container that does not already exist will be ignored
     * @return True in case of success, false otherwise.
     */
    bool setTimedTriggerInputs(const std::unordered_map<std::string, SchmittTriggerInput>& timedInputs);

    /**
     * Add a contact whose contact state need to be tracked
     * @param[in] contactName name of the contact
     * @param[in] initialState initial contact state
     * @param[in] params Schmitt Trigger parameters
     * @note this method does not reset the state and parameters if the contact already exists
     * @return True in case of success, false otherwise.
     */
    bool addContact(const std::string& contactName,
                    const bool& initialState,
                    const SchmittTriggerParams& params);
    
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
                    const bool& initialState,
                    const SchmittTriggerParams& params,
                    const double& time_now);
    
    /**
     * Reset a contact's state
     * @param[in] contactName name of the contact
     * @param[in] initialState contact state
     * @return True in case of success, false if contact does not exist/otherwise.
     */
    bool resetState(const std::string& contactName,
                    const bool& state);

    /**
     * Reset a contact's parameters
     * @param[in] contactName name of the contact
     * @param[in] initialState initial contact state
     * @param[in] params Schmitt Trigger parameters
     * @return True in case of success, false if contact does not exist/otherwise.
     */
    bool resetContact(const std::string& contactName,
                      const bool& state,
                      const SchmittTriggerParams& params);

    /**
     * Remove contact from the Detector
     * @param[in] contactName name of the contact
     * @return True in case of success, false if does not exist/otherwise.
     */
    bool removeContact(const std::string& contactName);
protected:
    /**
    * These custom parameter specifications should be specified by the derived class.
    * @param[in] handler configure the custom parameters for the detector
    * @return True in case of success, false otherwise.
    */
    virtual bool customInitialization(std::weak_ptr<BipedalLocomotion::ParametersHandler::IParametersHandler> handler) override;

    /**
    * Update contact states based on thresholding of contact normal forces and timing parameters
    * @return True in case of success, false otherwise.
    */
    virtual bool updateContactStates() override;

private:
    /**
    * Private implementation of the class
    */
    class Impl;
    std::unique_ptr<Impl> m_pimpl; /**< Pointer to implementation */
};


/**
 * Struct holding switching parameters for the Schmitt Trigger
 */
struct SchmittTriggerParams
{
    double onThreshold{0.0};      /**< high value threshold to initiate an ON state switch after switchOnAfter time-units*/
    double offThreshold{0.0};     /**< low value threshold to initiate an OFF state switch after switchOffAfter time-units*/
    double switchOnAfter{0.0};    /**< time units to wait for before switching to ON state from OFF state. Ensure it's greater than sampling time. */
    double switchOffAfter{0.0};   /**< time units to wait for before switching to OFF state from ON state. Ensure it's greater than sampling time. */
};

/**
 * Struct holding Schmitt Trigger inputs
 */
struct SchmittTriggerInput
{
    double time{0.0};    /**< time stamp*/ 
    double value{0.0};   /**< signal input*/
};

/**
 * Schmitt trigger unit that switches state using threshold and timing parameters
 */
class SchmittTriggerUnit
{
public:
    /**
     * Set current state  of the Schmitt trigger
     * @param state current state
     */
    void setState(const bool& state);
    
    /**
     * Set current state  of the Schmitt trigger
     * @param state current state
     * @param time_now time unit to set the timer parameters
     */
    void setState(const bool& state, const double& time_now);

    /**
     * Set configuration parameters of the Schmitt trigger
     * @param params struct holding Schmitt trigger parameters
     */
    void setParams(const SchmittTriggerParams& params);

    /**
     * Update the state of Schmitt trigger with the measurements
     * @param currentTime time of measurement
     * @param rawValue measurement
     * @return True in case of success, false otherwise.
     */
    void update(const double& currentTime, const double& rawValue);

    /**
     * Reset the state of Schmitt trigger to false
     */
    void reset();

    /**
     * Get the current state of the Schmitt trigger
     * @return state - true/false
     */
    bool getState();
    
    /**
     * Get the current state of the Schmitt trigger
     * @param[out] swtichTime
     * @return state - true/false
     */
    bool getState(double& switchTime);

    /**
     * Get currently configuration of Schmitt trigger
     * @return struct holding the parameters
     */
    SchmittTriggerParams getParams();

private:
    SchmittTriggerParams params; /**< Schmitt Trigger parameters*/
    bool state{false}; /**< current state*/
    double switchTime{0.}; /**> time instant at which the state was toggled */
    double previousTime{0.}; /**< previous update time*/
    double timer{0.}; /**< elapsed timer for current state*/
    double initialTime{0.}; /**< initialization time*/
};

} // namespace Contacts
} // namespace BipedalLocomotion

#endif // BIPEDAL_LOCOMOTION_CONTACT_DETECTORS_SCHMITT_TRIGGER_DETECTOR_H


