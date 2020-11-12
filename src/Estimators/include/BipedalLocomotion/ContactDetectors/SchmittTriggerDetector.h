/**
 * @file SchmittTriggerDetector.h
 * @authors Prashanth Ramadoss
 * @copyright 2020 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the GNU Lesser General Public License v2.1 or any later version.
 */

#ifndef BIPEDAL_LOCOMOTION_SCHMITT_TRIGGER_DETECTOR_H
#define BIPEDAL_LOCOMOTION_SCHMITT_TRIGGER_DETECTOR_H

#include <BipedalLocomotion/ContactDetectors/ContactDetector.h>

#include <iostream>
#include <unordered_map>

namespace BipedalLocomotion
{
namespace Estimators
{

struct SchmittTriggerParams;

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
     * Set contact force intensity and time stamp for an existing contact
     * @param[in] contactName Internal state of the estimator
     * @param[in] time time of measurement
     * @param[in] force contact force intensity (typically contact normal force)
     * @return True in case of success, false otherwise.
     */
    bool setTimedContactIntensity(const std::string& contactName,
                                  const double& time,
                                  const double& force);

    /**
     * Set contact force intensity and time stamp for existing contacts
     * @param[in] timedForces container of timed force intesities, pair(first, second): (time, force)
     * @note any contacts in the input container that does not already exist will be ignored
     * @return True in case of success, false otherwise.
     */
    bool setTimedContactIntensities(const std::unordered_map<std::string, std::pair<double, double>>& timedForces);

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
     * Get currently configuration of Schmitt trigger
     * @return struct holding the parameters
     */
    SchmittTriggerParams getParams();

private:
    SchmittTriggerParams params; /**< Schmitt Trigger parameters*/
    bool state{false}; /**< current state*/
    double previousTime{0.}; /**< previous update time*/
    double timer{0.}; /**< elapsed timer for current state*/
};

} // namespace Estimators
} // namespace BipedalLocomotion

#endif // BIPEDAL_LOCOMOTION_SCHMITT_TRIGGER_DETECTOR_H


