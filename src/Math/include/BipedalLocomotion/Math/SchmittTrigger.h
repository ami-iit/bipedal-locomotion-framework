/**
 * @file SchmittTrigger.h
 * @authors Giulio Romualdi
 * @copyright 2023 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the BSD-3-Clause license.
 */

#ifndef BIPEDAL_LOCOMOTION_MATH_SCHMITT_TRIGGER_H
#define BIPEDAL_LOCOMOTION_MATH_SCHMITT_TRIGGER_H

#include <memory>

#include <BipedalLocomotion/ParametersHandler/IParametersHandler.h>
#include <BipedalLocomotion/System/Advanceable.h>

namespace BipedalLocomotion
{
namespace Math
{

/**
 * SchmittTriggerState contains the internal state of the trigger,
 */
struct SchmittTriggerState
{
    bool state{false}; /**< current state*/
    double switchTime{0.0}; /**< time instant at which the state was toggled in seconds */
    double edgeTime{0.0}; /**< Time instant at which the raw value transited from low to high of
                             from high to low in seconds.*/
};

/**
 * SchmittTriggerState contains the input of the SchmittTrigger class.
 */
struct SchmittTriggerInput
{
    double time{0.0}; /**< Current time instant in seconds */
    double rawValue{0.0} ; /**< Raw value that should  */
};

/**
 * SchmittTrigger implements a discrete version of a SchmittTrigger
 * See [here](https://en.wikipedia.org/wiki/Schmitt_trigger) for further details.
 */
class SchmittTrigger
    : public BipedalLocomotion::System::Advanceable<SchmittTriggerInput, SchmittTriggerState>
{
public:
    /**
     * Struct holding switching parameters for the Schmitt Trigger
     */
    struct Params
    {
        double onThreshold{0.0}; /**< high value threshold to initiate an ON state switch after
                                    switchOnAfter time-units*/
        double offThreshold{0.0}; /**< low value threshold to initiate an OFF state switch after
                                     switchOffAfter time-units*/
        double switchOnAfter{0.0}; /**< time units to wait for before switching to ON state from OFF
                                      state. Ensure it's greater than sampling time. */
        double switchOffAfter{0.0}; /**< time units to wait for before switching to OFF state from
                                       ON state. Ensure it's greater than sampling time. */

        /** Threshold used for the comparison of two time instant. Given two time instants if the
         * error between the two is lower than the threshold, the time instants are considered
         * equal. */
        double timeComparisonThreshold{std::numeric_limits<double>::epsilon()};
    };

    /**
     * Initialize the SchmittTrigger block.
     * @param handler pointer to the parameter handler.
     * @note The following parameters are required
     * |  Parameter Name  |   Type   |                                     Description                                     | Mandatory |
     * |:----------------:|:--------:|:-----------------------------------------------------------------------------------:|:---------:|
     * |  `on_threshold`  | `double` | High value threshold to initiate an ON state switch after switchOnAfter time-units  |    Yes    |
     * | `off_threshold`  | `double` | Low value threshold to initiate an OFF state switch after switchOffAfter time-units |    Yes    |
     * | `switch_on_after`| `double` | Time units to wait for before switching to ON state from OFF state. Ensure it's greater than sampling time. |     Yes    |
     * |`switch_off_after`| `double` | Time units to wait for before switching to OFF state from ON state. Ensure it's greater than sampling time. |     Yes    |
     * | `time_comparison_threshold`| `double` | Threshold used for the comparison of two time instants. Given two time instants, if the error between the two is lower than the threshold, the time instants are considered equal. Default value [`std::numeric_limits<double>::epsilon()`](https://en.cppreference.com/w/cpp/types/numeric_limits/epsilon) |     No    |
     * @return true in case of success/false otherwise.
     */
    bool initialize(std::weak_ptr<const ParametersHandler::IParametersHandler> handler) override;

    /**
     * Initialize the SchmittTrigger block.
     * @param parameters parameter required to initialize the block.
     * @return true in case of success/false otherwise.
     */
    bool initialize(const Params& parameters);

    /**
     * Set the state of the SchmittTrigger.
     * @param state the state of the SchmittTrigger
     * @note When the state is set the internal timer is reset as well. This function should be
     * called only if the user want to force the state of the system.
     */
    void setState(const SchmittTriggerState& state);

    /**
     * Perform one step of the trigger.
     * @return true in case of success, false otherwise.
     */
    bool advance() override;

    /**
     * Check if the output of the trigger is valid.
     * @return true in case of success, false otherwise.
     */
    bool isOutputValid() const override;

    /**
     * Get the internal state of the SchmittTrigger.
     * @return a const reference to the state of the trigger.
     */
    const SchmittTriggerState& getOutput() const override;

    /**
     * Set the input of the trigger.
     * @param input the input of the system. It contains the raw value and the current time instant.
     * @return true in case of success, false otherwise.
     */
    bool setInput(const SchmittTriggerInput& input) override;

private:
    SchmittTriggerInput m_input; /**< Last input stored in the trigger */
    SchmittTriggerState m_state; /**< Current state stored in the trigger */
    Params m_params; /**< Set of switching parameters */
    double m_timer{0}; /**< Internal timer used by the switcher to understand if it is the time to
                          switch */
    double m_risingEdgeTimeInstant{-1}; /**< Internal quantity used to store the previous time */
    double m_fallingEdgeTimeInstant{-1}; /**< Internal quantity used to store the previous time */
};

} // namespace Math
} // namespace BipedalLocomotion

#endif // BIPEDAL_LOCOMOTION_MATH_SCHMITT_TRIGGER_H
