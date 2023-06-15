/**
 * @file SchmittTrigger.h
 * @authors Giulio Romualdi
 * @copyright 2023 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the BSD-3-Clause license.
 */

#ifndef BIPEDAL_LOCOMOTION_MATH_SCHMITT_TRIGGER_H
#define BIPEDAL_LOCOMOTION_MATH_SCHMITT_TRIGGER_H

#include <chrono>
#include <memory>

#include <BipedalLocomotion/Math/Constants.h>
#include <BipedalLocomotion/ParametersHandler/IParametersHandler.h>
#include <BipedalLocomotion/System/Advanceable.h>

namespace BipedalLocomotion
{
namespace Math
{

struct SchmittTriggerOutput
{
    bool state{false}; /**< current state*/

    /** Time instant at which the state was toggled */
    std::chrono::nanoseconds switchTime{std::chrono::nanoseconds::zero()};

    /** Time instant at which the raw value transited from low to high of from high to low .*/
    std::chrono::nanoseconds edgeTime{std::chrono::nanoseconds::zero()};
};

/**
 * SchmittTriggerState contains the input of the SchmittTrigger class.
 */
struct SchmittTriggerInput
{
    /** Current time instant in seconds */
    std::chrono::nanoseconds time{std::chrono::nanoseconds::zero()};
    double rawValue{0.0}; /**< Raw value considered as input by the SchmittTrigger. */
};

/**
 * SchmittTriggerState contains the internal state of the trigger,
 */
struct SchmittTriggerState : public SchmittTriggerOutput
{
    /** Internal timer used by the switcher to understand if it is the time to switch */
    std::chrono::nanoseconds timer{std::chrono::nanoseconds::zero()};

    /**< Internal quantity used to store the time at which a rising edge as been detected */
    std::chrono::nanoseconds risingEdgeTimeInstant{std::chrono::nanoseconds::zero()};

    /**< Internal quantity used to store the time at which a falling edge as been detected */
    std::chrono::nanoseconds fallingEdgeTimeInstant{std::chrono::nanoseconds::zero()};

    bool risingDetected{false}; /**< True if the rising edge is detected */
    bool fallingDetected{false}; /**< True if the falling edge is detected */
};

/**
 * SchmittTrigger implements a discrete version of a SchmittTrigger
 * See [here](https://en.wikipedia.org/wiki/Schmitt_trigger) for further details.
 */
class SchmittTrigger
    : public BipedalLocomotion::System::Advanceable<SchmittTriggerInput, SchmittTriggerOutput>
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

        /** Time to wait before switching to ON state from OFF state. Ensure it's greater than
         * sampling time. */
        std::chrono::nanoseconds switchOnAfter{std::chrono::nanoseconds::zero()};

        /** Time to wait before switching to OFF state from ON state. Ensure it's greater than
         * sampling time. */
        std::chrono::nanoseconds switchOffAfter{std::chrono::nanoseconds::zero()};
    };

    /**
     * Initialize the SchmittTrigger block.
     * @param handler pointer to the parameter handler.
     * @note The following parameters are required
     * |  Parameter Name  |   Type   |                                     Description                                     | Mandatory |
     * |:----------------:|:--------:|:-----------------------------------------------------------------------------------:|:---------:|
     * |  `on_threshold`  | `double` |     High value threshold to initiate an ON state switch after switchOnAfter         |    Yes    |
     * | `off_threshold`  | `double` |     Low value threshold to initiate an OFF state switch after switchOffAfter        |    Yes    |
     * | `switch_on_after`| `chrono:nanoseconds` | Nano seconds to wait for before switching to ON state from OFF state. Ensure it's greater than sampling time. |     Yes    |
     * |`switch_off_after`| `chrono:nanoseconds` | Nano seconds to wait for before switching to OFF state from ON state. Ensure it's greater than sampling time. |     Yes    |
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
     * @param state The desired state of the SchmittTrigger.
     * @return True if the state is successfully set, false otherwise.
     * @note This function should only be called when the user intends to manually control the
     * system state. In this case, all internal variables required by the trigger will be forcefully
     * updated to the provided SchmittTriggerState.
     */
    bool setState(const SchmittTriggerState& state);

    /**
     * Get the state of the SchmittTrigger.
     * @return the internal state of the SchmittTrigger.
     */
    const SchmittTriggerState& getState() const;

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
    const SchmittTriggerOutput& getOutput() const override;

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

    enum class FSM
    {
        Idle,
        Initialized,
        Reset,
        OutputInvalid,
        OutputValid,
    };

    FSM m_fsm{FSM::Idle};
};

} // namespace Math
} // namespace BipedalLocomotion

#endif // BIPEDAL_LOCOMOTION_MATH_SCHMITT_TRIGGER_H
