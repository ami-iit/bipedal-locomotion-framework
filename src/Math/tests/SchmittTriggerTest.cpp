/**
 * @file SchmittTriggerTest.cpp
 * @authors Giulio Romualdi
 * @copyright 2023 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the BSD-3-Clause license.
 */

#include <chrono>

// Catch2
#include <catch2/catch_test_macros.hpp>

#include <BipedalLocomotion/Math/SchmittTrigger.h>
#include <BipedalLocomotion/ParametersHandler/StdImplementation.h>

using namespace BipedalLocomotion::Math;

TEST_CASE("Schmitt trigger - Invalid threshold")
{
    using namespace std::chrono_literals;

    SchmittTrigger trigger;
    auto paramHandler = std::make_shared<BipedalLocomotion::ParametersHandler::StdImplementation>();
    paramHandler->setParameter("on_threshold", 0.0);
    paramHandler->setParameter("off_threshold", 0.1);
    paramHandler->setParameter("switch_on_after", 500ms);
    paramHandler->setParameter("switch_off_after", 500ms);

    REQUIRE_FALSE(trigger.initialize(paramHandler));
}

TEST_CASE("Schmitt trigger - Same threshold")
{
    using namespace std::chrono_literals;

    SchmittTrigger trigger;
    auto paramHandler = std::make_shared<BipedalLocomotion::ParametersHandler::StdImplementation>();
    constexpr std::chrono::nanoseconds switchOffAfter = 200ms;
    constexpr std::chrono::nanoseconds switchOnAfter = 200ms;
    constexpr double onThreshold = 0;
    constexpr double offThreshold = 0;
    paramHandler->setParameter("on_threshold", onThreshold);
    paramHandler->setParameter("off_threshold", offThreshold);
    paramHandler->setParameter("switch_on_after", switchOnAfter);
    paramHandler->setParameter("switch_off_after", switchOffAfter);

    REQUIRE(trigger.initialize(paramHandler));

    SchmittTriggerState initialState;
    trigger.setState(initialState);

    constexpr std::chrono::nanoseconds samplingTime = 10ms;
    constexpr std::chrono::nanoseconds timeWindow = 10s;
    for (unsigned int i = 0; i < timeWindow / samplingTime; i++)
    {
        const std::chrono::nanoseconds t = samplingTime * i;
        const double signal = 10 * std::sin(2 * M_PI * std::chrono::duration<double>(t).count());

        trigger.setInput({t, signal});
        REQUIRE(trigger.advance());

        if (t < 200ms)
        {
            // In this time slot the output should be false
            REQUIRE_FALSE(trigger.getOutput().state);
        }
        else if (t < 690ms)
        {
            // In this time slot the output should be true
            REQUIRE(trigger.getOutput().state);
        }

    }
}
