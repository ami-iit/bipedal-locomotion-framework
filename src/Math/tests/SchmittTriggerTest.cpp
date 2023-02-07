/**
 * @file SchmittTriggerTest.cpp
 * @authors Giulio Romualdi
 * @copyright 2023 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the BSD-3-Clause license.
 */

// Catch2
#include <catch2/catch.hpp>

#include <BipedalLocomotion/Math/SchmittTrigger.h>
#include <BipedalLocomotion/ParametersHandler/StdImplementation.h>

using namespace BipedalLocomotion::Math;

TEST_CASE("Schmitt trigger - Invalid threshold")
{
    SchmittTrigger trigger;
    auto paramHandler = std::make_shared<BipedalLocomotion::ParametersHandler::StdImplementation>();
    paramHandler->setParameter("on_threshold", 0.0);
    paramHandler->setParameter("off_threshold", 0.1);
    paramHandler->setParameter("switch_on_after", 0.5);
    paramHandler->setParameter("switch_off_after", 0.5);

    REQUIRE_FALSE(trigger.initialize(paramHandler));
}

TEST_CASE("Schmitt trigger - Same threshold")
{
    SchmittTrigger trigger;
    auto paramHandler = std::make_shared<BipedalLocomotion::ParametersHandler::StdImplementation>();
    constexpr double switchOffAfter = 0.2;
    constexpr double switchOnAfter = 0.2;
    constexpr double onThreshold = 0;
    constexpr double offThreshold = 0;
    paramHandler->setParameter("on_threshold", onThreshold);
    paramHandler->setParameter("off_threshold", offThreshold);
    paramHandler->setParameter("switch_on_after", switchOnAfter);
    paramHandler->setParameter("switch_off_after", switchOffAfter);

    REQUIRE(trigger.initialize(paramHandler));

    SchmittTriggerState initialState;
    trigger.setState(initialState);

    constexpr double samplingTime = 0.01;
    constexpr double timeWindow = 10;
    for (unsigned int i = 0; i < timeWindow / samplingTime; i++)
    {
        const double t = samplingTime * i;
        const double signal = 10 * std::sin(2 * M_PI * t);

        trigger.setInput({t, signal});
        REQUIRE(trigger.advance());

        if (t < 0.2)
        {
            // In this time slot the output should be false
            REQUIRE_FALSE(trigger.getOutput().state);
        }
        else if (t < 0.69)
        {
            // In this time slot the output should be true
            REQUIRE(trigger.getOutput().state);
        }

    }
}
