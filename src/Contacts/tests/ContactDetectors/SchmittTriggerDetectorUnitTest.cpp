/**
 * @file SchmittTriggerDetectorUnitTest.cpp
 * @authors Prashanth Ramadoss, Giulio Romualdi
 * @copyright 2020-2023 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the BSD-3-Clause license.
 */

#include <chrono>

#include <catch2/catch_test_macros.hpp>

#include <BipedalLocomotion/ContactDetectors/SchmittTriggerDetector.h>
#include <BipedalLocomotion/Math/SchmittTrigger.h>
#include <BipedalLocomotion/ParametersHandler/IParametersHandler.h>
#include <BipedalLocomotion/ParametersHandler/StdImplementation.h>

using namespace BipedalLocomotion::ParametersHandler;
using namespace BipedalLocomotion::Contacts;
using namespace BipedalLocomotion::Math;

TEST_CASE("Schmitt Trigger Detector")
{
    SchmittTriggerDetector detector;

    using namespace std::chrono_literals;

    std::shared_ptr<StdImplementation> originalHandler = std::make_shared<StdImplementation>();
    IParametersHandler::shared_ptr parameterHandler = originalHandler;

    parameterHandler->setParameter("contacts", std::vector<std::string>{"right"});
    parameterHandler->setParameter("contact_make_thresholds", std::vector<double>{100});
    parameterHandler->setParameter("contact_break_thresholds", std::vector<double>{10});
    parameterHandler->setParameter("contact_make_switch_times",
                                   std::vector<std::chrono::nanoseconds>{200ms});
    parameterHandler->setParameter("contact_break_switch_times",
                                   std::vector<std::chrono::nanoseconds>{200ms, 200ms});

    // mismatch in parameter size shoulc cause initialization failure
    REQUIRE_FALSE(detector.initialize(parameterHandler));

    // initialization should pass
    parameterHandler->setParameter("contact_break_switch_times",
                                   std::vector<std::chrono::nanoseconds>{200ms});
    REQUIRE(detector.initialize(parameterHandler));

    // set all contacts to false
    REQUIRE(detector.resetContacts());

    // rise signal
    detector.setTimedTriggerInput("right", {100ms ,120});
    detector.advance();
    detector.setTimedTriggerInput("right", {200ms, 120});
    detector.advance();
    detector.setTimedTriggerInput("right", {300ms, 120});
    detector.advance();

    // contact state should turn true
    EstimatedContact rightContact;
    REQUIRE(detector.get("right", rightContact));
    REQUIRE(rightContact.isActive);
    REQUIRE(rightContact.switchTime == 300ms);

    // fall signal
    detector.setTimedTriggerInput("right", {400ms, 7});
    detector.advance();
    detector.setTimedTriggerInput("right", {500ms, 7});
    detector.advance();
    detector.setTimedTriggerInput("right", {600ms, 7});
    detector.advance();

    // contact state should turn false
    REQUIRE(detector.get("right", rightContact));
    REQUIRE(!rightContact.isActive);
    REQUIRE(rightContact.switchTime == 600ms);

    // add a new contact
    EstimatedContact newContact;
    SchmittTrigger::Params params;
    params.offThreshold = 10;
    params.onThreshold = 100;
    params.switchOffAfter = 200ms;
    params.switchOnAfter = 200ms;

    detector.addContact("left", {false, 600ms, 600ms}, params);
    auto contacts = detector.getOutput();
    REQUIRE(contacts.size() == 2);

    // test multiple measurement updates
    std::unordered_map<std::string, SchmittTriggerInput> timedForces;
    timedForces["right"] = {700ms, 120};
    timedForces["left"] = {700ms, 120};
    detector.setTimedTriggerInputs(timedForces);
    detector.advance();

    timedForces["right"] = {800ms, 120};
    timedForces["left"] = {800ms, 120};
    detector.setTimedTriggerInputs(timedForces);
    detector.advance();

    timedForces["right"] = {900ms, 120};
    timedForces["left"] = {900ms, 120};
    detector.setTimedTriggerInputs(timedForces);
    detector.advance();

    contacts = detector.getOutput();
    REQUIRE(contacts["right"].isActive);
    REQUIRE(contacts["left"].isActive);
    REQUIRE(contacts["right"].switchTime == 900ms);

    // Test removing contact
    REQUIRE(detector.removeContact("left"));
    contacts = detector.getOutput();
    REQUIRE(contacts.size() == 1);

    // Test resetting contact
    detector.resetContact("right", false, params);
    REQUIRE(detector.get("right", rightContact));
    REQUIRE(!rightContact.isActive);
}
