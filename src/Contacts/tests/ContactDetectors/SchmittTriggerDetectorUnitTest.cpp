/**
 * @file SchmittTriggerDetectorUnitTest.cpp
 * @authors Prashanth Ramadoss, Giulio Romualdi
 * @copyright 2020-2023 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the BSD-3-Clause license.
 */

#include <catch2/catch.hpp>

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

    std::shared_ptr<StdImplementation> originalHandler = std::make_shared<StdImplementation>();
    IParametersHandler::shared_ptr parameterHandler = originalHandler;

    parameterHandler->setParameter("contacts", std::vector<std::string>{"right"});
    parameterHandler->setParameter("contact_make_thresholds", std::vector<double>{100});
    parameterHandler->setParameter("contact_break_thresholds", std::vector<double>{10});
    parameterHandler->setParameter("contact_make_switch_times", std::vector<double>{0.2});
    parameterHandler->setParameter("contact_break_switch_times", std::vector<double>{0.2, 0.2});

    // mismatch in parameter size shoulc cause initialization failure
    REQUIRE_FALSE(detector.initialize(parameterHandler));

    // initialization should pass
    parameterHandler->setParameter("contact_break_switch_times", std::vector<double>{0.2});
    REQUIRE(detector.initialize(parameterHandler));

    // set all contacts to false
    REQUIRE(detector.resetContacts());

    // rise signal
    detector.setTimedTriggerInput("right", SchmittTriggerInput{.time = 0.1, .rawValue = 120});
    detector.advance();
    detector.setTimedTriggerInput("right", SchmittTriggerInput{.time = 0.2, .rawValue = 120});
    detector.advance();
    detector.setTimedTriggerInput("right", SchmittTriggerInput{.time = 0.3, .rawValue = 120});
    detector.advance();

    // contact state should turn true
    EstimatedContact rightContact;
    REQUIRE(detector.get("right", rightContact));
    REQUIRE(rightContact.isActive);
    REQUIRE(rightContact.switchTime == 0.3);

    // fall signal
    detector.setTimedTriggerInput("right", SchmittTriggerInput{.time = 0.4, .rawValue = 7});
    detector.advance();
    detector.setTimedTriggerInput("right", SchmittTriggerInput{.time = 0.5, .rawValue = 7});
    detector.advance();
    detector.setTimedTriggerInput("right", SchmittTriggerInput{.time = 0.6, .rawValue = 7});
    detector.advance();

    // contact state should turn false
    REQUIRE(detector.get("right", rightContact));
    REQUIRE(!rightContact.isActive);
    REQUIRE(rightContact.switchTime == 0.6);

    // add a new contact
    EstimatedContact newContact;
    SchmittTrigger::Params params;
    params.offThreshold = 10;
    params.onThreshold = 100;
    params.switchOffAfter = 0.2;
    params.switchOnAfter = 0.2;

    detector.addContact("left",
                        SchmittTriggerState{.state = false, .switchTime = 0.6, .edgeTime = 0.6},
                        params);
    auto contacts = detector.getOutput();
    REQUIRE(contacts.size() == 2);

    // test multiple measurement updates
    std::unordered_map<std::string, SchmittTriggerInput> timedForces;
    timedForces["right"] = {0.7, 120};
    timedForces["left"] = {0.7, 120};
    detector.setTimedTriggerInputs(timedForces);
    detector.advance();

    timedForces["right"] = {0.8, 120};
    timedForces["left"] = {0.8, 120};
    detector.setTimedTriggerInputs(timedForces);
    detector.advance();

    timedForces["right"] = {0.9, 120};
    timedForces["left"] = {0.9, 120};
    detector.setTimedTriggerInputs(timedForces);
    detector.advance();

    contacts = detector.getOutput();
    REQUIRE(contacts["right"].isActive);
    REQUIRE(contacts["left"].isActive);
    REQUIRE(contacts["right"].switchTime == 0.9);

    // Test removing contact
    REQUIRE(detector.removeContact("left"));
    contacts = detector.getOutput();
    REQUIRE(contacts.size() == 1);

    // Test resetting contact
    detector.resetContact("right", false, params);
    REQUIRE(detector.get("right", rightContact));
    REQUIRE(!rightContact.isActive);
}
