/**
 * @file SchmittTriggerDetectorUnitTest.cpp
 * @authors Prashanth Ramadoss
 * @copyright 2020 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the GNU Lesser General Public License v2.1 or any later version.
 */

#include <catch2/catch.hpp>

#include <BipedalLocomotion/ParametersHandler/IParametersHandler.h>
#include <BipedalLocomotion/ParametersHandler/StdImplementation.h>
#include <BipedalLocomotion/ContactDetectors/SchmittTriggerDetector.h>

using namespace BipedalLocomotion::Estimators;
using namespace BipedalLocomotion::ParametersHandler;
using namespace BipedalLocomotion::Planners;

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
    REQUIRE(detector.initialize(parameterHandler) == false);

    // initialization should pass
    parameterHandler->setParameter("contact_break_switch_times", std::vector<double>{0.2});
    REQUIRE(detector.initialize(parameterHandler) == true);

    // set all contacts to false
    detector.resetContacts();

    // rise signal
    detector.setTimedContactIntensity("right", 0.1, 120);
    detector.advance();
    detector.setTimedContactIntensity("right", 0.2, 120);
    detector.advance();
    detector.setTimedContactIntensity("right", 0.3, 120);
    detector.advance();

    // contact state should turn true
    auto rightContact = detector.get("right");
    REQUIRE(rightContact.isActive);

    // fall signal
    detector.setTimedContactIntensity("right", 0.4, 7);
    detector.advance();
    detector.setTimedContactIntensity("right", 0.5, 7);
    detector.advance();
    detector.setTimedContactIntensity("right", 0.6, 7);
    detector.advance();

    // contact state should turn false
    rightContact = detector.get("right");
    REQUIRE(!rightContact.isActive);

    // add a new contact
    Contact newContact;
    SchmittTriggerParams params;
    params.offThreshold = 10;
    params.onThreshold = 100;
    params.switchOffAfter = 0.2;
    params.switchOnAfter = 0.2;

    detector.addContact("left", false, params);
    auto contacts = detector.get();
    REQUIRE(contacts.size() == 2);

    // test multiple measurement updates
    std::unordered_map<std::string, std::pair<double, double> > timedForces;
    timedForces["right"] = std::make_pair(0.7, 120);
    timedForces["left"] = std::make_pair(0.7, 120);
    detector.setTimedContactIntensities(timedForces);
    detector.advance();

    timedForces["right"] = std::make_pair(0.8, 120);
    timedForces["left"] = std::make_pair(0.8, 120);
    detector.setTimedContactIntensities(timedForces);
    detector.advance();

    timedForces["right"] = std::make_pair(0.9, 120);
    timedForces["left"] = std::make_pair(0.9, 120);
    detector.setTimedContactIntensities(timedForces);
    detector.advance();

    contacts = detector.get();
    REQUIRE(contacts["right"].isActive);
    REQUIRE(contacts["left"].isActive);

    // Test removing contact
    REQUIRE(detector.removeContact("left"));
    contacts = detector.get();
    REQUIRE(contacts.size() == 1);

    // Test resetting contact
    detector.resetContact("right", false, params);
    rightContact = detector.get("right");
    REQUIRE(!rightContact.isActive);
}

