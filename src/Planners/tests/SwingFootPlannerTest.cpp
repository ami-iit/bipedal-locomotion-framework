/**
 * @file SwingFootPlannerTest.cpp
 * @authors Giulio Romualdi
 * @copyright 2020 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the BSD-3-Clause license.
 */

#include <chrono>

// Catch2
#include <catch2/catch.hpp>

#include <BipedalLocomotion/ParametersHandler/StdImplementation.h>
#include <BipedalLocomotion/Contacts/ContactList.h>
#include <BipedalLocomotion/Planners/SwingFootPlanner.h>

#include <manif/SE3.h>

using namespace BipedalLocomotion::Contacts;
using namespace BipedalLocomotion::Planners;
using namespace BipedalLocomotion::ParametersHandler;

TEST_CASE("Swing foot planner")
{
    using namespace std::chrono_literals;

    ContactList contactList;

    // first footstep
    manif::SE3d transform{{0.9, 0, 0}, Eigen::AngleAxisd(1.5708, Eigen::Vector3d::UnitZ())};
    REQUIRE(contactList.addContact(transform, 0s, 300ms));

    // second
    transform = manif::SE3d({0.8899, 0.1345, 0.0600},
                            Eigen::AngleAxisd(1.7208, Eigen::Vector3d::UnitZ()));
    REQUIRE(contactList.addContact(transform, 600ms, 1s + 500ms));

    // third
    transform = manif::SE3d({0.8104, 0.3915, 0.1800},
                            Eigen::AngleAxisd(2.0208, Eigen::Vector3d::UnitZ()));
    REQUIRE(contactList.addContact(transform, 1s + 800ms, 2s + 700ms));

    // forth
    transform = manif::SE3d({0.6585, 0.6135, 0.3000},
                            Eigen::AngleAxisd(2.3208, Eigen::Vector3d::UnitZ()));
    REQUIRE(contactList.addContact(transform, 3s, 3s + 900ms));

    // fifth
    transform = manif::SE3d({0.3261, 0.8388, 0.4800},
                            Eigen::AngleAxisd(2.7708, Eigen::Vector3d::UnitZ()));
    REQUIRE(contactList.addContact(transform, 4s, 6s));

    // Set the parameters
    std::shared_ptr<IParametersHandler> handler = std::make_shared<StdImplementation>();
    constexpr std::chrono::nanoseconds dT = 10ms;
    handler->setParameter("sampling_time", dT);
    handler->setParameter("step_height", 0.1);
    handler->setParameter("foot_apex_time", 0.5);
    handler->setParameter("foot_landing_velocity", 0.0);
    handler->setParameter("foot_landing_acceleration", 0.0);
    handler->setParameter("foot_take_off_velocity", 0.0);
    handler->setParameter("foot_take_off_acceleration", 0.0);

    // initialize the planner
    SwingFootPlanner planner;
    REQUIRE(planner.initialize(handler));
    planner.setContactList(contactList);

    std::cout << "pos = [";
    for (std::chrono::nanoseconds time = contactList.firstContact()->activationTime;
         time < contactList.lastContact()->deactivationTime;
         time += dT)
    {
        std::cout << planner.getOutput().transform.translation().transpose() << " "
                  << planner.getOutput().transform.quat().w() << " "
                  << planner.getOutput().transform.quat().vec().transpose() << std::endl;

        // advance the planner
        REQUIRE(planner.advance());
    }

    std::cout << "];" << std::endl;
}
