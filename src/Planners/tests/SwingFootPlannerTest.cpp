/**
 * @file SwingFootPlannerTest.cpp
 * @authors Giulio Romualdi
 * @copyright 2020 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the GNU Lesser General Public License v2.1 or any later version.
 */

// Catch2
#include <catch2/catch.hpp>

#include <BipedalLocomotion/ParametersHandler/StdImplementation.h>
#include <BipedalLocomotion/Planners/ContactList.h>
#include <BipedalLocomotion/Planners/SwingFootPlanner.h>

#include <manif/SE3.h>

using namespace BipedalLocomotion::Planners;
using namespace BipedalLocomotion::ParametersHandler;

TEST_CASE("Swing foot planner")
{
    ContactList contactList;

    // first footstep
    manif::SE3d transform{{0.9, 0, 0}, Eigen::AngleAxisd(1.5708, Eigen::Vector3d::UnitZ())};
    REQUIRE(contactList.addContact(transform, 0.0, 0.3));

    // second
    transform = manif::SE3d({0.8899, 0.1345, 0.0600},
                            Eigen::AngleAxisd(1.7208, Eigen::Vector3d::UnitZ()));
    REQUIRE(contactList.addContact(transform, 0.6, 1.5));

    // third
    transform = manif::SE3d({0.8104, 0.3915, 0.1800},
                            Eigen::AngleAxisd(2.0208, Eigen::Vector3d::UnitZ()));
    REQUIRE(contactList.addContact(transform, 1.8, 2.7));

    // forth
    transform = manif::SE3d({0.6585, 0.6135, 0.3000},
                            Eigen::AngleAxisd(2.3208, Eigen::Vector3d::UnitZ()));
    REQUIRE(contactList.addContact(transform, 3.0, 3.9));

    // fifth
    transform = manif::SE3d({0.3261, 0.8388, 0.4800},
                            Eigen::AngleAxisd(2.7708, Eigen::Vector3d::UnitZ()));
    REQUIRE(contactList.addContact(transform, 4.2, 6.0));

    // Set the parameters
    std::shared_ptr<IParametersHandler> handler = std::make_shared<StdImplementation>();
    constexpr double dT = 0.01;
    handler->setParameter("sampling_time", dT);
    handler->setParameter("step_height", 0.1);
    handler->setParameter("foot_apex_time", 0.5);
    handler->setParameter("foot_landing_velocity", 0.0);
    handler->setParameter("foot_landing_acceleration", 0.0);

    // initialize the planner
    SwingFootPlanner planner;
    REQUIRE(planner.initialize(handler));
    planner.setContactList(contactList);

    std::cout << "pos = [";
    const std::size_t numberOfIterations = (contactList.lastContact()->deactivationTime + 1) / dT;
    for (std::size_t i = 0; i < numberOfIterations; i++)
    {
        std::cout << planner.get().transform.translation().transpose() << " "
                  << planner.get().transform.quat().w() << " "
                  << planner.get().transform.quat().vec().transpose() << std::endl;

        // advance the planner
        REQUIRE(planner.advance());
    }

    std::cout << "];" << std::endl;
}
