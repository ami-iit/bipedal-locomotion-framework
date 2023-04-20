/**
 * @file TimeVaryingDCMPlannerTest.cpp
 * @authors Giulio Romualdi
 * @copyright 2020 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the BSD-3-Clause license.
 */

#include <chrono>

// Catch2
#include <catch2/catch_test_macros.hpp>

#include <BipedalLocomotion/Contacts/ContactPhaseList.h>
#include <BipedalLocomotion/Math/Constants.h>
#include <BipedalLocomotion/ParametersHandler/StdImplementation.h>
#include <BipedalLocomotion/Planners/TimeVaryingDCMPlanner.h>

using namespace BipedalLocomotion::Planners;
using namespace BipedalLocomotion::Contacts;
using namespace BipedalLocomotion::ParametersHandler;
using namespace BipedalLocomotion::Math;

TEST_CASE("TimeVaryingDCMPlanner")
{
    using namespace std::chrono_literals;

    ContactPhaseList phaseList;

    ContactListMap contactListMap;

    // left foot
    // first footstep
    Eigen::Vector3d leftPos;
    leftPos << 0, -0.8, 0;
    manif::SE3d leftTransform(leftPos, manif::SO3d::Identity());
    REQUIRE(contactListMap["left"].addContact(leftTransform, 0s, 1s));

    // second footstep
    leftPos(0) = 0.25;
    leftPos(2) = 0.2;
    leftTransform = manif::SE3d(leftPos, manif::SO3d::Identity());
    REQUIRE(contactListMap["left"].addContact(leftTransform, 2s, 7s));

    // right foot
    // first footstep
    Eigen::Vector3d rightPos;
    rightPos << 0, 0.8, 0;
    manif::SE3d rightTransform(rightPos, manif::SO3d::Identity());
    REQUIRE(contactListMap["right"].addContact(rightTransform, 0s, 3s));

    // second footstep
    rightPos(0) = 0.25;
    rightPos(2) = 0.2;
    rightTransform = manif::SE3d(rightPos, manif::SO3d::Identity());
    REQUIRE(contactListMap["right"].addContact(rightTransform, 4s, 7s));
    phaseList.setLists(contactListMap);

    // Set the parameters
    std::shared_ptr<IParametersHandler> handler = std::make_shared<StdImplementation>();
    handler->setParameter("planner_sampling_time", 50ms);
    handler->setParameter("number_of_foot_corners", 4);

    // set the foot-corners
    std::vector<double> footCorner(3);
    footCorner[0] = 0.1;
    footCorner[1] = 0.05;
    footCorner[2] = 0;
    handler->setParameter("foot_corner_0", footCorner);

    footCorner[1] = -0.05;
    handler->setParameter("foot_corner_1", footCorner);

    footCorner[0] = -0.1;
    footCorner[1] = -0.05;
    handler->setParameter("foot_corner_2", footCorner);

    footCorner[1] = 0.05;
    handler->setParameter("foot_corner_3", footCorner);

    // set the weight of the cost function
    handler->setParameter("omega_dot_weight", 1.0);
    handler->setParameter("dcm_tracking_weight", 1.0);
    handler->setParameter("omega_dot_rate_of_change_weight", 10.0);
    handler->setParameter("vrp_rate_of_change_weight", 100.0);
    handler->setParameter("dcm_rate_of_change_weight", 1.0);

    // set the initial state
    DCMPlannerState initialState;
    initialState.dcmPosition.setZero();
    initialState.dcmPosition[2] = 0.53;
    initialState.dcmVelocity.setZero();
    initialState.vrpPosition = initialState.dcmPosition;
    initialState.omega = std::sqrt(StandardAccelerationOfGravitation / initialState.dcmPosition[2]);

    // Initialize the planner
    TimeVaryingDCMPlanner planner;
    REQUIRE(planner.initialize(handler));

    REQUIRE(planner.setContactPhaseList(phaseList));
    planner.setInitialState(initialState);

    REQUIRE(planner.computeTrajectory());

    constexpr std::size_t numberOfIterations = 150;
    for (std::size_t i = 0; i < numberOfIterations; i++)
    {
        REQUIRE(planner.advance());
    }
}
