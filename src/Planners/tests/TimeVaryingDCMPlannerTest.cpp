/**
 * @file TimeVaryingDCMPlannerTest.cpp
 * @authors Giulio Romualdi
 * @copyright 2020 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the GNU Lesser General Public License v2.1 or any later version.
 */

// Catch2
#include <catch2/catch.hpp>

#include <BipedalLocomotion/ParametersHandler/StdImplementation.h>
#include <BipedalLocomotion/Planners/ContactPhaseList.h>
#include <BipedalLocomotion/Planners/TimeVaryingDCMPlanner.h>

#include <iDynTree/Core/VectorFixSize.h>

using namespace BipedalLocomotion::Planners;
using namespace BipedalLocomotion::ParametersHandler;

TEST_CASE("TimeVaryingDCMPlanner")
{
    auto phaseList = std::make_shared<ContactPhaseList>();

    ContactListMap contactListMap;

    iDynTree::Transform leftTransform = iDynTree::Transform::Identity();
    iDynTree::Position leftPosition;

    leftPosition[0] = 0;
    leftPosition[1] = -0.8;
    leftPosition[2] = 0;
    leftTransform.setPosition(leftPosition);
    REQUIRE(contactListMap["left"].addContact(leftTransform, 0.0, 1.0));

    leftPosition[0] = 0.25;
    leftPosition[2] = 0.2;
    leftTransform.setPosition(leftPosition);
    REQUIRE(contactListMap["left"].addContact(leftTransform, 2.0, 7.0));

    iDynTree::Transform rightTransform = iDynTree::Transform::Identity();
    iDynTree::Position rightPosition;

    rightPosition[0] = 0;
    rightPosition[1] = 0.8;
    rightPosition[2] = 0;
    rightTransform.setPosition(rightPosition);
    REQUIRE(contactListMap["right"].addContact(rightTransform, 0.0, 3.0));

    rightPosition[0] = 0.25;
    rightPosition[2] = 0.2;
    rightTransform.setPosition(rightPosition);
    REQUIRE(contactListMap["right"].addContact(rightTransform, 4.0, 7.0));

    phaseList->setLists(contactListMap);

    // Set the parameters
    std::shared_ptr<IParametersHandler> handler = std::make_shared<StdImplementation>();
    handler->setParameter("planner_sampling_time", 0.05);
    handler->setParameter("number_of_foot_corners", 4);

    iDynTree::Vector3 footCorner;
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
    initialState.omega = std::sqrt(9.81 / initialState.dcmPosition[2]);

    // Initialize the planner
    TimeVaryingDCMPlanner planner;
    REQUIRE(planner.initialize(handler));

    REQUIRE(planner.setContactPhaseList(phaseList));
    planner.setInitialState(initialState);

    REQUIRE(planner.computeTrajectory());

    for(int i = 0; i < 150; i++)
    {
        REQUIRE(planner.advance());
    }
}
