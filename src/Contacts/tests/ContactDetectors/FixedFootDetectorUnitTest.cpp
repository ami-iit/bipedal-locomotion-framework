/**
 * @file FixedFootDetectorUnitTest.cpp
 * @authors Giulio Romualdi
 * @copyright 2021-2023 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the BSD-3-Clause license.
 */

#include <catch2/catch_test_macros.hpp>

#include <BipedalLocomotion/Contacts/ContactPhaseList.h>

#include <BipedalLocomotion/ContactDetectors/FixedFootDetector.h>
#include <BipedalLocomotion/ParametersHandler/IParametersHandler.h>
#include <BipedalLocomotion/ParametersHandler/StdImplementation.h>
#include <chrono>

using namespace BipedalLocomotion::ParametersHandler;
using namespace BipedalLocomotion::Contacts;

struct FixedFootState
{
    EstimatedContact leftFoot;
    EstimatedContact rightFoot;
};

FixedFootState getFixedFootState(const std::chrono::nanoseconds& t, const ContactListMap& listMap)
{
    // t            0   1   2   3   4   5   6   7   8   9   10  11  12  13  14  15  16  17
    // L            |+++|---|+++++++++++|---|+++++++++++|---|+++++++++++|---|+++++++++++|
    // R            |+++++++++++|---|+++++++++++|---|+++++++++++|---|+++++++++++|---|+++|
    // stance foot  |LLL|RRR|RRR|LLL|LLL|RRR|RRR|LLL|LLL|RRR|RRR|LLL|LLL|RRR|RRR|LLL|LLL|

    using namespace std::chrono_literals;

    FixedFootState state;
    state.leftFoot.pose = listMap.find("left_foot")->second.getPresentContact(t)->pose;
    state.rightFoot.pose = listMap.find("right_foot")->second.getPresentContact(t)->pose;

    if (t < 1s)
    {
        state.leftFoot.isActive = true;
        state.rightFoot.isActive = false;
    } else if (t < 3s)
    {
        state.leftFoot.isActive = false;
        state.rightFoot.isActive = true;
    } else if (t < 5s)
    {
        state.leftFoot.isActive = true;
        state.rightFoot.isActive = false;
    } else if (t < 7s)
    {
        state.leftFoot.isActive = false;
        state.rightFoot.isActive = true;
    } else if (t < 9s)
    {
        state.leftFoot.isActive = true;
        state.rightFoot.isActive = false;
    } else if (t < 11s)
    {
        state.leftFoot.isActive = false;
        state.rightFoot.isActive = true;
    } else if (t < 13s)
    {
        state.leftFoot.isActive = true;
        state.rightFoot.isActive = false;
    } else if (t < 15s)
    {
        state.leftFoot.isActive = false;
        state.rightFoot.isActive = true;
    } else
    {
        state.leftFoot.isActive = true;
        state.rightFoot.isActive = false;
    }

    return state;
}

ContactPhaseList createContactList()
{
    // t            0   1   2   3   4   5   6   7   8   9   10  11  12  13  14  15  16  17
    // L            |+++|---|+++++++++++|---|+++++++++++|---|+++++++++++|---|+++++++++++|
    // R            |+++++++++++|---|+++++++++++|---|+++++++++++|---|+++++++++++|---|+++|
    // stance foot  |LLL|RRR|RRR|LLL|LLL|RRR|RRR|LLL|LLL|RRR|RRR|LLL|LLL|RRR|RRR|LLL|LLL|

    ContactListMap contactListMap;
    using namespace std::chrono_literals;

    Eigen::Vector3d leftPosition = Eigen::Vector3d::Zero();
    manif::SE3d leftTransform(leftPosition, manif::SO3d::Identity());
    contactListMap["left_foot"].addContact(leftTransform, 0s, 1s);

    leftPosition(0) += 0.05;
    leftTransform.translation(leftPosition);
    contactListMap["left_foot"].addContact(leftTransform, 2s, 5s);

    leftPosition(0) += 0.1;
    leftTransform.translation(leftPosition);
    contactListMap["left_foot"].addContact(leftTransform, 6s, 9s);

    leftPosition(0) += 0.1;
    leftTransform.translation(leftPosition);
    contactListMap["left_foot"].addContact(leftTransform, 10s, 13s);

    leftPosition(0) += 0.1;
    leftTransform.translation(leftPosition);
    contactListMap["left_foot"].addContact(leftTransform, 14s, 17s);

    // right foot
    // first footstep
    Eigen::Vector3d rightPosition = Eigen::Vector3d::Zero();
    manif::SE3d rightTransform(rightPosition, manif::SO3d::Identity());

    contactListMap["right_foot"].addContact(rightTransform, 0s, 3s);

    rightPosition(0) += 0.1;
    rightTransform.translation(rightPosition);
    contactListMap["right_foot"].addContact(rightTransform, 4s, 7s);

    rightPosition(0) += 0.1;
    rightTransform.translation(rightPosition);
    contactListMap["right_foot"].addContact(rightTransform, 8s, 11s);

    rightPosition(0) += 0.1;
    rightTransform.translation(rightPosition);
    contactListMap["right_foot"].addContact(rightTransform, 12s, 15s);

    rightPosition(0) += 0.05;
    rightTransform.translation(rightPosition);
    contactListMap["right_foot"].addContact(rightTransform, 16s, 17s);

    ContactPhaseList phaseList;
    phaseList.setLists(contactListMap);
    return phaseList;
}

TEST_CASE("Fixed Foot Detector")
{
    using namespace std::chrono_literals;
    constexpr std::chrono::nanoseconds dT = 10ms;
    constexpr std::chrono::nanoseconds horizon = 20s;
    FixedFootDetector detector;
    auto handler = std::make_shared<StdImplementation>();
    handler->setParameter("sampling_time", dT);
    REQUIRE(detector.initialize(handler));

    const auto phaseList = createContactList();
    detector.setContactPhaseList(phaseList);

    for (std::chrono::nanoseconds currentTime = 0s; currentTime < horizon; currentTime += dT)
    {
        // advance is used to advance the time stored in the detector and to evaluate the outputs
        REQUIRE(detector.advance());

        auto state = getFixedFootState(currentTime, phaseList.lists());

        REQUIRE(detector.getOutput().find("right_foot")->second.isActive == state.rightFoot.isActive);
        REQUIRE(detector.getOutput().find("left_foot")->second.isActive == state.leftFoot.isActive);

        if (state.leftFoot.isActive)
        {
            REQUIRE(detector.getOutput().find("left_foot")->second.pose == state.leftFoot.pose);
        } else if (state.rightFoot.isActive)
        {
            REQUIRE(detector.getOutput().find("right_foot")->second.pose == state.rightFoot.pose);
        } else
        {
            // Error this should never happen
            REQUIRE(false);
        }
    }
}
