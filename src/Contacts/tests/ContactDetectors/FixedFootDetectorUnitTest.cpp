/**
 * @file FixedFootDetectorUnitTest.cpp
 * @authors Giulio Romualdi
 * @copyright 2021 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the GNU Lesser General Public License v2.1 or any later version.
 */

#include <catch2/catch.hpp>

#include <BipedalLocomotion/Contacts/ContactPhaseList.h>

#include <BipedalLocomotion/ContactDetectors/FixedFootDetector.h>
#include <BipedalLocomotion/ParametersHandler/IParametersHandler.h>
#include <BipedalLocomotion/ParametersHandler/StdImplementation.h>

using namespace BipedalLocomotion::ParametersHandler;
using namespace BipedalLocomotion::Contacts;

struct FixedFootState
{
    EstimatedContact leftFoot;
    EstimatedContact rightFoot;
};

FixedFootState getFixedFootState(double t, const ContactListMap& listMap)
{
    // t            0   1   2   3   4   5   6   7   8   9   10  11  12  13  14  15  16  17
    // L            |+++|---|+++++++++++|---|+++++++++++|---|+++++++++++|---|+++++++++++|
    // R            |+++++++++++|---|+++++++++++|---|+++++++++++|---|+++++++++++|---|+++|
    // stance foot  |LLL|RRR|RRR|LLL|LLL|RRR|RRR|LLL|LLL|RRR|RRR|LLL|LLL|RRR|RRR|LLL|LLL|

    FixedFootState state;
    state.leftFoot.pose = listMap.find("left_foot")->second.getPresentContact(t)->pose;
    state.rightFoot.pose = listMap.find("right_foot")->second.getPresentContact(t)->pose;

    if (t <= 1)
    {
        state.leftFoot.isActive = true;
        state.rightFoot.isActive = false;
    } else if (t <= 3)
    {
        state.leftFoot.isActive = false;
        state.rightFoot.isActive = true;
    } else if (t <= 5)
    {
        state.leftFoot.isActive = true;
        state.rightFoot.isActive = false;
    } else if (t <= 7)
    {
        state.leftFoot.isActive = false;
        state.rightFoot.isActive = true;
    } else if (t <= 9)
    {
        state.leftFoot.isActive = true;
        state.rightFoot.isActive = false;
    } else if (t <= 11)
    {
        state.leftFoot.isActive = false;
        state.rightFoot.isActive = true;
    } else if (t <= 13)
    {
        state.leftFoot.isActive = true;
        state.rightFoot.isActive = false;
    } else if (t <= 15)
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

    Eigen::Vector3d leftPosition = Eigen::Vector3d::Zero();
    manif::SE3d leftTransform(leftPosition, manif::SO3d::Identity());
    contactListMap["left_foot"].addContact(leftTransform, 0.0, 1.0);

    leftPosition(0) += 0.05;
    leftTransform.translation(leftPosition);
    contactListMap["left_foot"].addContact(leftTransform, 2.0, 5.0);

    leftPosition(0) += 0.1;
    leftTransform.translation(leftPosition);
    contactListMap["left_foot"].addContact(leftTransform, 6.0, 9.0);

    leftPosition(0) += 0.1;
    leftTransform.translation(leftPosition);
    contactListMap["left_foot"].addContact(leftTransform, 10.0, 13.0);

    leftPosition(0) += 0.1;
    leftTransform.translation(leftPosition);
    contactListMap["left_foot"].addContact(leftTransform, 14.0, 17.0);

    // right foot
    // first footstep
    Eigen::Vector3d rightPosition = Eigen::Vector3d::Zero();
    manif::SE3d rightTransform(rightPosition, manif::SO3d::Identity());

    contactListMap["right_foot"].addContact(rightTransform, 0.0, 3.0);

    rightPosition(0) += 0.1;
    rightTransform.translation(rightPosition);
    contactListMap["right_foot"].addContact(rightTransform, 4.0, 7.0);

    rightPosition(0) += 0.1;
    rightTransform.translation(rightPosition);
    contactListMap["right_foot"].addContact(rightTransform, 8.0, 11.0);

    rightPosition(0) += 0.1;
    rightTransform.translation(rightPosition);
    contactListMap["right_foot"].addContact(rightTransform, 12.0, 15.0);

    rightPosition(0) += 0.05;
    rightTransform.translation(rightPosition);
    contactListMap["right_foot"].addContact(rightTransform, 16.0, 17.0);

    ContactPhaseList phaseList;
    phaseList.setLists(contactListMap);
    return phaseList;
}

TEST_CASE("Fixed Foot Detector")
{
    constexpr auto dT = 0.01;
    FixedFootDetector detector;
    auto handler = std::make_shared<StdImplementation>();
    handler->setParameter("sampling_time", dT);
    REQUIRE(detector.initialize(handler));

    const auto phaseList = createContactList();
    double currentTime = phaseList.firstPhase()->beginTime;
    detector.setContactPhaseList(phaseList);

    for (; currentTime < 20;)
    {
        auto state = getFixedFootState(currentTime, phaseList.lists());
        REQUIRE(detector.advance());
        REQUIRE(detector.getOutput().find("right_foot")->second.isActive == state.rightFoot.isActive);
        REQUIRE(detector.getOutput().find("left_foot")->second.isActive == state.leftFoot.isActive);

        if (state.leftFoot.isActive)
        {
            REQUIRE(detector.getOutput().find("left_foot")->second.pose == state.leftFoot.pose);
        } else if (state.rightFoot.isActive)
        {
            REQUIRE(detector.getOutput().find("right_foot")->second.pose == state.rightFoot.pose);
        }
        else
        {
            // Error this should never happen
            REQUIRE(false);
        }

        currentTime += dT;
    }
}
