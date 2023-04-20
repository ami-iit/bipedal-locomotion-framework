/**
 * @file ContactPhaseListTest.cpp
 * @authors Stefano Dafarra, Giulio Romualdi
 * @copyright 2020,2021 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the BSD-3-Clause license.
 */

#include <chrono>

// Catch2
#include <catch2/catch_test_macros.hpp>

#include <BipedalLocomotion/Contacts/ContactPhaseList.h>

using namespace BipedalLocomotion::Contacts;

TEST_CASE("Rule of five")
{
    using namespace std::chrono_literals;

    ContactPhaseList phaseList;

    ContactListMap contactListMap;
    REQUIRE(contactListMap["left"].addContact(manif::SE3d::Identity(), 0s, 1s));
    REQUIRE(contactListMap["left"].addContact(manif::SE3d::Identity(), 2s, 5s));
    REQUIRE(contactListMap["left"].addContact(manif::SE3d::Identity(), 6s, 7s));

    REQUIRE(contactListMap["right"].addContact(manif::SE3d::Identity(), 0s, 3s));
    REQUIRE(contactListMap["right"].addContact(manif::SE3d::Identity(), 4s, 7s));

    phaseList.setLists(contactListMap);

    SECTION("Copy constructor")
    {
        ContactPhaseList newList(phaseList);
        phaseList.clear();
        const ContactListMap& contactListMap = newList.lists();
        ContactList::const_iterator expectedLeft = contactListMap.at("left").begin();
        REQUIRE(newList.firstPhase()->activeContacts.at("left")->pose == manif::SE3d::Identity());
    }

    SECTION("Copy assignment operator")
    {
        ContactPhaseList newList;
        newList = phaseList;

        phaseList.clear();
        const ContactListMap& contactListMap = newList.lists();
        ContactList::const_iterator expectedLeft = contactListMap.at("left").begin();
        REQUIRE(newList.firstPhase()->activeContacts.at("left")->pose == manif::SE3d::Identity());
    }

    SECTION("Move constructor")
    {
        ContactPhaseList newList(std::move(phaseList));

        const ContactListMap& contactListMap = newList.lists();
        ContactList::const_iterator expectedLeft = contactListMap.at("left").begin();
        REQUIRE(newList.firstPhase()->activeContacts.at("left")->pose == manif::SE3d::Identity());
    }

    SECTION("Move assignment operator")
    {
        ContactPhaseList newList;
        newList = std::move(phaseList);

        const ContactListMap& contactListMap = newList.lists();
        ContactList::const_iterator expectedLeft = contactListMap.at("left").begin();
        REQUIRE(newList.firstPhase()->activeContacts.at("left")->pose == manif::SE3d::Identity());
    }
}

TEST_CASE("ContactPhaseList")
{
    using namespace std::chrono_literals;

    ContactPhaseList phaseList;

    SECTION("Set from map")
    {
        ContactListMap contactListMap;
        REQUIRE(contactListMap["left"].addContact(manif::SE3d::Identity(), 0s, 1s));
        REQUIRE(contactListMap["left"].addContact(manif::SE3d::Identity(), 2s, 5s));
        REQUIRE(contactListMap["left"].addContact(manif::SE3d::Identity(), 6s, 7s));

        REQUIRE(contactListMap["right"].addContact(manif::SE3d::Identity(), 0s, 3s));
        REQUIRE(contactListMap["right"].addContact(manif::SE3d::Identity(), 4s, 7s));

        phaseList.setLists(contactListMap);
    }

    ContactList contactListLeft, contactListRight, contactListAdditional;
    contactListLeft.setDefaultName("left");
    contactListRight.setDefaultName("right");
    contactListAdditional.setDefaultName("additional");

    REQUIRE(contactListLeft.addContact(manif::SE3d::Identity(), 0s, 1s));
    REQUIRE(contactListLeft.addContact(manif::SE3d::Identity(), 2s, 5s));
    REQUIRE(contactListLeft.addContact(manif::SE3d::Identity(), 6s, 7s));

    REQUIRE(contactListRight.addContact(manif::SE3d::Identity(), 0s, 3s));
    REQUIRE(contactListRight.addContact(manif::SE3d::Identity(), 4s, 7s));

    REQUIRE(contactListAdditional.addContact(manif::SE3d::Identity(), 4s, 5s));
    REQUIRE(contactListAdditional.addContact(manif::SE3d::Identity(), 6s, 7s + 500ms));

    REQUIRE(phaseList.setLists({contactListAdditional, contactListLeft, contactListRight}));

    SECTION("Present phase")
    {
        auto it = phaseList.begin();
        std::advance(it, 1);
        bool same = phaseList.getPresentPhase(it->beginTime) == it;
        REQUIRE(same);

        std::advance(it, 1);
        same = phaseList.getPresentPhase(((it->beginTime + it->endTime).count() / 2)*1ns) == it;
        REQUIRE(same);

        std::advance(it, 1);

        // the interval of a phase is defined as t = [t_begin, t_end) (i.e. t_end is not included)
        same = phaseList.getPresentPhase(it->endTime) == std::next(it, 1);
        REQUIRE(same);
    }


    SECTION("Check phases")
    {
        REQUIRE(phaseList.size() == 8);

        const ContactListMap& contactListMap = phaseList.lists();
        ContactList::const_iterator expectedLeft = contactListMap.at("left").begin();
        ContactList::const_iterator expectedRight = contactListMap.at("right").begin();
        ContactList::const_iterator expectedAdditional = contactListMap.at("additional").begin();

        ContactPhaseList::const_iterator phase = phaseList.begin();
        REQUIRE(phase->beginTime == 0s);
        REQUIRE(phase->endTime == 1s);
        REQUIRE(phase->activeContacts.size() == 2);
        bool ok = phase->activeContacts.at("left") == expectedLeft;
        REQUIRE(ok);
        ok = phase->activeContacts.at("right") == expectedRight;
        REQUIRE(ok);

        phase++;
        expectedLeft++;

        REQUIRE(phase->beginTime == 1s);
        REQUIRE(phase->endTime == 2s);
        REQUIRE(phase->activeContacts.size() == 1);
        ok = phase->activeContacts.at("right") == expectedRight;
        REQUIRE(ok);

        phase++;

        REQUIRE(phase->beginTime == 2s);
        REQUIRE(phase->endTime == 3s);
        REQUIRE(phase->activeContacts.size() == 2);
        ok = phase->activeContacts.at("left") == expectedLeft;
        REQUIRE(ok);
        ok = phase->activeContacts.at("right") == expectedRight;
        REQUIRE(ok);

        phase++;
        expectedRight++;

        REQUIRE(phase->beginTime == 3s);
        REQUIRE(phase->endTime == 4s);
        REQUIRE(phase->activeContacts.size() == 1);
        ok = phase->activeContacts.at("left") == expectedLeft;
        REQUIRE(ok);

        phase++;

        REQUIRE(phase->beginTime == 4s);
        REQUIRE(phase->endTime == 5s);
        REQUIRE(phase->activeContacts.size() == 3);
        ok = phase->activeContacts.at("left") == expectedLeft;
        REQUIRE(ok);
        ok = phase->activeContacts.at("right") == expectedRight;
        REQUIRE(ok);
        ok = phase->activeContacts.at("additional") == expectedAdditional;
        REQUIRE(ok);

        phase++;
        expectedLeft++;
        expectedAdditional++;

        REQUIRE(phase->beginTime == 5s);
        REQUIRE(phase->endTime == 6s);
        REQUIRE(phase->activeContacts.size() == 1);
        ok = phase->activeContacts.at("right") == expectedRight;
        REQUIRE(ok);

        phase++;

        REQUIRE(phase->beginTime == 6s);
        REQUIRE(phase->endTime == 7s);
        REQUIRE(phase->activeContacts.size() == 3);
        ok = phase->activeContacts.at("left") == expectedLeft;
        REQUIRE(ok);
        ok = phase->activeContacts.at("right") == expectedRight;
        REQUIRE(ok);
        ok = phase->activeContacts.at("additional") == expectedAdditional;
        REQUIRE(ok);

        phase++;
        expectedLeft++;
        expectedRight++;

        REQUIRE(phase->beginTime == 7s);
        REQUIRE(phase->endTime == 7s + 500ms);
        REQUIRE(phase->activeContacts.size() == 1);
        ok = phase->activeContacts.at("additional") == expectedAdditional;
        REQUIRE(ok);

        phase++;
        expectedAdditional++;

        ok = phase == phaseList.end();
        REQUIRE(ok);
        ok = expectedLeft == contactListMap.at("left").end();
        REQUIRE(ok);
        ok = expectedRight == contactListMap.at("right").end();
        REQUIRE(ok);
        ok = expectedAdditional == contactListMap.at("additional").end();
        REQUIRE(ok);
    }
}
