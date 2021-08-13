/**
 * @file ContactPhaseListTest.cpp
 * @authors Stefano Dafarra, Giulio Romualdi
 * @copyright 2020,2021 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the GNU Lesser General Public License v2.1 or any later version.
 */

// Catch2
#include <catch2/catch.hpp>

#include <BipedalLocomotion/Contacts/ContactPhaseList.h>

using namespace BipedalLocomotion::Contacts;

TEST_CASE("Rule of five")
{
    ContactPhaseList phaseList;

    ContactListMap contactListMap;
    REQUIRE(contactListMap["left"].addContact(manif::SE3d::Identity(), 0.0, 1.0));
    REQUIRE(contactListMap["left"].addContact(manif::SE3d::Identity(), 2.0, 5.0));
    REQUIRE(contactListMap["left"].addContact(manif::SE3d::Identity(), 6.0, 7.0));

    REQUIRE(contactListMap["right"].addContact(manif::SE3d::Identity(), 0.0, 3.0));
    REQUIRE(contactListMap["right"].addContact(manif::SE3d::Identity(), 4.0, 7.0));

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
    ContactPhaseList phaseList;

    SECTION("Set from map")
    {
        ContactListMap contactListMap;
        REQUIRE(contactListMap["left"].addContact(manif::SE3d::Identity(), 0.0, 1.0));
        REQUIRE(contactListMap["left"].addContact(manif::SE3d::Identity(), 2.0, 5.0));
        REQUIRE(contactListMap["left"].addContact(manif::SE3d::Identity(), 6.0, 7.0));

        REQUIRE(contactListMap["right"].addContact(manif::SE3d::Identity(), 0.0, 3.0));
        REQUIRE(contactListMap["right"].addContact(manif::SE3d::Identity(), 4.0, 7.0));

        phaseList.setLists(contactListMap);
    }

    ContactList contactListLeft, contactListRight, contactListAdditional;
    contactListLeft.setDefaultName("left");
    contactListRight.setDefaultName("right");
    contactListAdditional.setDefaultName("additional");

    REQUIRE(contactListLeft.addContact(manif::SE3d::Identity(), 0.0, 1.0));
    REQUIRE(contactListLeft.addContact(manif::SE3d::Identity(), 2.0, 5.0));
    REQUIRE(contactListLeft.addContact(manif::SE3d::Identity(), 6.0, 7.0));

    REQUIRE(contactListRight.addContact(manif::SE3d::Identity(), 0.0, 3.0));
    REQUIRE(contactListRight.addContact(manif::SE3d::Identity(), 4.0, 7.0));

    REQUIRE(contactListAdditional.addContact(manif::SE3d::Identity(), 4.0, 5.0));
    REQUIRE(contactListAdditional.addContact(manif::SE3d::Identity(), 6.0, 7.5));

    REQUIRE(phaseList.setLists({contactListAdditional, contactListLeft, contactListRight}));

    SECTION("Present phase")
    {
        auto it = phaseList.begin();
        std::advance(it, 1);
        bool same = phaseList.getPresentPhase(it->beginTime) == it;
        REQUIRE(same);

        std::advance(it, 1);
        same = phaseList.getPresentPhase((it->beginTime + it->endTime) / 2.0) == it;
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
        REQUIRE(phase->beginTime == 0.0);
        REQUIRE(phase->endTime == 1.0);
        REQUIRE(phase->activeContacts.size() == 2);
        bool ok = phase->activeContacts.at("left") == expectedLeft;
        REQUIRE(ok);
        ok = phase->activeContacts.at("right") == expectedRight;
        REQUIRE(ok);

        phase++;
        expectedLeft++;

        REQUIRE(phase->beginTime == 1.0);
        REQUIRE(phase->endTime == 2.0);
        REQUIRE(phase->activeContacts.size() == 1);
        ok = phase->activeContacts.at("right") == expectedRight;
        REQUIRE(ok);

        phase++;

        REQUIRE(phase->beginTime == 2.0);
        REQUIRE(phase->endTime == 3.0);
        REQUIRE(phase->activeContacts.size() == 2);
        ok = phase->activeContacts.at("left") == expectedLeft;
        REQUIRE(ok);
        ok = phase->activeContacts.at("right") == expectedRight;
        REQUIRE(ok);

        phase++;
        expectedRight++;

        REQUIRE(phase->beginTime == 3.0);
        REQUIRE(phase->endTime == 4.0);
        REQUIRE(phase->activeContacts.size() == 1);
        ok = phase->activeContacts.at("left") == expectedLeft;
        REQUIRE(ok);

        phase++;

        REQUIRE(phase->beginTime == 4.0);
        REQUIRE(phase->endTime == 5.0);
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

        REQUIRE(phase->beginTime == 5.0);
        REQUIRE(phase->endTime == 6.0);
        REQUIRE(phase->activeContacts.size() == 1);
        ok = phase->activeContacts.at("right") == expectedRight;
        REQUIRE(ok);

        phase++;

        REQUIRE(phase->beginTime == 6.0);
        REQUIRE(phase->endTime == 7.0);
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

        REQUIRE(phase->beginTime == 7.0);
        REQUIRE(phase->endTime == 7.5);
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
