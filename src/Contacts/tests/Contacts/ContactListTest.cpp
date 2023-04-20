/**
 * @file ContactListTest.cpp
 * @authors Stefano Dafarra, Giulio Romualdi
 * @copyright 2020 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the BSD-3-Clause license.
 */

#include <chrono>

// Catch2
#include <catch2/catch_test_macros.hpp>

// manif
#include <manif/manif.h>

#include <BipedalLocomotion/Contacts/ContactList.h>
#include <BipedalLocomotion/Contacts/ContactListJsonParser.h>
#include <BipedalLocomotion/Contacts/Contact.h>

using namespace BipedalLocomotion::Contacts;

bool contactsAreEqual(const PlannedContact& c1, const PlannedContact& c2)
{
    constexpr double tolerance = 1e-5;
    return (c1.type == c2.type) && (c1.name == c2.name)
           && (c1.pose.coeffs().isApprox(c2.pose.coeffs(), tolerance))
           && (c1.activationTime == c2.activationTime)
           && (c1.deactivationTime == c2.deactivationTime);
}

TEST_CASE("ContactList")
{
    using namespace std::chrono_literals;

    ContactList list;
    PlannedContact p1, p2;
    p1.activationTime = 100ms;
    p1.deactivationTime = 500ms;

    p2.activationTime = 1s;
    p2.deactivationTime = 1s + 500ms;

    bool ok1 = list.addContact(p2);
    bool ok2 = list.addContact(p1);

    SECTION("Insertion")
    {
        REQUIRE(ok1);
        REQUIRE(ok2);
    }

    SECTION("Insertion order")
    {
        REQUIRE(contactsAreEqual(p1, *list.firstContact()));
        REQUIRE(contactsAreEqual(p2, *list.lastContact()));

        PlannedContact p3;
        p3.activationTime = 600ms;
        p3.deactivationTime = 800ms;

        REQUIRE(list.addContact(p3));
        REQUIRE(list.size() == 3);
        REQUIRE(contactsAreEqual(p3, *(++list.begin())));
    }

    SECTION("Size")
    {
        REQUIRE(list.size() == 2);
    }

    SECTION("Invalid insertion")
    {
        PlannedContact p3;
        p3.activationTime = 900ms;
        p3.deactivationTime = 1s + 600ms;

        REQUIRE_FALSE(list.addContact(p3));
    }

    SECTION("Edit")
    {
        PlannedContact p2Modified;
        p2Modified = p2;
        p2Modified.type = ContactType::POINT;

        REQUIRE(list.editContact(list.lastContact(), p2Modified));
        REQUIRE(contactsAreEqual(p2Modified, *list.lastContact()));
    }

    SECTION("Present step")
    {
        REQUIRE(contactsAreEqual(p2, *list.getPresentContact(1s + 200ms)));
        REQUIRE(contactsAreEqual(p2, *list.getPresentContact(1s + 600ms)));
        REQUIRE(contactsAreEqual(p1, *list.getPresentContact(600ms)));
        bool same = list.getPresentContact(0s) == list.end();
        REQUIRE(same);
    }

    SECTION("Keep present step")
    {
        list.clear();
        REQUIRE(list.size() == 0);
    }

    SECTION("Accessor")
    {
        bool ok = true;
        for (size_t i = 0; i < 49; ++i)
        {
            ok = ok && list.addContact(manif::SE3d::Identity(), 2s + i * 1s, 2s + 500ms + i * 1s);
        }
        REQUIRE(ok);
        REQUIRE(list.size() == 51);

        ContactList::const_iterator it = list.begin();
        for (size_t i = 0; i < list.size(); ++i)
        {
            ok = ok && contactsAreEqual(list[i], *it);
            it++;
        }
        REQUIRE(ok);
    }
}

TEST_CASE("ContactList JSON parser")
{
    using namespace std::chrono_literals;

    ContactListMap contactListMap;
    contactListMap["left"].setDefaultName("left_foot");
    REQUIRE(contactListMap["left"].addContact(manif::SE3d::Random(), 0s, 1s));
    REQUIRE(contactListMap["left"].addContact(manif::SE3d::Random(), 2s, 5s));

    contactListMap["right"].setDefaultName("right_foot");
    REQUIRE(contactListMap["right"].addContact(manif::SE3d::Random(), 0s, 3s));

    REQUIRE(contactListMapToJson(contactListMap, "contact_list.json"));

    ContactListMap newContactListMap = contactListMapFromJson("contact_list.json");

    REQUIRE(newContactListMap.size() == contactListMap.size());

    for (const auto& [key, contacts] : newContactListMap)
    {
        for (int i = 0; i < contacts.size(); i++)
        {
            REQUIRE(contactsAreEqual(contactListMap[key][i], contacts[i]));
        }
    }
}
