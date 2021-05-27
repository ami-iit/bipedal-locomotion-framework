/**
 * @file ContactListTest.cpp
 * @authors Stefano Dafarra, Giulio Romualdi
 * @copyright 2020 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the GNU Lesser General Public License v2.1 or any later version.
 */

// Catch2
#include <catch2/catch.hpp>

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
    ContactList list;
    PlannedContact p1, p2;
    p1.activationTime = 0.1;
    p1.deactivationTime = 0.5;

    p2.activationTime = 1.0;
    p2.deactivationTime = 1.5;

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
        p3.activationTime = 0.6;
        p3.deactivationTime = 0.8;

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
        p3.activationTime = 0.9;
        p3.deactivationTime = 1.6;

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
        REQUIRE(contactsAreEqual(p2, *list.getPresentContact(1.2)));
        REQUIRE(contactsAreEqual(p2, *list.getPresentContact(1.6)));
        REQUIRE(contactsAreEqual(p1, *list.getPresentContact(0.6)));
        bool same = list.getPresentContact(0.0) == list.end();
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
            ok = ok && list.addContact(manif::SE3d::Identity(), 2.0 + i, 2.5 + i);
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
    ContactListMap contactListMap;
    contactListMap["left"].setDefaultName("left_foot");
    REQUIRE(contactListMap["left"].addContact(manif::SE3d::Random(), 0.0, 1.0));
    REQUIRE(contactListMap["left"].addContact(manif::SE3d::Random(), 2.0, 5.0));

    contactListMap["right"].setDefaultName("right_foot");
    REQUIRE(contactListMap["right"].addContact(manif::SE3d::Random(), 0.0, 3.0));

    REQUIRE(contactListMapToJson(contactListMap, "contact_list.json"));

    ContactListMap newContactListMap = contactListMapFromJson("contact_list.json");

    REQUIRE(newContactListMap.size() == contactListMap.size());

    for (const auto& [key, contacts] : newContactListMap)
    {
        for (int i = 0; i < contacts.size(); i++)
            REQUIRE(contactsAreEqual(contactListMap[key][i], contacts[i]));
    }
}
