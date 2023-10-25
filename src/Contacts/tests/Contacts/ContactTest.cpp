/**
 * @file Contact.cpp
 * @authors Giulio Romualdi
 * @copyright 2023 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the BSD-3-Clause license.
 */

#include <chrono>

// Catch2
#include <catch2/catch_test_macros.hpp>

// manif
#include <manif/manif.h>

#include <BipedalLocomotion/Contacts/Contact.h>

using namespace BipedalLocomotion::Contacts;

TEST_CASE("ContactWrench")
{
    ContactWrench contact;
    contact.pose = manif::SE3d::Random();
    contact.wrench.force() = Eigen::Vector3d::Random();
    contact.wrench.force()[2] = 10;
    contact.wrench.torque() = Eigen::Vector3d::Random();

    Eigen::Vector3d expectedZMP;
    expectedZMP[0] = -contact.wrench.torque()[1] / contact.wrench.force()[2];
    expectedZMP[1] = contact.wrench.torque()[0] / contact.wrench.force()[2];
    expectedZMP[2] = 0;

    REQUIRE(contact.getLocalZMP().isApprox(expectedZMP));

    expectedZMP = contact.pose.act(expectedZMP);
    REQUIRE(contact.getGlobalZMP().isApprox(expectedZMP));
}
