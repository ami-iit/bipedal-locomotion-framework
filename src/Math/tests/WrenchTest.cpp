/**
 * @file WrenchTest.cpp
 * @authors Giulio Romualdi
 * @copyright 2021 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the BSD-3-Clause license.
 */

// Catch2
#include <catch2/catch_test_macros.hpp>

#include <BipedalLocomotion/Math/Wrench.h>

#include <manif/SE3.h>
#include <manif/SO3.h>

using namespace BipedalLocomotion::Math;

TEST_CASE("Wrench test")
{
    Wrenchd wrench = Wrenchd::Random();

    SECTION("Get force and torque")
    {
        REQUIRE(wrench.head<3>().cwiseEqual(wrench.force()).all());
        REQUIRE(wrench.tail<3>().cwiseEqual(wrench.torque()).all());
    }

    SECTION("set force and torque")
    {
        Wrenchd newWrench;
        newWrench.force() = wrench.force();
        newWrench.torque() = wrench.torque();

        REQUIRE(newWrench.force().cwiseEqual(wrench.force()).all());
        REQUIRE(newWrench.torque().cwiseEqual(wrench.torque()).all());
    }

    SECTION("Apply transform")
    {
        const manif::SE3d transform = manif::SE3d::Random();
        const Wrench transformedWrench = transform * wrench;

        // Get the adjoint matrix for spatial force vector
        // this is a conversion from AdjointTransform to AdjointWrenchTransform
        manif::SE3d::Jacobian adjointMatrix = transform.adj();
        adjointMatrix.bottomLeftCorner<3, 3>() = adjointMatrix.topRightCorner<3, 3>();
        adjointMatrix.topRightCorner<3, 3>().setZero();
        REQUIRE(transformedWrench.isApprox(adjointMatrix * wrench));
    }

    SECTION("Apply rotation")
    {
        const manif::SO3d rotation = manif::SO3d::Random();
        const Wrench rotatedWrench = rotation * wrench;

        REQUIRE(rotatedWrench.force().isApprox(rotation.rotation() * wrench.force()));
        REQUIRE(rotatedWrench.torque().isApprox(rotation.rotation() * wrench.torque()));
    }

    SECTION("Local CoP")
    {
        // ensure that the force is positive
        wrench.force()[2] = 10;

        Eigen::Vector3d expectedCoP;
        expectedCoP[0] = -wrench.torque()[1] / wrench.force()[2];
        expectedCoP[1] = wrench.torque()[0] / wrench.force()[2];
        expectedCoP[2] = 0;

        REQUIRE(wrench.getLocalCoP().isApprox(expectedCoP));
    }
}
