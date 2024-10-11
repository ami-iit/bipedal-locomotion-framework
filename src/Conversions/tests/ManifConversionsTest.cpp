/**
 * @file ManifConversionsTest.cpp
 * @authors Prashanth Ramadoss
 * @copyright 2020 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the BSD-3-Clause license.
 */

// Catch2
#include <catch2/catch_test_macros.hpp>

#include <BipedalLocomotion/Conversions/CommonConversions.h>
#include <BipedalLocomotion/Conversions/ManifConversions.h>
#include <iDynTree/EigenHelpers.h>

TEST_CASE("Manif Conversions")
{
    Eigen::Vector3d pos;
    pos << 0.0296, -0.1439,  0.4915;
    Eigen::Quaterniond quat = Eigen::Quaterniond(0.3218, -0.6304, -0.6292, 0.3212);
    quat.normalize();

    Eigen::Matrix3d R = quat.toRotationMatrix();
    Eigen::Matrix4d H = BipedalLocomotion::Conversions::toEigenPose(R, pos);

    iDynTree::Transform iDynH;
    iDynTree::fromEigen(iDynH, H);

    manif::SE3d pose = BipedalLocomotion::Conversions::toManifPose(iDynH);

    constexpr double tolerance = 1e-4;
    REQUIRE(pose.transform().isApprox(H, tolerance));

    manif::SE3d pose2 = BipedalLocomotion::Conversions::toManifPose(R, pos);
    REQUIRE(pose2.transform().isApprox(iDynTree::toEigen(iDynH.asHomogeneousTransform())));

    manif::SO3d rot = quat;
    iDynTree::Rotation idynRot = BipedalLocomotion::Conversions::toiDynTreeRot(rot);
    REQUIRE(iDynTree::toEigen(idynRot).isApprox(R, tolerance));
}
