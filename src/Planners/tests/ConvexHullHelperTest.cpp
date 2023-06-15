/**
 * @file ConvexHullHelperTest.cpp
 * @authors Giulio Romualdi
 * @copyright 2020 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the BSD-3-Clause license.
 */

// Catch2
#include <catch2/catch_test_macros.hpp>

#include <BipedalLocomotion/Planners/ConvexHullHelper.h>

using namespace BipedalLocomotion::Planners;

TEST_CASE("Convex Hull helper")
{
    ConvexHullHelper helper;

    Eigen::MatrixXd p(3, 8);

    p(0,0) = 0.6269;
    p(1,0) = 0.7207;
    p(2,0) = 0.3000;

    p(0,1) = 0.5538;
    p(1,1) = 0.6526;
    p(2,1) = 0.3000;

    p(0,2) = 0.6901;
    p(1,2) = 0.5062;
    p(2,2) = 0.3000;

    p(0,3) = 0.7633;
    p(1,3) = 0.5744;
    p(2,3) = 0.3000;

    p(0,4) = 0.8927;
    p(1,4) = 0.7319;
    p(2,4) = 0.2400;

    p(0,5) = 0.8101;
    p(1,5) = 0.6754;
    p(2,5) = 0.2400;

    p(0,6) = 0.9231;
    p(1,6) = 0.5103;
    p(2,6) = 0.2400;

    p(0,7) = 1.0056;
    p(1,7) = 0.5668;
    p(2,7) = 0.2400;

    REQUIRE(helper.buildConvexHull(p));

    // check if the points belong to convex hull
    for(size_t col = 0; col < p.cols(); col++)
        REQUIRE(helper.doesPointBelongToConvexHull(p.col(col)));

    // p = [0 0 0] does not belong to the convex hull
    Eigen::VectorXd pointOutsideConvexHull(3);
    pointOutsideConvexHull.setZero();
    REQUIRE_FALSE(helper.doesPointBelongToConvexHull(pointOutsideConvexHull));
}
