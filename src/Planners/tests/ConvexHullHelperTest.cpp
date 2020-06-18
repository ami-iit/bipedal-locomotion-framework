/**
 * @file ConvexHullHelperTest.cpp
 * @authors Giulio Romualdi
 * @copyright 2020 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the GNU Lesser General Public License v2.1 or any later version.
 */

// Catch2
#include <catch2/catch.hpp>

#include <BipedalLocomotion/Planners/ConvexHullHelper.h>

using namespace BipedalLocomotion::Planners;

TEST_CASE("Convex Hull helper")
{
    ConvexHullHelper helper;

    // initialize the points
    std::vector<iDynTree::VectorDynSize> points;
    iDynTree::VectorDynSize p(3);

    p(0) = 0.6269;
    p(1) = 0.7207;
    p(2) = 0.3000;
    points.push_back(p);

    p(0) = 0.5538;
    p(1) = 0.6526;
    points.push_back(p);

    p(0) = 0.6901;
    p(1) = 0.5062;
    points.push_back(p);

    p(0) = 0.7633;
    p(1) = 0.5744;
    points.push_back(p);

    p(0) = 0.8927;
    p(1) = 0.7319;
    p(2) = 0.2400;
    points.push_back(p);

    p(0) = 0.8101;
    p(1) = 0.6754;
    points.push_back(p);

    p(0) = 0.9231;
    p(1) = 0.5103;
    points.push_back(p);

    p(0) = 1.0056;
    p(1) = 0.5668;
    points.push_back(p);

    REQUIRE(helper.buildConvexHull(points));

    // check if the points belong to convex hull
    for(const auto& point : points)
        REQUIRE(helper.doesPointBelongToCovexHull(point));

    // p = [0 0 0] does not belong to the convex hull
    iDynTree::VectorDynSize pointOutsideConvexHull(3);
    pointOutsideConvexHull.zero();
    REQUIRE_FALSE(helper.doesPointBelongToCovexHull(pointOutsideConvexHull));
}
