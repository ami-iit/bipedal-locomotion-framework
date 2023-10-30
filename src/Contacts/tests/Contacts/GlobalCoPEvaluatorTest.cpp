/**
 * @file GlobalCoPEvaluatorTest.cpp
 * @authors Giulio Romualdi
 * @copyright 2023 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the BSD-3-Clause license.
 */

#include <chrono>

// Catch2
#include <catch2/catch_test_macros.hpp>

#include <BipedalLocomotion/Contacts/GlobalCoPEvaluator.h>
#include <BipedalLocomotion/ParametersHandler/StdImplementation.h>

using namespace BipedalLocomotion::Contacts;
using namespace BipedalLocomotion::ParametersHandler;

TEST_CASE("GlobalCoPEvaluator")
{
    auto handler = std::make_shared<StdImplementation>();
    handler->setParameter("minimum_normal_force", 1.0);
    handler->setParameter("cop_admissible_limits", std::vector<double>{0.1, 0.1});
    handler->setParameter("constant_cop_tolerance", 0.01);
    handler->setParameter("constant_cop_max_counter", 10);

    GlobalCoPEvaluator evaluator;
    REQUIRE(evaluator.initialize(handler));
    REQUIRE_FALSE(evaluator.isOutputValid());

    SECTION("Zero contact")
    {
        // this should fail
        REQUIRE_FALSE(evaluator.advance());
        REQUIRE_FALSE(evaluator.isOutputValid());
    }

    SECTION("One contact but invalid")
    {
        ContactWrench contact;
        contact.pose.setIdentity();
        contact.wrench.setZero();

        // this is lower than the threshold
        contact.wrench.force()(2) = 0.01;

        REQUIRE(evaluator.setInput({contact}));

        // this should fail
        REQUIRE_FALSE(evaluator.advance());
        REQUIRE_FALSE(evaluator.isOutputValid());
    }

    SECTION("One valid contact")
    {
        Eigen::Vector3d translation;
        translation[0] = 2.1;
        translation[1] = 1.1;
        translation[2] = 0.9;

        ContactWrench contact;
        contact.pose.setIdentity();
        contact.pose.translation(translation);
        contact.wrench.setZero();

        // this is lower than the threshold
        contact.wrench.force()(2) = 10.0;

        REQUIRE(evaluator.setInput({contact}));

        // this should fail
        REQUIRE(evaluator.advance());
        REQUIRE(evaluator.isOutputValid());

        REQUIRE(translation.isApprox(evaluator.getOutput()));
    }

    SECTION("Two valid contacts")
    {
        Eigen::Vector3d translation1;
        translation1[0] = 2.1;
        translation1[1] = 1.1;
        translation1[2] = 0.9;

        ContactWrench contact1;
        contact1.pose.setIdentity();
        contact1.pose.translation(translation1);
        contact1.wrench.setZero();

        // this is lower than the threshold
        contact1.wrench.force()(2) = 10.0;

        Eigen::Vector3d translation2;
        translation2[0] = 2.1;
        translation2[1] = -1.1;
        translation2[2] = 0.9;

        ContactWrench contact2;
        contact2.pose.setIdentity();
        contact2.pose.translation(translation2);
        contact2.wrench.setZero();

        // this is lower than the threshold
        contact2.wrench.force()(2) = 10.0;

        REQUIRE(evaluator.setInput({contact1, contact2}));

        // this should fail
        REQUIRE(evaluator.advance());
        REQUIRE(evaluator.isOutputValid());

        // compute the average translation
        Eigen::Vector3d averageTranslation = (translation1 + translation2) / 2.0;
        REQUIRE(averageTranslation.isApprox(evaluator.getOutput()));
    }
}
