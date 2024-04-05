/**
 * @file ContinousContactModelTest.cpp
 * @authors Giulio Romualdi
 * @copyright 2020 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the BSD-3-Clause license.
 */

#include <random>

// Catch2
#include <catch2/catch_test_macros.hpp>

#include <iDynTree/EigenHelpers.h>
#include <iDynTree/SpatialAcc.h>

#include <BipedalLocomotion/ContactModels/ContinuousContactModel.h>
#include <BipedalLocomotion/ParametersHandler/StdImplementation.h>

using namespace iDynTree;
using namespace BipedalLocomotion::ContactModels;
using namespace BipedalLocomotion::ParametersHandler;

template <typename T, typename U>
void checkVectorAreEqual(const T& vector1, const U& vector2, double tol = 0)
{
    REQUIRE(vector1.size() == vector2.size());

    for (unsigned int i = 0; i < vector1.size(); i++)
        REQUIRE(std::abs(vector1[i] - vector2[i]) <= tol);
}

TEST_CASE("Continuous Contact")
{
    Transform world_T_link{Transform::Identity()};
    world_T_link.setRotation(Rotation::RPY(-0.15, 0.2, 0.1));
    world_T_link.setPosition(Position(-0.02, 0.01, 0.005));

    Transform nullForceTransform{Transform::Identity()};
    Twist linkVelocity{Twist::Zero()};
    toEigen(linkVelocity.getLinearVec3()).setRandom();
    toEigen(linkVelocity.getAngularVec3()).setRandom();

    constexpr double springCoeff = 2000.0;
    constexpr double damperCoeff = 100.0;

    constexpr double length = 0.12;
    constexpr double width = 0.09;

    std::shared_ptr<IParametersHandler> handler = std::make_shared<StdImplementation>();
    handler->setParameter("spring_coeff", springCoeff);
    handler->setParameter("damper_coeff", damperCoeff);
    handler->setParameter("length", length);
    handler->setParameter("width", width);

    ContinuousContactModel model;
    REQUIRE(model.initialize(handler));
    model.setState(linkVelocity, world_T_link);
    model.setNullForceTransform(nullForceTransform);

    SECTION("Test contact wrench")
    {
        // Compute numerical integral using Montecarlo method

        // Instantiate the two uniform distributions
        std::default_random_engine generator;
        generator.seed(42);
        std::uniform_real_distribution xAxis(-length / 2, length / 2);
        std::uniform_real_distribution yAxis(-width / 2, width / 2);

        constexpr double area = length * width;
        constexpr unsigned int samples = 1e4;

        Wrench numericalWrench;
        numericalWrench.zero();

        for (unsigned int i = 0; i < samples; i++)
        {
            double x = xAxis(generator);
            double y = yAxis(generator);

            // force
            toEigen(numericalWrench.getLinearVec3()) += toEigen(model.getForceAtPoint(x, y));

            // torque
            toEigen(numericalWrench.getAngularVec3())
                += toEigen(model.getTorqueGeneratedAtPoint(x, y));
        }

        toEigen(numericalWrench.getLinearVec3()) = toEigen(numericalWrench.getLinearVec3())
                                                   / samples * area
                                                   * std::abs(world_T_link.getRotation()(2, 2));
        toEigen(numericalWrench.getAngularVec3()) = toEigen(numericalWrench.getAngularVec3())
                                                    / samples * area
                                                    * std::abs(world_T_link.getRotation()(2, 2));

        constexpr double tolerance = 1e-2;

        checkVectorAreEqual(numericalWrench.getLinearVec3(),
                            model.getContactWrench().getLinearVec3(),
                            tolerance);
        checkVectorAreEqual(numericalWrench.getAngularVec3(),
                            model.getContactWrench().getAngularVec3(),
                            tolerance);
    }


    SECTION("Test regressor")
    {
        MatrixDynSize regressor = model.getRegressor();
        Wrench wrenchComputed;
        Vector2 contactParams;
        contactParams(0) = springCoeff;
        contactParams(1) = damperCoeff;
        toEigen(wrenchComputed.getLinearVec3()) = toEigen(regressor).topRows<3>() * toEigen(contactParams);
        toEigen(wrenchComputed.getAngularVec3()) = toEigen(regressor).bottomRows<3>() * toEigen(contactParams);

        constexpr double tolerance = 1e-7;
        checkVectorAreEqual(wrenchComputed.getLinearVec3(),
                            model.getContactWrench().getLinearVec3(),
                            tolerance);
        checkVectorAreEqual(wrenchComputed.getAngularVec3(),
                            model.getContactWrench().getAngularVec3(),
                            tolerance);
    }

    SECTION("Test contact dynamics")
    {
        // Instantiate the acceleration
        SpatialAcc acceleration;
        for (unsigned int i = 0; i < acceleration.size(); i++)
            acceleration(i) = 1;

        // define numerical step
        double numericalDerivStep = 1e-6;

        // compute the numerical derivative
        Transform world_T_link_prev, world_T_link_next;
        Twist linkVelocity_prev, linkVelocity_next;

        Position world_T_link_nextPos, world_T_link_prevPos;
        Rotation world_T_link_nextRotation, world_T_link_prevRotation;

        // the propagation of the transformation is based under the assumption of twists written in
        // mixed representation

        // Position propagation
        toEigen(world_T_link_prevPos)
            = toEigen(world_T_link.getPosition())
              - toEigen(linkVelocity.getLinearVec3()) * numericalDerivStep;

        toEigen(world_T_link_nextPos)
            = toEigen(world_T_link.getPosition())
              + toEigen(linkVelocity.getLinearVec3()) * numericalDerivStep;

        AngularMotionVector3 linkUnitRotation;

        // Rotation propagation
        // R(t - dt) = exp(-skew(omega) dt) R(t) (this is valid for MIXED only and CONSTANT angular
        // velocity)
        toEigen(linkUnitRotation) = -toEigen(linkVelocity.getAngularVec3()) * numericalDerivStep;
        world_T_link_prevRotation = linkUnitRotation.exp() * world_T_link.getRotation();

        // R(t + dt) = exp(skew(omega) dt) R(t) (this is valid for MIXED only and CONSTANT angular
        // velocity)
        toEigen(linkUnitRotation) = toEigen(linkVelocity.getAngularVec3()) * numericalDerivStep;
        world_T_link_nextRotation = linkUnitRotation.exp() * world_T_link.getRotation();

        // update the transformations
        world_T_link_prev.setPosition(world_T_link_prevPos);
        world_T_link_prev.setRotation(world_T_link_prevRotation);

        world_T_link_next.setPosition(world_T_link_nextPos);
        world_T_link_next.setRotation(world_T_link_nextRotation);

        // Velocity (linear and angular) propagation
        toEigen(linkVelocity_prev.getLinearVec3())
            = toEigen(linkVelocity.getLinearVec3())
              - toEigen(acceleration.getLinearVec3()) * numericalDerivStep;
        toEigen(linkVelocity_prev.getAngularVec3())
            = toEigen(linkVelocity.getAngularVec3())
              - toEigen(acceleration.getAngularVec3()) * numericalDerivStep;

        toEigen(linkVelocity_next.getLinearVec3())
            = toEigen(linkVelocity.getLinearVec3())
              + toEigen(acceleration.getLinearVec3()) * numericalDerivStep;
        toEigen(linkVelocity_next.getAngularVec3())
            = toEigen(linkVelocity.getAngularVec3())
              + toEigen(acceleration.getAngularVec3()) * numericalDerivStep;

        Vector6 contactWrenchRate;
        toEigen(contactWrenchRate) = toEigen(model.getAutonomousDynamics())
                                     + toEigen(model.getControlMatrix()) * toEigen(acceleration);

        model.setState(linkVelocity_prev, world_T_link_prev);
        model.setNullForceTransform(nullForceTransform);

        Wrench contactWrench_prev = model.getContactWrench();

        model.setState(linkVelocity_next, world_T_link_next);
        model.setNullForceTransform(nullForceTransform);

        Wrench contactWrench_next = model.getContactWrench();


        // Evaluate the numerical rate of change
        Vector6 contactWrenchRateNumerical;
        toEigen(contactWrenchRateNumerical)
            = (toEigen(contactWrench_next) - toEigen(contactWrench_prev))
              / (2 * numericalDerivStep);

        constexpr double tolerance = 1e-4;
        checkVectorAreEqual(contactWrenchRateNumerical, contactWrenchRate, tolerance);
    }
}
