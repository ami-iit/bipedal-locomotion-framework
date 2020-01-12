/**
 * @file ContinousContactModelTest.cpp
 * @authors Giulio Romualdi
 * @copyright 2020 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the GNU Lesser General Public License v2.1 or any later version.
 */

#include <random>

// Catch2
#include <catch2/catch.hpp>

#include <iDynTree/Core/EigenHelpers.h>
#include <iDynTree/Core/SpatialAcc.h>

#include <BipedalLocomotionControllers/ContactModels/ContinuousContactModel.h>

using namespace iDynTree;
using namespace BipedalLocomotionControllers::ContactModels;

template <typename T, typename U>
bool checkVectorAreEqual(const T& vector1, const U& vector2, double tol = 0)
{
    REQUIRE(vector1.size() == vector2.size());

    for (unsigned int i = 0; i < vector1.size(); i++)
        REQUIRE(std::fabs(vector1[i] - vector2[i]) <= tol);
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

    double springCoeff = 2000.0;
    double damperCoeff = 100.0;

    double length = 0.12;
    double width = 0.09;

    ContinuousContactModel model({{"length", length}, {"width", width}},
                                 {{"spring_coeff", springCoeff}, {"damper_coeff", damperCoeff}});

    model.setState({{"frame_transform", world_T_link},
                    {"null_force_transform", nullForceTransform},
                    {"twist", linkVelocity}});

    SECTION("Test contact wrench")
    {
        // Compute numerical integral using Montecarlo method

        // Instantiate the two uniform distributions
        std::default_random_engine generator;
        generator.seed(42);
        std::uniform_real_distribution xAxis(-length / 2, length / 2);
        std::uniform_real_distribution yAxis(-width / 2, width / 2);

        double area = length * width;
        unsigned int samples = 1e7;

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

        double tollerance = 1e-4;

        checkVectorAreEqual(numericalWrench.getLinearVec3(),
                            model.getContactWrench().getLinearVec3(),
                            tollerance);
        checkVectorAreEqual(numericalWrench.getAngularVec3(),
                            model.getContactWrench().getAngularVec3(),
                            tollerance);
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

        model.setState({{"frame_transform", world_T_link_prev},
                        {"null_force_transform", nullForceTransform},
                        {"twist", linkVelocity_prev}});
        Wrench contactWrench_prev = model.getContactWrench();

        model.setState({{"frame_transform", world_T_link_next},
                        {"null_force_transform", nullForceTransform},
                        {"twist", linkVelocity_next}});
        Wrench contactWrench_next = model.getContactWrench();


        // Evaluate the numerical rate of change
        Vector6 contactWrenchRateNumerical;
        toEigen(contactWrenchRateNumerical)
            = (toEigen(contactWrench_next) - toEigen(contactWrench_prev))
              / (2 * numericalDerivStep);

        double tollerance = 1e-4;
        checkVectorAreEqual(contactWrenchRateNumerical, contactWrenchRate, tollerance);
    }
}
