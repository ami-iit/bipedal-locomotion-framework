/**
 * @file ContinousContactModelTest.cpp
 * @authors Giulio Romualdi
 * @copyright 2020 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the GNU Lesser General Public License v2.1 or any later version.
 */

#include <random>

// Catch2
#include <catch2/catch.hpp>

#include <BipedalLocomotionControllers/ContactModels/ContinuousContactModel.h>
#include <iDynTree/Core/EigenHelpers.h>

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
    iDynTree::Transform frameTransform{iDynTree::Transform::Identity()};
    frameTransform.setRotation(iDynTree::Rotation::RPY(-0.15, 0.2, 0.1));
    frameTransform.setPosition(iDynTree::Position(-0.02, 0.01, 0.005));

    iDynTree::Transform nullForceTransform{iDynTree::Transform::Identity()};
    iDynTree::Twist twist{iDynTree::Twist::Zero()};
    iDynTree::toEigen(twist.getLinearVec3()).setRandom();
    iDynTree::toEigen(twist.getAngularVec3()).setRandom();

    double springCoeff = 2000.0;
    double damperCoeff = 100.0;

    double length = 0.12;
    double width = 0.09;

    ContinuousContactModel model({{"length", length}, {"width", width}},
                                 {{"spring_coeff", springCoeff}, {"damper_coeff", damperCoeff}});

    model.setState({{"frame_transform", frameTransform},
                    {"null_force_transform", nullForceTransform},
                    {"twist", twist}});

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

        iDynTree::Wrench numericalWrench;
        numericalWrench.zero();

        for (unsigned int i = 0; i < samples; i++)
        {
            double x = xAxis(generator);
            double y = yAxis(generator);

            // force
            iDynTree::toEigen(numericalWrench.getLinearVec3())
                += iDynTree::toEigen(model.getForceAtPoint(x, y));

            // torque
            iDynTree::toEigen(numericalWrench.getAngularVec3())
                += iDynTree::toEigen(model.getTorqueGeneratedAtPoint(x, y));
        }

        iDynTree::toEigen(numericalWrench.getLinearVec3())
            = iDynTree::toEigen(numericalWrench.getLinearVec3()) / samples * area
              * std::abs(frameTransform.getRotation()(2, 2));
        iDynTree::toEigen(numericalWrench.getAngularVec3())
            = iDynTree::toEigen(numericalWrench.getAngularVec3()) / samples * area
              * std::abs(frameTransform.getRotation()(2, 2));

        double tollerance = 1e-4;

        checkVectorAreEqual(numericalWrench.getLinearVec3(),
                            model.getContactWrench().getLinearVec3(),
                            tollerance);
        checkVectorAreEqual(numericalWrench.getAngularVec3(),
                            model.getContactWrench().getAngularVec3(),
                            tollerance);
    }
}
