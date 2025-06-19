/**
 * @file EMAWithLimitsTest.cpp
 * @authors Giulio Romualdi
 * @copyright 2025 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the BSD-3-Clause license.
 */

// Catch2
#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <BipedalLocomotion/JointLevelControllers/EMAWithLimits.h>
#include <BipedalLocomotion/ParametersHandler/StdImplementation.h>

using namespace BipedalLocomotion::JointLevelControllers;
using namespace BipedalLocomotion::ParametersHandler;
using Catch::Approx;

using Vector1d = Eigen::Matrix<double, 1, 1>;

// Helper function to create parameter handler with basic configuration
std::shared_ptr<StdImplementation>
createBasicParameterHandler(double scale = 0.5,
                            double alpha = 0.8,
                            const Eigen::VectorXd& lowerLimit = Eigen::Vector3d(-1.0, -2.0, -3.0),
                            const Eigen::VectorXd& upperLimit = Eigen::Vector3d(1.0, 2.0, 3.0),
                            double softLimitFactor = 0.9)
{
    auto handler = std::make_shared<StdImplementation>();
    handler->setParameter("scale", scale);
    handler->setParameter("alpha", alpha);
    handler->setParameter("lower_limit", lowerLimit);
    handler->setParameter("upper_limit", upperLimit);
    handler->setParameter("soft_limit_factor", softLimitFactor);
    return handler;
}

TEST_CASE("EMAWithLimits - Initialization")
{
    EMAWithLimits controller;

    SECTION("Valid initialization")
    {
        auto handler = createBasicParameterHandler();
        REQUIRE(controller.initialize(handler));
    }

    SECTION("Invalid parameter handler (nullptr)")
    {
        std::weak_ptr<const IParametersHandler> nullHandler;
        REQUIRE_FALSE(controller.initialize(nullHandler));
    }

    SECTION("Missing required parameters")
    {
        auto handler = std::make_shared<StdImplementation>();
        // Missing all required parameters
        REQUIRE_FALSE(controller.initialize(handler));
    }

    SECTION("Missing scale parameter")
    {
        auto handler = std::make_shared<StdImplementation>();
        handler->setParameter("alpha", 0.8);
        handler->setParameter("lower_limit", Eigen::Vector2d(-1.0, -2.0));
        handler->setParameter("upper_limit", Eigen::Vector2d(1.0, 2.0));
        REQUIRE_FALSE(controller.initialize(handler));
    }

    SECTION("Missing alpha parameter")
    {
        auto handler = std::make_shared<StdImplementation>();
        handler->setParameter("scale", 0.5);
        handler->setParameter("lower_limit", Eigen::Vector2d(-1.0, -2.0));
        handler->setParameter("upper_limit", Eigen::Vector2d(1.0, 2.0));
        REQUIRE_FALSE(controller.initialize(handler));
    }

    SECTION("Size mismatch between lower and upper limits")
    {
        auto handler = std::make_shared<StdImplementation>();
        handler->setParameter("scale", 0.5);
        handler->setParameter("alpha", 0.8);
        handler->setParameter("lower_limit", Eigen::Vector2d(-1.0, -2.0)); // Size 2
        handler->setParameter("upper_limit", Eigen::Vector3d(1.0, 2.0, 3.0)); // Size 3
        REQUIRE_FALSE(controller.initialize(handler));
    }

    SECTION("Invalid scale (zero)")
    {
        auto handler = createBasicParameterHandler(0.0); // scale = 0
        REQUIRE_FALSE(controller.initialize(handler));
    }

    SECTION("Invalid scale (negative)")
    {
        auto handler = createBasicParameterHandler(-0.5); // scale < 0
        REQUIRE_FALSE(controller.initialize(handler));
    }

    SECTION("Valid alpha (zero)")
    {
        auto handler = createBasicParameterHandler(0.5, 0.0); // alpha = 0
        REQUIRE(controller.initialize(handler));
    }

    SECTION("Valid alpha (one)")
    {
        auto handler = createBasicParameterHandler(0.5, 1.0); // alpha = 1
        REQUIRE(controller.initialize(handler));
    }

    SECTION("Invalid alpha (greater than one)")
    {
        auto handler = createBasicParameterHandler(0.5, 1.5); // alpha > 1
        REQUIRE_FALSE(controller.initialize(handler));
    }

    SECTION("Invalid alpha (negative)")
    {
        auto handler = createBasicParameterHandler(0.5, -0.1); // alpha < 0
        REQUIRE_FALSE(controller.initialize(handler));
    }

    SECTION("Valid initialization without soft limit factor (default)")
    {
        auto handler = std::make_shared<StdImplementation>();
        handler->setParameter("scale", 0.5);
        handler->setParameter("alpha", 0.8);
        handler->setParameter("lower_limit", Eigen::Vector2d(-1.0, -2.0));
        handler->setParameter("upper_limit", Eigen::Vector2d(1.0, 2.0));
        // soft_limit_factor will use default value
        REQUIRE(controller.initialize(handler));
    }

    SECTION("Invalid soft limit factor (zero)")
    {
        auto handler = createBasicParameterHandler(0.5,
                                                   0.8,
                                                   Eigen::Vector2d(-1.0, -2.0),
                                                   Eigen::Vector2d(1.0, 2.0),
                                                   0.0); // softLimitFactor = 0
        REQUIRE_FALSE(controller.initialize(handler));
    }

    SECTION("Invalid soft limit factor (negative)")
    {
        auto handler = createBasicParameterHandler(0.5,
                                                   0.8,
                                                   Eigen::Vector2d(-1.0, -2.0),
                                                   Eigen::Vector2d(1.0, 2.0),
                                                   -0.5); // softLimitFactor < 0
        REQUIRE_FALSE(controller.initialize(handler));
    }
}

TEST_CASE("EMAWithLimits - Input Validation and State Management")
{
    EMAWithLimits controller;
    auto handler = createBasicParameterHandler();
    REQUIRE(controller.initialize(handler));

    SECTION("Valid input size")
    {
        Eigen::Vector3d input(0.1, 0.2, 0.3);
        REQUIRE(controller.setInput(input));
    }

    SECTION("Invalid input size (too small)")
    {
        Eigen::Vector2d input(0.1, 0.2); // Expected size is 3
        REQUIRE_FALSE(controller.setInput(input));
    }

    SECTION("Invalid input size (too large)")
    {
        Eigen::Vector4d input(0.1, 0.2, 0.3, 0.4); // Expected size is 3
        REQUIRE_FALSE(controller.setInput(input));
    }

    SECTION("Output validity before reset")
    {
        REQUIRE_FALSE(controller.isOutputValid());
    }

    SECTION("Advance fails before reset")
    {
        Eigen::Vector3d input(0.1, 0.2, 0.3);
        REQUIRE(controller.setInput(input));
        REQUIRE_FALSE(controller.advance()); // Should fail because not reset
    }

    SECTION("Output validity after reset but before advance")
    {
        controller.reset();
        REQUIRE(controller.isOutputValid()); // Should be valid after reset
    }

    SECTION("Successful advance after reset")
    {
        controller.reset();
        Eigen::Vector3d input(0.1, 0.2, 0.3);
        REQUIRE(controller.setInput(input));
        REQUIRE(controller.advance());
        REQUIRE(controller.isOutputValid());
    }
}

TEST_CASE("EMAWithLimits - Basic Functionality")
{
    constexpr double scale = 0.5;
    constexpr double alpha = 0.8;
    const Eigen::Vector2d lowerLimit(-2.0, -4.0);
    const Eigen::Vector2d upperLimit(2.0, 4.0);
    constexpr double softLimitFactor = 1.0; // No soft limiting

    auto handler
        = createBasicParameterHandler(scale, alpha, lowerLimit, upperLimit, softLimitFactor);
    EMAWithLimits controller;
    REQUIRE(controller.initialize(handler));
    controller.reset();

    SECTION("Small input within scaled limits")
    {
        Eigen::Vector2d input(0.5, -0.5); // Will be scaled to (0.25, -0.25)
        REQUIRE(controller.setInput(input));
        REQUIRE(controller.advance());

        const Eigen::VectorXd output = controller.getOutput();
        REQUIRE(output.size() == 2);

        // Expected: scale * input, then unscale to joint limits, then EMA
        // Since previousAppliedActions = 0, EMA: alpha * unscaled + (1-alpha) * 0
        // Unscaled: input_scaled * (upper - lower) / 2 + (upper + lower) / 2
        double expectedUnscaled1 = 0.25 * (2.0 - (-2.0)) / 2.0 + (2.0 + (-2.0)) / 2.0; // = 0.5
        double expectedUnscaled2 = -0.25 * (4.0 - (-4.0)) / 2.0 + (4.0 + (-4.0)) / 2.0; // = -1.0
        double expected1 = alpha * expectedUnscaled1; // = 0.8 * 0.5 = 0.4
        double expected2 = alpha * expectedUnscaled2; // = 0.8 * (-1.0) = -0.8

        REQUIRE(output(0) == Approx(expected1).epsilon(1e-6));
        REQUIRE(output(1) == Approx(expected2).epsilon(1e-6));
    }

    SECTION("Input exceeding scale limits gets clipped")
    {
        Eigen::Vector2d input(5.0, -5.0); // Large input, will be clipped to (1.0, -1.0) after
                                          // scaling
        REQUIRE(controller.setInput(input));
        REQUIRE(controller.advance());

        const Eigen::VectorXd output = controller.getOutput();

        // Scaled input: (2.5, -2.5) -> clipped to (1.0, -1.0)
        // Unscaled: (1.0, -1.0) -> (2.0, -4.0) (full range)
        // EMA: alpha * unscaled = (0.8 * 2.0, 0.8 * -4.0) = (1.6, -3.2)
        REQUIRE(output(0) == Approx(1.6).epsilon(1e-6));
        REQUIRE(output(1) == Approx(-3.2).epsilon(1e-6));
    }

    SECTION("EMA behavior over multiple steps")
    {
        // First step
        Eigen::Vector2d input1(1.0, 0.0);
        REQUIRE(controller.setInput(input1));
        REQUIRE(controller.advance());
        Eigen::VectorXd output1 = controller.getOutput();

        // Second step with same input
        REQUIRE(controller.setInput(input1));
        REQUIRE(controller.advance());
        Eigen::VectorXd output2 = controller.getOutput();

        // Third step with same input
        REQUIRE(controller.setInput(input1));
        REQUIRE(controller.advance());
        Eigen::VectorXd output3 = controller.getOutput();

        // Output should converge towards steady state
        REQUIRE(std::abs(output2(0) - output1(0)) > std::abs(output3(0) - output2(0)));
        REQUIRE(output3(0) > output2(0)); // Should be increasing toward steady state
        REQUIRE(output2(0) > output1(0));
    }
}

TEST_CASE("EMAWithLimits - Soft Limits")
{
    constexpr double scale = 1.0;
    constexpr double alpha = 1.0; // No EMA smoothing for clearer testing
    const Vector1d lowerLimit(-2.0);
    const Vector1d upperLimit(2.0);
    constexpr double softLimitFactor = 0.5; // 50% of full range

    auto handler
        = createBasicParameterHandler(scale, alpha, lowerLimit, upperLimit, softLimitFactor);
    EMAWithLimits controller;
    REQUIRE(controller.initialize(handler));
    controller.reset();

    SECTION("Input within soft limits")
    {
        Vector1d input(0.5); // Should map to soft limit range
        REQUIRE(controller.setInput(input));
        REQUIRE(controller.advance());

        const Eigen::VectorXd output = controller.getOutput();

        // Soft limits: center = 0, range = 4 * 0.5 = 2, so soft limits = [-1, 1]
        // Input 0.5 -> maps to 0.5 * 1.0 = 0.5 within soft limits
        REQUIRE(output(0) == Approx(0.5).epsilon(1e-6));
    }

    SECTION("Input exceeding soft limits gets clipped")
    {
        Vector1d input(1.0); // Will map to soft limit boundary
        REQUIRE(controller.setInput(input));
        REQUIRE(controller.advance());

        const Eigen::VectorXd output = controller.getOutput();

        // Input 1.0 maps to upper soft limit boundary = 1.0
        REQUIRE(output(0) == Approx(1.0).epsilon(1e-6));
    }
}

TEST_CASE("EMAWithLimits - Edge Cases")
{
    SECTION("Zero input")
    {
        auto handler = createBasicParameterHandler();
        EMAWithLimits controller;
        REQUIRE(controller.initialize(handler));
        controller.reset();

        Eigen::Vector3d input = Eigen::Vector3d::Zero();
        REQUIRE(controller.setInput(input));
        REQUIRE(controller.advance());

        const Eigen::VectorXd output = controller.getOutput();
        REQUIRE(output.size() == 3);
        // Zero input should produce zero output (at center of joint ranges)
        REQUIRE(output(0) == Approx(0.0).margin(1e-10));
        REQUIRE(output(1) == Approx(0.0).margin(1e-10));
        REQUIRE(output(2) == Approx(0.0).margin(1e-10));
    }

    SECTION("Asymmetric joint limits")
    {
        const Vector1d lowerLimit(-1.0);
        const Vector1d upperLimit(3.0); // Asymmetric range
        auto handler = createBasicParameterHandler(1.0, 0.5, lowerLimit, upperLimit, 1.0);

        EMAWithLimits controller;
        REQUIRE(controller.initialize(handler));
        controller.reset();

        // Test that center is correctly computed
        Vector1d input(0.0);
        REQUIRE(controller.setInput(input));
        REQUIRE(controller.advance());

        const Eigen::VectorXd output = controller.getOutput();
        // Center should be (upperLimit + lowerLimit) / 2 = (3.0 + (-1.0)) / 2 = 1.0
        // With alpha = 0.5 and previous = 0, output = 0.5 * 1.0 = 0.5
        REQUIRE(output(0) == Approx(0.5).epsilon(1e-6));
    }

    SECTION("Very small alpha (slow EMA)")
    {
        auto handler = createBasicParameterHandler(1.0, 0.01); // Very small alpha
        EMAWithLimits controller;
        REQUIRE(controller.initialize(handler));
        controller.reset();

        Eigen::Vector3d input(1.0, 1.0, 1.0);
        REQUIRE(controller.setInput(input));
        REQUIRE(controller.advance());

        const Eigen::VectorXd output1 = controller.getOutput();

        // With very small alpha, output should be very close to previous (which is 0)
        REQUIRE(std::abs(output1(0)) < 0.1); // Should be very small
        REQUIRE(std::abs(output1(1)) < 0.1);
        REQUIRE(std::abs(output1(2)) < 0.1);
    }

    SECTION("Very large alpha (fast EMA)")
    {
        auto handler = createBasicParameterHandler(1.0, 0.99); // Very large alpha
        EMAWithLimits controller;
        REQUIRE(controller.initialize(handler));
        controller.reset();

        Eigen::Vector3d input(1.0, 1.0, 1.0);
        REQUIRE(controller.setInput(input));
        REQUIRE(controller.advance());

        const Eigen::VectorXd output1 = controller.getOutput();

        // With very large alpha, output should be very close to current unscaled input
        // This depends on the unscaling, but should be close to the target
        REQUIRE(std::abs(output1(0)) > 0.5); // Should be significant
    }
}

TEST_CASE("EMAWithLimits - Reset Functionality")
{
    auto handler = createBasicParameterHandler(1.0, 0.5);
    EMAWithLimits controller;
    REQUIRE(controller.initialize(handler));

    SECTION("Reset sets state to ready")
    {
        REQUIRE_FALSE(controller.isOutputValid());
        controller.reset();
        REQUIRE(controller.isOutputValid());
    }

    SECTION("Reset clears previous applied actions")
    {
        controller.reset();

        // First advance
        Eigen::Vector3d input(1.0, 1.0, 1.0);
        REQUIRE(controller.setInput(input));
        REQUIRE(controller.advance());
        Eigen::VectorXd output1 = controller.getOutput();

        // Reset and advance again with same input
        controller.reset();
        REQUIRE(controller.setInput(input));
        REQUIRE(controller.advance());
        Eigen::VectorXd output2 = controller.getOutput();

        // Outputs should be identical because reset clears history
        REQUIRE(output1(0) == Approx(output2(0)).epsilon(1e-10));
        REQUIRE(output1(1) == Approx(output2(1)).epsilon(1e-10));
        REQUIRE(output1(2) == Approx(output2(2)).epsilon(1e-10));
    }

    SECTION("Multiple resets work correctly")
    {
        controller.reset();
        REQUIRE(controller.isOutputValid());

        controller.reset();
        REQUIRE(controller.isOutputValid());

        controller.reset();
        REQUIRE(controller.isOutputValid());
    }
}

TEST_CASE("EMAWithLimits - Mathematical Properties")
{
    SECTION("EMA convergence")
    {
        auto handler = createBasicParameterHandler(1.0,
                                                   0.8, //
                                                   Vector1d(-10.0),
                                                   Vector1d(10.0));
        EMAWithLimits controller;
        REQUIRE(controller.initialize(handler));
        controller.reset();

        // Apply constant input for many steps
        const Vector1d constantInput(0.5);
        const int numSteps = 20;

        for (int i = 0; i < numSteps; ++i)
        {
            REQUIRE(controller.setInput(constantInput));
            REQUIRE(controller.advance());
            REQUIRE(controller.isOutputValid());
        }
        const Eigen::VectorXd finalOutput = controller.getOutput();

        // Do only one step to compare
        controller.reset();
        REQUIRE(controller.setInput(constantInput));
        REQUIRE(controller.advance());
        const Eigen::VectorXd singleStepOutput = controller.getOutput();

        // After many steps, should be much closer to steady state than single step
        REQUIRE(std::abs(finalOutput(0)) > std::abs(singleStepOutput(0)));
    }

    SECTION("Linearity of scaling")
    {
        auto handler = createBasicParameterHandler(2.0,
                                                   1.0, //
                                                   Vector1d(-10.0),
                                                   Vector1d(10.0));
        EMAWithLimits controller;
        REQUIRE(controller.initialize(handler));

        // Test input 1
        controller.reset();
        Vector1d input1(0.1);
        REQUIRE(controller.setInput(input1));
        REQUIRE(controller.advance());
        Eigen::VectorXd output1 = controller.getOutput();

        // Test input 2 (double)
        controller.reset();
        Vector1d input2(0.2);
        REQUIRE(controller.setInput(input2));
        REQUIRE(controller.advance());
        Eigen::VectorXd output2 = controller.getOutput();

        // Due to scaling and unscaling, output2 should be approximately 2 * output1
        REQUIRE(output2(0) == Approx(2.0 * output1(0)).epsilon(1e-6));
    }

    SECTION("Boundary behavior")
    {
        auto handler = createBasicParameterHandler(1.0, 1.0, Vector1d(-1.0), Vector1d(1.0), 1.0);
        EMAWithLimits controller;
        REQUIRE(controller.initialize(handler));
        controller.reset();

        // Test extreme positive input
        Vector1d maxInput(1000.0); // Very large input
        REQUIRE(controller.setInput(maxInput));
        REQUIRE(controller.advance());
        Eigen::VectorXd output = controller.getOutput();

        // Should be clipped to upper limit
        REQUIRE(output(0) <= 1.0);
        REQUIRE(output(0) == Approx(1.0).epsilon(1e-6));
    }
}
