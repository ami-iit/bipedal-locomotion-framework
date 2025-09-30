/**
 * @file PositionToCurrentControllerTest.cpp
 * @authors Giulio Romualdi
 * @copyright 2025 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the BSD-3-Clause license.
 */

// Catch2
#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <BipedalLocomotion/JointLevelControllers/PositionToCurrentController.h>
#include <BipedalLocomotion/JointLevelControllers/PositionToJointController.h>
#include <BipedalLocomotion/ParametersHandler/StdImplementation.h>

using namespace BipedalLocomotion::JointLevelControllers;
using Catch::Approx;

// Helper function to create parameter handler with basic configuration
std::shared_ptr<BipedalLocomotion::ParametersHandler::StdImplementation>
createBasicParameterHandler(const std::vector<std::string>& joints,
                            double kp = 10.0,
                            double gearRatio = 100.0,
                            double kTau = 0.111,
                            double kd = 0.0)
{
    auto ptr = std::make_shared<BipedalLocomotion::ParametersHandler::StdImplementation>();
    ptr->setParameter("joints_list", joints);

    auto ptrKp = std::make_shared<BipedalLocomotion::ParametersHandler::StdImplementation>();
    auto ptrKd = std::make_shared<BipedalLocomotion::ParametersHandler::StdImplementation>();
    auto ptrGear = std::make_shared<BipedalLocomotion::ParametersHandler::StdImplementation>();
    auto ptrKTau = std::make_shared<BipedalLocomotion::ParametersHandler::StdImplementation>();

    for (const auto& joint : joints)
    {
        ptrKp->setParameter(joint, kp);
        ptrKd->setParameter(joint, kd);
        ptrGear->setParameter(joint, gearRatio);
        ptrKTau->setParameter(joint, kTau);
    }

    ptr->setGroup("kp", ptrKp);
    ptr->setGroup("kd", ptrKd);
    ptr->setGroup("gearbox", ptrGear);
    ptr->setGroup("k_tau", ptrKTau);

    return ptr;
}

TEST_CASE("PositionToCurrentController - Basic Initialization")
{
    std::vector<std::string> joints{"joint_1", "joint_2", "joint_3"};

    SECTION("Valid initialization with required parameters")
    {
        auto ptr = createBasicParameterHandler(joints);
        PositionToCurrentController controller;
        REQUIRE(controller.initialize(ptr));
    }

    SECTION("Missing joints_list parameter")
    {
        auto ptr = std::make_shared<BipedalLocomotion::ParametersHandler::StdImplementation>();
        PositionToCurrentController controller;
        REQUIRE_FALSE(controller.initialize(ptr));
    }

    SECTION("Missing required parameter groups")
    {
        auto ptr = std::make_shared<BipedalLocomotion::ParametersHandler::StdImplementation>();
        ptr->setParameter("joints_list", joints);

        PositionToCurrentController controller;
        REQUIRE_FALSE(controller.initialize(ptr));
    }

    SECTION("Invalid parameter handler (nullptr)")
    {
        std::weak_ptr<const BipedalLocomotion::ParametersHandler::IParametersHandler> nullPtr;
        PositionToCurrentController controller;
        REQUIRE_FALSE(controller.initialize(nullPtr));
    }
}

TEST_CASE("PositionToCurrentController - Basic Position Control")
{
    constexpr double kp = 10.0;
    constexpr double gearRatio = 100.0;
    constexpr double kTau = 0.111;
    std::vector<std::string> joints{"joint_1"};

    auto ptr = createBasicParameterHandler(joints, kp, gearRatio, kTau);
    PositionToCurrentController controller;
    REQUIRE(controller.initialize(ptr));

    SECTION("Zero position error produces zero current")
    {
        PositionToJointControllerInput input;
        input.referencePosition = Eigen::VectorXd::Ones(1) * 0.5;
        input.feedbackPosition = Eigen::VectorXd::Ones(1) * 0.5;
        input.feedbackVelocity = Eigen::VectorXd::Zero(1);

        REQUIRE(controller.setInput(input));
        REQUIRE(controller.advance());
        REQUIRE(controller.isOutputValid());

        const auto& output = controller.getOutput();
        REQUIRE(output.size() == 1);
        REQUIRE(output(0) == Approx(0.0).margin(1e-10));
    }

    SECTION("Positive position error produces positive current")
    {
        PositionToJointControllerInput input;
        input.referencePosition = Eigen::VectorXd::Ones(1) * 1.0;
        input.feedbackPosition = Eigen::VectorXd::Zero(1);
        input.feedbackVelocity = Eigen::VectorXd::Zero(1);

        REQUIRE(controller.setInput(input));
        REQUIRE(controller.advance());
        REQUIRE(controller.isOutputValid());

        const auto& output = controller.getOutput();
        // Expected: kp * (1.0 - 0.0) / (gearRatio * kTau) = 10.0 / (100.0 * 0.111) = 0.9009
        REQUIRE(output(0) == Approx(0.9009).epsilon(1e-3));
    }

    SECTION("Negative position error produces negative current")
    {
        PositionToJointControllerInput input;
        input.referencePosition = Eigen::VectorXd::Zero(1);
        input.feedbackPosition = Eigen::VectorXd::Ones(1) * 1.0;
        input.feedbackVelocity = Eigen::VectorXd::Zero(1);

        REQUIRE(controller.setInput(input));
        REQUIRE(controller.advance());

        const auto& output = controller.getOutput();
        // Expected: kp * (0.0 - 1.0) / (gearRatio * kTau) = -10.0 / (100.0 * 0.111) = -0.9009
        REQUIRE(output(0) == Approx(-0.9009).epsilon(1e-3));
    }

    SECTION("Derivative term reduces current with positive velocity")
    {
        // Re-initialize controller with non-zero kd and zero kp to isolate D action
        constexpr double kd = 2.0;
        auto ptrD = createBasicParameterHandler(joints, /*kp*/ 0.0, gearRatio, kTau, kd);
        PositionToCurrentController ctrlD;
        REQUIRE(ctrlD.initialize(ptrD));

        PositionToJointControllerInput input;
        input.referencePosition = Eigen::VectorXd::Zero(1); // zero position error
        input.feedbackPosition = Eigen::VectorXd::Zero(1);
        input.feedbackVelocity = Eigen::VectorXd::Ones(1) * 1.5; // rad/s

        REQUIRE(ctrlD.setInput(input));
        REQUIRE(ctrlD.advance());

        const auto& output = ctrlD.getOutput();
        // Expected: (-kd * vel) / (gearRatio * kTau) = (-2.0 * 1.5) / (100 * 0.111)
        const double expected = (-kd * 1.5) / (gearRatio * kTau);
        REQUIRE(output(0) == Approx(expected).epsilon(1e-6));
    }
}

TEST_CASE("PositionToCurrentController - Tanh-based Friction Compensation")
{
    constexpr double kp = 10.0;
    constexpr double gearRatio = 100.0;
    constexpr double kTau = 0.111;
    constexpr double coulombFriction = 2.0; // Nm
    constexpr double activationVelocity = 1.0; // rad/s
    std::vector<std::string> joints{"joint_1"};

    auto ptr = createBasicParameterHandler(joints, kp, gearRatio, kTau);

    // Add friction compensation with activation velocity
    auto ptrFriction = std::make_shared<BipedalLocomotion::ParametersHandler::StdImplementation>();
    auto ptrActivation
        = std::make_shared<BipedalLocomotion::ParametersHandler::StdImplementation>();
    ptrFriction->setParameter("joint_1", coulombFriction);
    ptrActivation->setParameter("joint_1", activationVelocity);
    ptr->setGroup("coulomb_friction", ptrFriction);
    ptr->setGroup("activation_velocity", ptrActivation);

    PositionToCurrentController controller;
    REQUIRE(controller.initialize(ptr));

    SECTION("High positive velocity saturates friction compensation")
    {
        PositionToJointControllerInput input;
        input.referencePosition = Eigen::VectorXd::Zero(1);
        input.feedbackPosition = Eigen::VectorXd::Zero(1);
        input.feedbackVelocity = Eigen::VectorXd::Ones(1) * 5.0; // High positive velocity

        REQUIRE(controller.setInput(input));
        REQUIRE(controller.advance());

        const auto& output = controller.getOutput();
        // Expected: coulombFriction * tanh(5.0/1.0) / (gearRatio * kTau)
        // tanh(5.0) ≈ 0.9999, so ≈ 2.0 / (100.0 * 0.111) = 0.1802
        REQUIRE(output(0) == Approx(0.1802).epsilon(1e-2));
    }

    SECTION("High negative velocity saturates friction compensation")
    {
        PositionToJointControllerInput input;
        input.referencePosition = Eigen::VectorXd::Zero(1);
        input.feedbackPosition = Eigen::VectorXd::Zero(1);
        input.feedbackVelocity = Eigen::VectorXd::Ones(1) * -5.0; // High negative velocity

        REQUIRE(controller.setInput(input));
        REQUIRE(controller.advance());

        const auto& output = controller.getOutput();
        // Expected: coulombFriction * tanh(-5.0/1.0) / (gearRatio * kTau)
        // tanh(-5.0) ≈ -0.9999, so ≈ -2.0 / (100.0 * 0.111) = -0.1802
        REQUIRE(output(0) == Approx(-0.1802).epsilon(1e-2));
    }

    SECTION("Low velocity provides proportional friction compensation")
    {
        PositionToJointControllerInput input;
        input.referencePosition = Eigen::VectorXd::Zero(1);
        input.feedbackPosition = Eigen::VectorXd::Zero(1);
        input.feedbackVelocity = Eigen::VectorXd::Ones(1) * 0.1; // Low velocity

        REQUIRE(controller.setInput(input));
        REQUIRE(controller.advance());

        const auto& output = controller.getOutput();
        // Expected: coulombFriction * tanh(0.1/1.0) / (gearRatio * kTau)
        // tanh(0.1) ≈ 0.0997, so ≈ 2.0 * 0.0997 / (100.0 * 0.111) = 0.0180
        double expectedTanh = std::tanh(0.1);
        double expected = coulombFriction * expectedTanh / (gearRatio * kTau);
        REQUIRE(output(0) == Approx(expected).epsilon(1e-4));
    }

    SECTION("Zero velocity produces zero friction")
    {
        PositionToJointControllerInput input;
        input.referencePosition = Eigen::VectorXd::Zero(1);
        input.feedbackPosition = Eigen::VectorXd::Zero(1);
        input.feedbackVelocity = Eigen::VectorXd::Zero(1); // Zero velocity

        REQUIRE(controller.setInput(input));
        REQUIRE(controller.advance());

        const auto& output = controller.getOutput();
        // Expected: coulombFriction * tanh(0.0/1.0) / (gearRatio * kTau) = 0.0
        REQUIRE(output(0) == Approx(0.0).margin(1e-10));
    }

    SECTION("Combined position error and tanh friction compensation")
    {
        PositionToJointControllerInput input;
        input.referencePosition = Eigen::VectorXd::Ones(1) * 1.0;
        input.feedbackPosition = Eigen::VectorXd::Zero(1);
        input.feedbackVelocity = Eigen::VectorXd::Ones(1) * 1.0; // Velocity = activation velocity

        REQUIRE(controller.setInput(input));
        REQUIRE(controller.advance());

        const auto& output = controller.getOutput();
        // Expected: (kp * 1.0 + coulombFriction * tanh(1.0)) / (gearRatio * kTau)
        // tanh(1.0) ≈ 0.7616
        double tanhValue = std::tanh(1.0);
        double expected = (kp + coulombFriction * tanhValue) / (gearRatio * kTau);
        REQUIRE(output(0) == Approx(expected).epsilon(1e-3));
    }
}

TEST_CASE("PositionToCurrentController - Different Activation Velocities")
{
    constexpr double kp = 10.0;
    constexpr double gearRatio = 100.0;
    constexpr double kTau = 0.111;
    constexpr double coulombFriction = 2.0;
    std::vector<std::string> joints{"joint_1"};

    SECTION("High activation velocity (sharp transition)")
    {
        constexpr double activationVelocity = 10.0; // High activation velocity

        auto ptr = createBasicParameterHandler(joints, kp, gearRatio, kTau);
        auto ptrFriction
            = std::make_shared<BipedalLocomotion::ParametersHandler::StdImplementation>();
        auto ptrActivation
            = std::make_shared<BipedalLocomotion::ParametersHandler::StdImplementation>();
        ptrFriction->setParameter("joint_1", coulombFriction);
        ptrActivation->setParameter("joint_1", activationVelocity);
        ptr->setGroup("coulomb_friction", ptrFriction);
        ptr->setGroup("activation_velocity", ptrActivation);

        PositionToCurrentController controller;
        REQUIRE(controller.initialize(ptr));

        PositionToJointControllerInput input;
        input.referencePosition = Eigen::VectorXd::Zero(1);
        input.feedbackPosition = Eigen::VectorXd::Zero(1);
        input.feedbackVelocity = Eigen::VectorXd::Ones(1) * 0.5; // Small velocity

        REQUIRE(controller.setInput(input));
        REQUIRE(controller.advance());

        const auto& output = controller.getOutput();
        // With high activation velocity, even small feedbackVelocity should produce small tanh
        // tanh(0.5/10.0) = tanh(0.05) ≈ 0.05
        double tanhValue = std::tanh(0.5 / activationVelocity);
        double expected = coulombFriction * tanhValue / (gearRatio * kTau);
        REQUIRE(output(0) == Approx(expected).epsilon(1e-4));
    }

    SECTION("Low activation velocity (smooth transition)")
    {
        constexpr double activationVelocity = 0.1; // Low activation velocity

        auto ptr = createBasicParameterHandler(joints, kp, gearRatio, kTau);
        auto ptrFriction
            = std::make_shared<BipedalLocomotion::ParametersHandler::StdImplementation>();
        auto ptrActivation
            = std::make_shared<BipedalLocomotion::ParametersHandler::StdImplementation>();
        ptrFriction->setParameter("joint_1", coulombFriction);
        ptrActivation->setParameter("joint_1", activationVelocity);
        ptr->setGroup("coulomb_friction", ptrFriction);
        ptr->setGroup("activation_velocity", ptrActivation);

        PositionToCurrentController controller;
        REQUIRE(controller.initialize(ptr));

        PositionToJointControllerInput input;
        input.referencePosition = Eigen::VectorXd::Zero(1);
        input.feedbackPosition = Eigen::VectorXd::Zero(1);
        input.feedbackVelocity = Eigen::VectorXd::Ones(1) * 0.05; // Small velocity

        REQUIRE(controller.setInput(input));
        REQUIRE(controller.advance());

        const auto& output = controller.getOutput();
        // With low activation velocity, small feedbackVelocity should produce larger tanh
        // tanh(0.05/0.1) = tanh(0.5) ≈ 0.4621
        double tanhValue = std::tanh(0.05 / activationVelocity);
        double expected = coulombFriction * tanhValue / (gearRatio * kTau);
        REQUIRE(output(0) == Approx(expected).epsilon(1e-4));
    }
}

TEST_CASE("PositionToCurrentController - Default Activation Velocity")
{
    constexpr double kp = 10.0;
    constexpr double gearRatio = 100.0;
    constexpr double kTau = 0.111;
    constexpr double coulombFriction = 2.0;
    constexpr double defaultActivationVelocity = 1e-5; // Default value from implementation
    std::vector<std::string> joints{"joint_1"};

    auto ptr = createBasicParameterHandler(joints, kp, gearRatio, kTau);

    // Add only friction compensation (activation_velocity will use default)
    auto ptrFriction = std::make_shared<BipedalLocomotion::ParametersHandler::StdImplementation>();
    ptrFriction->setParameter("joint_1", coulombFriction);
    ptr->setGroup("coulomb_friction", ptrFriction);

    PositionToCurrentController controller;
    REQUIRE(controller.initialize(ptr));

    SECTION("Small velocity with default activation velocity")
    {
        PositionToJointControllerInput input;
        input.referencePosition = Eigen::VectorXd::Zero(1);
        input.feedbackPosition = Eigen::VectorXd::Zero(1);
        input.feedbackVelocity = Eigen::VectorXd::Ones(1) * 0.001; // Small velocity

        REQUIRE(controller.setInput(input));
        REQUIRE(controller.advance());

        const auto& output = controller.getOutput();
        // With very small activation velocity, tanh should saturate quickly
        // tanh(0.001/1e-5) = tanh(100) ≈ 1.0
        double tanhValue = std::tanh(0.001 / defaultActivationVelocity);
        double expected = coulombFriction * tanhValue / (gearRatio * kTau);
        REQUIRE(output(0) == Approx(expected).epsilon(1e-3));

        // Should be close to full friction compensation
        REQUIRE(std::abs(tanhValue) > 0.99); // tanh should be nearly saturated
    }
}

TEST_CASE("PositionToCurrentController - Current Limiting")
{
    constexpr double kp = 100.0; // High gain to trigger limiting
    constexpr double gearRatio = 100.0;
    constexpr double kTau = 0.111;
    constexpr double currentLimit = 1.0; // A
    std::vector<std::string> joints{"joint_1"};

    auto ptr = createBasicParameterHandler(joints, kp, gearRatio, kTau);

    // Add current limiting
    auto ptrLimit = std::make_shared<BipedalLocomotion::ParametersHandler::StdImplementation>();
    ptrLimit->setParameter("joint_1", currentLimit);
    ptr->setGroup("current_limit", ptrLimit);

    PositionToCurrentController controller;
    REQUIRE(controller.initialize(ptr));

    SECTION("Current is limited to maximum value")
    {
        PositionToJointControllerInput input;
        input.referencePosition = Eigen::VectorXd::Ones(1) * 10.0; // Large error
        input.feedbackPosition = Eigen::VectorXd::Zero(1);
        input.feedbackVelocity = Eigen::VectorXd::Zero(1);

        REQUIRE(controller.setInput(input));
        REQUIRE(controller.advance());

        const auto& output = controller.getOutput();
        REQUIRE(output(0) == Approx(currentLimit).margin(1e-6));
    }

    SECTION("Current is limited to minimum value")
    {
        PositionToJointControllerInput input;
        input.referencePosition = Eigen::VectorXd::Zero(1);
        input.feedbackPosition = Eigen::VectorXd::Ones(1) * 10.0; // Large negative error
        input.feedbackVelocity = Eigen::VectorXd::Zero(1);

        REQUIRE(controller.setInput(input));
        REQUIRE(controller.advance());

        const auto& output = controller.getOutput();
        REQUIRE(output(0) == Approx(-currentLimit).margin(1e-6));
    }
}

TEST_CASE("PositionToCurrentController - TN Curve Limiting")
{
    constexpr double kp = 100.0;
    constexpr double gearRatio = 100.0;
    constexpr double kTau = 0.111;
    constexpr double currentLimit = 10.0;
    constexpr double ratedSpeed = 5.0; // rad/s
    constexpr double noLoadSpeed = 15.0; // rad/s
    std::vector<std::string> joints{"joint_1"};

    auto ptr = createBasicParameterHandler(joints, kp, gearRatio, kTau);

    // Add TN curve parameters
    auto ptrLimit = std::make_shared<BipedalLocomotion::ParametersHandler::StdImplementation>();
    auto ptrRated = std::make_shared<BipedalLocomotion::ParametersHandler::StdImplementation>();
    auto ptrNoLoad = std::make_shared<BipedalLocomotion::ParametersHandler::StdImplementation>();

    ptrLimit->setParameter("joint_1", currentLimit);
    ptrRated->setParameter("joint_1", ratedSpeed);
    ptrNoLoad->setParameter("joint_1", noLoadSpeed);

    ptr->setGroup("current_limit", ptrLimit);
    ptr->setGroup("rated_speed", ptrRated);
    ptr->setGroup("no_load_speed", ptrNoLoad);

    PositionToCurrentController controller;
    REQUIRE(controller.initialize(ptr));

    SECTION("Flat region - low velocity")
    {
        PositionToJointControllerInput input;
        input.referencePosition = Eigen::VectorXd::Ones(1) * 10.0; // Large error
        input.feedbackPosition = Eigen::VectorXd::Zero(1);
        input.feedbackVelocity = Eigen::VectorXd::Ones(1) * 2.0; // Below rated speed

        REQUIRE(controller.setInput(input));
        REQUIRE(controller.advance());

        const auto& output = controller.getOutput();
        REQUIRE(output(0) == Approx(currentLimit).margin(1e-6));
    }

    SECTION("Linear falloff region - medium velocity")
    {
        PositionToJointControllerInput input;
        input.referencePosition = Eigen::VectorXd::Ones(1) * 10.0; // Large error
        input.feedbackPosition = Eigen::VectorXd::Zero(1);
        input.feedbackVelocity = Eigen::VectorXd::Ones(1) * 10.0; // Between rated and no-load

        REQUIRE(controller.setInput(input));
        REQUIRE(controller.advance());

        const auto& output = controller.getOutput();
        // Expected: linear interpolation between currentLimit at ratedSpeed and 0 at noLoadSpeed
        // slope = -currentLimit / (noLoadSpeed - ratedSpeed) = -10.0 / (15.0 - 5.0) = -1.0
        // limit = slope * velocity + intercept = -1.0 * 10.0 + 15.0 = 5.0
        REQUIRE(output(0) == Approx(5.0).margin(1e-6));
    }

    SECTION("No-load region - high velocity")
    {
        PositionToJointControllerInput input;
        input.referencePosition = Eigen::VectorXd::Ones(1) * 10.0; // Large error
        input.feedbackPosition = Eigen::VectorXd::Zero(1);
        input.feedbackVelocity = Eigen::VectorXd::Ones(1) * 20.0; // Above no-load speed

        REQUIRE(controller.setInput(input));
        REQUIRE(controller.advance());

        const auto& output = controller.getOutput();
        REQUIRE(output(0) == Approx(0.0).margin(1e-10));
    }

    SECTION("Negative velocity uses absolute value")
    {
        PositionToJointControllerInput input;
        input.referencePosition = Eigen::VectorXd::Ones(1) * 10.0; // Large error
        input.feedbackPosition = Eigen::VectorXd::Zero(1);
        input.feedbackVelocity = Eigen::VectorXd::Ones(1) * -10.0; // Negative velocity

        REQUIRE(controller.setInput(input));
        REQUIRE(controller.advance());

        const auto& output = controller.getOutput();
        // Same limit as positive velocity
        REQUIRE(output(0) == Approx(5.0).margin(1e-6));
    }
}

TEST_CASE("PositionToCurrentController - Input Validation")
{
    std::vector<std::string> joints{"joint_1", "joint_2"};
    auto ptr = createBasicParameterHandler(joints);

    PositionToCurrentController controller;
    REQUIRE(controller.initialize(ptr));

    SECTION("Correct input size")
    {
        PositionToJointControllerInput input;
        input.referencePosition = Eigen::VectorXd::Zero(2);
        input.feedbackPosition = Eigen::VectorXd::Zero(2);
        input.feedbackVelocity = Eigen::VectorXd::Zero(2);

        REQUIRE(controller.setInput(input));
    }

    SECTION("Incorrect reference position size")
    {
        PositionToJointControllerInput input;
        input.referencePosition = Eigen::VectorXd::Zero(1); // Wrong size
        input.feedbackPosition = Eigen::VectorXd::Zero(2);
        input.feedbackVelocity = Eigen::VectorXd::Zero(2);

        REQUIRE_FALSE(controller.setInput(input));
    }

    SECTION("Incorrect feedback position size")
    {
        PositionToJointControllerInput input;
        input.referencePosition = Eigen::VectorXd::Zero(2);
        input.feedbackPosition = Eigen::VectorXd::Zero(3); // Wrong size
        input.feedbackVelocity = Eigen::VectorXd::Zero(2);

        REQUIRE_FALSE(controller.setInput(input));
    }

    SECTION("Incorrect feedback velocity size")
    {
        PositionToJointControllerInput input;
        input.referencePosition = Eigen::VectorXd::Zero(2);
        input.feedbackPosition = Eigen::VectorXd::Zero(2);
        input.feedbackVelocity = Eigen::VectorXd::Zero(1); // Wrong size

        REQUIRE_FALSE(controller.setInput(input));
    }
}

TEST_CASE("PositionToCurrentController - Multi-Joint Configuration")
{
    std::vector<std::string> joints{"joint_1", "joint_2", "joint_3"};
    constexpr double kp1 = 10.0, kp2 = 20.0, kp3 = 30.0;
    constexpr double kd1 = 1.0, kd2 = 2.0, kd3 = 3.0;
    constexpr double gear1 = 100.0, gear2 = 150.0, gear3 = 200.0;
    constexpr double kTau1 = 0.1, kTau2 = 0.2, kTau3 = 0.3;

    auto ptr = std::make_shared<BipedalLocomotion::ParametersHandler::StdImplementation>();
    ptr->setParameter("joints_list", joints);

    auto ptrKp = std::make_shared<BipedalLocomotion::ParametersHandler::StdImplementation>();
    auto ptrKd = std::make_shared<BipedalLocomotion::ParametersHandler::StdImplementation>();
    auto ptrGear = std::make_shared<BipedalLocomotion::ParametersHandler::StdImplementation>();
    auto ptrKTau = std::make_shared<BipedalLocomotion::ParametersHandler::StdImplementation>();

    ptrKp->setParameter("joint_1", kp1);
    ptrKp->setParameter("joint_2", kp2);
    ptrKp->setParameter("joint_3", kp3);

    ptrKd->setParameter("joint_1", kd1);
    ptrKd->setParameter("joint_2", kd2);
    ptrKd->setParameter("joint_3", kd3);

    ptrGear->setParameter("joint_1", gear1);
    ptrGear->setParameter("joint_2", gear2);
    ptrGear->setParameter("joint_3", gear3);

    ptrKTau->setParameter("joint_1", kTau1);
    ptrKTau->setParameter("joint_2", kTau2);
    ptrKTau->setParameter("joint_3", kTau3);

    ptr->setGroup("kp", ptrKp);
    ptr->setGroup("kd", ptrKd);
    ptr->setGroup("gearbox", ptrGear);
    ptr->setGroup("k_tau", ptrKTau);

    PositionToCurrentController controller;
    REQUIRE(controller.initialize(ptr));

    SECTION("Different gains produce different outputs")
    {
        PositionToJointControllerInput input;
        input.referencePosition = Eigen::VectorXd::Ones(3);
        input.feedbackPosition = Eigen::VectorXd::Zero(3);
        input.feedbackVelocity = Eigen::VectorXd::Zero(3);

        REQUIRE(controller.setInput(input));
        REQUIRE(controller.advance());

        const auto& output = controller.getOutput();
        REQUIRE(output.size() == 3);

        // Each joint should have different output due to different parameters
        double expected1 = kp1 / (gear1 * kTau1);
        double expected2 = kp2 / (gear2 * kTau2);
        double expected3 = kp3 / (gear3 * kTau3);

        REQUIRE(output(0) == Approx(expected1).epsilon(1e-6));
        REQUIRE(output(1) == Approx(expected2).epsilon(1e-6));
        REQUIRE(output(2) == Approx(expected3).epsilon(1e-6));
    }
}

TEST_CASE("PositionToCurrentController - Edge Cases")
{
    std::vector<std::string> joints{"joint_1"};

    SECTION("Invalid current limit (negative)")
    {
        auto ptr = createBasicParameterHandler(joints);
        auto ptrLimit = std::make_shared<BipedalLocomotion::ParametersHandler::StdImplementation>();
        ptrLimit->setParameter("joint_1", -1.0); // Negative limit
        ptr->setGroup("current_limit", ptrLimit);

        PositionToCurrentController controller;
        REQUIRE_FALSE(controller.initialize(ptr));
    }

    SECTION("Output validity before and after advance")
    {
        auto ptr = createBasicParameterHandler(joints);
        PositionToCurrentController controller;
        REQUIRE(controller.initialize(ptr));

        // Output should be invalid before advance
        REQUIRE_FALSE(controller.isOutputValid());

        PositionToJointControllerInput input;
        input.referencePosition = Eigen::VectorXd::Zero(1);
        input.feedbackPosition = Eigen::VectorXd::Zero(1);
        input.feedbackVelocity = Eigen::VectorXd::Zero(1);

        REQUIRE(controller.setInput(input));
        REQUIRE(controller.advance());

        // Output should be valid after advance
        REQUIRE(controller.isOutputValid());
    }

    SECTION("Zero velocity friction compensation")
    {
        auto ptr = createBasicParameterHandler(joints);
        auto ptrFriction
            = std::make_shared<BipedalLocomotion::ParametersHandler::StdImplementation>();
        ptrFriction->setParameter("joint_1", 1.0);
        ptr->setGroup("coulomb_friction", ptrFriction);

        PositionToCurrentController controller;
        REQUIRE(controller.initialize(ptr));

        PositionToJointControllerInput input;
        input.referencePosition = Eigen::VectorXd::Zero(1);
        input.feedbackPosition = Eigen::VectorXd::Zero(1);
        input.feedbackVelocity = Eigen::VectorXd::Zero(1); // Zero velocity

        REQUIRE(controller.setInput(input));
        REQUIRE(controller.advance());

        const auto& output = controller.getOutput();
        // With zero velocity, sign(0) = 0, so friction term should be zero
        REQUIRE(output(0) == Approx(0.0).margin(1e-10));
    }
}
