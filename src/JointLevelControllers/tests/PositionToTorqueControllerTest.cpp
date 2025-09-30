/**
 * @file PositionToTorqueControllerTest.cpp
 * @authors Giulio Romualdi
 * @copyright 2025 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the BSD-3-Clause license.
 */

// Catch2
#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <BipedalLocomotion/JointLevelControllers/PositionToJointController.h>
#include <BipedalLocomotion/JointLevelControllers/PositionToTorqueController.h>
#include <BipedalLocomotion/ParametersHandler/StdImplementation.h>

using namespace BipedalLocomotion::JointLevelControllers;
using Catch::Approx;

// Helper function to create parameter handler with basic configuration
std::shared_ptr<BipedalLocomotion::ParametersHandler::StdImplementation>
createBasicParameterHandler(const std::vector<std::string>& joints,
                            double kp = 10.0,
                            double kd = 0.0)
{
    auto ptr = std::make_shared<BipedalLocomotion::ParametersHandler::StdImplementation>();
    ptr->setParameter("joints_list", joints);

    auto ptrKp = std::make_shared<BipedalLocomotion::ParametersHandler::StdImplementation>();
    auto ptrKd = std::make_shared<BipedalLocomotion::ParametersHandler::StdImplementation>();
    for (const auto& joint : joints)
    {
        ptrKp->setParameter(joint, kp);
        ptrKd->setParameter(joint, kd);
    }

    ptr->setGroup("kp", ptrKp);
    ptr->setGroup("kd", ptrKd);

    return ptr;
}

TEST_CASE("PositionToTorqueController - Basic Initialization")
{
    std::vector<std::string> joints{"joint_1", "joint_2", "joint_3"};

    SECTION("Valid initialization with required parameters")
    {
        auto ptr = createBasicParameterHandler(joints);
        PositionToTorqueController controller;
        REQUIRE(controller.initialize(ptr));
    }

    SECTION("Missing joints_list parameter")
    {
        auto ptr = std::make_shared<BipedalLocomotion::ParametersHandler::StdImplementation>();
        PositionToTorqueController controller;
        REQUIRE_FALSE(controller.initialize(ptr));
    }

    SECTION("Missing required parameter groups")
    {
        auto ptr = std::make_shared<BipedalLocomotion::ParametersHandler::StdImplementation>();
        ptr->setParameter("joints_list", joints);

        PositionToTorqueController controller;
        REQUIRE_FALSE(controller.initialize(ptr));
    }

    SECTION("Invalid parameter handler (nullptr)")
    {
        std::weak_ptr<const BipedalLocomotion::ParametersHandler::IParametersHandler> nullPtr;
        PositionToTorqueController controller;
        REQUIRE_FALSE(controller.initialize(nullPtr));
    }
}

TEST_CASE("PositionToTorqueController - Basic Position Control")
{
    constexpr double kp = 10.0;
    std::vector<std::string> joints{"joint_1"};

    auto ptr = createBasicParameterHandler(joints, kp /*kp*/, 0.0 /*kd*/);
    PositionToTorqueController controller;
    REQUIRE(controller.initialize(ptr));

    SECTION("Zero position error produces zero torque")
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

    SECTION("Positive position error produces positive torque")
    {
        PositionToJointControllerInput input;
        input.referencePosition = Eigen::VectorXd::Ones(1) * 1.0;
        input.feedbackPosition = Eigen::VectorXd::Zero(1);
        input.feedbackVelocity = Eigen::VectorXd::Zero(1);

        REQUIRE(controller.setInput(input));
        REQUIRE(controller.advance());
        REQUIRE(controller.isOutputValid());

        const auto& output = controller.getOutput();
        REQUIRE(output(0) == Approx(kp * (1.0 - 0.0)).margin(1e-12));
    }

    SECTION("Negative position error produces negative torque")
    {
        PositionToJointControllerInput input;
        input.referencePosition = Eigen::VectorXd::Zero(1);
        input.feedbackPosition = Eigen::VectorXd::Ones(1) * 1.0;
        input.feedbackVelocity = Eigen::VectorXd::Zero(1);

        REQUIRE(controller.setInput(input));
        REQUIRE(controller.advance());

        const auto& output = controller.getOutput();
        REQUIRE(output(0) == Approx(kp * (0.0 - 1.0)).margin(1e-12));
    }

    SECTION("Derivative term reduces torque with positive velocity")
    {
        // Re-initialize controller with non-zero kd and zero kp to isolate D action
        constexpr double kd = 2.0;
        auto ptrD = createBasicParameterHandler(joints, /*kp*/ 0.0, /*kd*/ kd);
        PositionToTorqueController ctrlD;
        REQUIRE(ctrlD.initialize(ptrD));

        PositionToJointControllerInput input;
        input.referencePosition = Eigen::VectorXd::Zero(1); // zero position error
        input.feedbackPosition = Eigen::VectorXd::Zero(1);
        input.feedbackVelocity = Eigen::VectorXd::Ones(1) * 1.5; // rad/s

        REQUIRE(ctrlD.setInput(input));
        REQUIRE(ctrlD.advance());

        const auto& output = ctrlD.getOutput();
        const double expected = (-kd * 1.5);
        REQUIRE(output(0) == Approx(expected).epsilon(1e-12));
    }

    SECTION("Combined P and D actions")
    {
        constexpr double kd = 2.0;
        auto ptrPD = createBasicParameterHandler(joints, /*kp*/ kp, /*kd*/ kd);
        PositionToTorqueController ctrlPD;
        REQUIRE(ctrlPD.initialize(ptrPD));

        PositionToJointControllerInput input;
        input.referencePosition = Eigen::VectorXd::Ones(1) * 1.0; // error = 1.0
        input.feedbackPosition = Eigen::VectorXd::Zero(1);
        input.feedbackVelocity = Eigen::VectorXd::Ones(1) * 0.5; // D term = -kd*0.5

        REQUIRE(ctrlPD.setInput(input));
        REQUIRE(ctrlPD.advance());

        const auto& output = ctrlPD.getOutput();
        const double expected = kp * 1.0 - kd * 0.5; // 10 - 1 = 9
        REQUIRE(output(0) == Approx(expected).epsilon(1e-12));
    }
}

TEST_CASE("PositionToTorqueController - Torque Limiting")
{
    constexpr double kp = 100.0; // High gain to trigger limiting
    constexpr double torqueLimit = 1.0; // Nm
    std::vector<std::string> joints{"joint_1"};

    auto ptr = createBasicParameterHandler(joints, kp, 0.0);

    // Add torque limiting
    auto ptrLimit = std::make_shared<BipedalLocomotion::ParametersHandler::StdImplementation>();
    ptrLimit->setParameter("joint_1", torqueLimit);
    ptr->setGroup("torque_limit", ptrLimit);

    PositionToTorqueController controller;
    REQUIRE(controller.initialize(ptr));

    SECTION("Torque is limited to maximum value")
    {
        PositionToJointControllerInput input;
        input.referencePosition = Eigen::VectorXd::Ones(1) * 10.0; // Large error -> large torque
        input.feedbackPosition = Eigen::VectorXd::Zero(1);
        input.feedbackVelocity = Eigen::VectorXd::Zero(1);

        REQUIRE(controller.setInput(input));
        REQUIRE(controller.advance());

        const auto& output = controller.getOutput();
        REQUIRE(output(0) == Approx(torqueLimit).margin(1e-6));
    }

    SECTION("Torque is limited to minimum value")
    {
        PositionToJointControllerInput input;
        input.referencePosition = Eigen::VectorXd::Zero(1);
        input.feedbackPosition = Eigen::VectorXd::Ones(1) * 10.0; // Large negative error
        input.feedbackVelocity = Eigen::VectorXd::Zero(1);

        REQUIRE(controller.setInput(input));
        REQUIRE(controller.advance());

        const auto& output = controller.getOutput();
        REQUIRE(output(0) == Approx(-torqueLimit).margin(1e-6));
    }
}

TEST_CASE("PositionToTorqueController - Input Validation")
{
    std::vector<std::string> joints{"joint_1", "joint_2"};
    auto ptr = createBasicParameterHandler(joints);

    PositionToTorqueController controller;
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

TEST_CASE("PositionToTorqueController - Multi-Joint Configuration")
{
    std::vector<std::string> joints{"joint_1", "joint_2", "joint_3"};
    constexpr double kp1 = 10.0, kp2 = 20.0, kp3 = 30.0;
    constexpr double kd1 = 1.0, kd2 = 2.0, kd3 = 3.0;

    auto ptr = std::make_shared<BipedalLocomotion::ParametersHandler::StdImplementation>();
    ptr->setParameter("joints_list", joints);

    auto ptrKp = std::make_shared<BipedalLocomotion::ParametersHandler::StdImplementation>();
    auto ptrKd = std::make_shared<BipedalLocomotion::ParametersHandler::StdImplementation>();

    ptrKp->setParameter("joint_1", kp1);
    ptrKp->setParameter("joint_2", kp2);
    ptrKp->setParameter("joint_3", kp3);

    ptrKd->setParameter("joint_1", kd1);
    ptrKd->setParameter("joint_2", kd2);
    ptrKd->setParameter("joint_3", kd3);

    ptr->setGroup("kp", ptrKp);
    ptr->setGroup("kd", ptrKd);

    PositionToTorqueController controller;
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

        double expected1 = kp1;
        double expected2 = kp2;
        double expected3 = kp3;

        REQUIRE(output(0) == Approx(expected1).epsilon(1e-12));
        REQUIRE(output(1) == Approx(expected2).epsilon(1e-12));
        REQUIRE(output(2) == Approx(expected3).epsilon(1e-12));
    }
}

TEST_CASE("PositionToTorqueController - Edge Cases")
{
    std::vector<std::string> joints{"joint_1"};

    SECTION("Invalid torque limit (negative)")
    {
        auto ptr = createBasicParameterHandler(joints);
        auto ptrLimit = std::make_shared<BipedalLocomotion::ParametersHandler::StdImplementation>();
        ptrLimit->setParameter("joint_1", -1.0); // Negative limit
        ptr->setGroup("torque_limit", ptrLimit);

        PositionToTorqueController controller;
        REQUIRE_FALSE(controller.initialize(ptr));
    }

    SECTION("Output validity before and after advance")
    {
        auto ptr = createBasicParameterHandler(joints);
        PositionToTorqueController controller;
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
}
