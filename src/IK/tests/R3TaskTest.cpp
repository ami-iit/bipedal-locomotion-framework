/**
 * @file R3TaskTest.cpp
 * @authors Giulio Romualdi
 * @copyright 2022 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the BSD-3-Clause license.
 */

// Catch2
#include <catch2/catch_test_macros.hpp>

// BipedalLocomotion
#include <BipedalLocomotion/Conversions/ManifConversions.h>
#include <BipedalLocomotion/ParametersHandler/StdImplementation.h>
#include <BipedalLocomotion/System/VariablesHandler.h>
#include <BipedalLocomotion/IK/R3Task.h>

#include <iDynTree/EigenHelpers.h>
#include <iDynTree/ModelTestUtils.h>

using namespace BipedalLocomotion::ParametersHandler;
using namespace BipedalLocomotion::System;
using namespace BipedalLocomotion::IK;

TEST_CASE("SE3 Task")
{
    constexpr double kp = 1.0;
    const std::string robotVelocity = "robotVelocity";


    auto kinDyn = std::make_shared<iDynTree::KinDynComputations>();
    auto parameterHandler = std::make_shared<StdImplementation>();

    parameterHandler->setParameter("robot_velocity_variable_name",
                                   robotVelocity);

    parameterHandler->setParameter("kp_linear", kp);

    // set the velocity representation
    REQUIRE(kinDyn->setFrameVelocityRepresentation(
        iDynTree::FrameVelocityRepresentation::MIXED_REPRESENTATION));

    for (std::size_t numberOfJoints = 6; numberOfJoints < 40; numberOfJoints += 15)
    {
        // create the model
        const iDynTree::Model model = iDynTree::getRandomModel(numberOfJoints);
        REQUIRE(kinDyn->loadRobotModel(model));

        const auto worldBasePos = iDynTree::getRandomTransform();
        const auto baseVel = iDynTree::getRandomTwist();
        iDynTree::VectorDynSize jointsPos(model.getNrOfDOFs());
        iDynTree::VectorDynSize jointsVel(model.getNrOfDOFs());
        iDynTree::Vector3 gravity;

        for (auto& joint : jointsPos)
        {
            joint = iDynTree::getRandomDouble();
        }

        for (auto& joint : jointsVel)
        {
            joint = iDynTree::getRandomDouble();
        }

        for (auto& element : gravity)
        {
            element = iDynTree::getRandomDouble();
        }

        REQUIRE(kinDyn->setRobotState(worldBasePos, jointsPos, baseVel, jointsVel, gravity));

        // Instantiate the handler
        VariablesHandler variablesHandler;
        variablesHandler.addVariable("dummy1", 10);
        variablesHandler.addVariable(robotVelocity, model.getNrOfDOFs() + 6);
        variablesHandler.addVariable("dummy2", 15);

        const std::string controlledFrame = model.getFrameName(numberOfJoints);
        parameterHandler->setParameter("frame_name", controlledFrame);

        DYNAMIC_SECTION("Model with " << numberOfJoints << " joints - [All DoF]")
        {
            R3Task task;
            REQUIRE(task.setKinDyn(kinDyn));
            REQUIRE(task.initialize(parameterHandler));
            REQUIRE(task.setVariablesHandler(variablesHandler));
            REQUIRE(task.size() == 3);

            const auto desiredPosition = manif::R3d::Random();
            const auto desiredVelocity = manif::R3d::Tangent::Random();

            REQUIRE(task.setSetPoint(desiredPosition.coeffs(), desiredVelocity.coeffs()));

            REQUIRE(task.update());
            REQUIRE(task.isValid());

            // get A and b
            Eigen::Ref<const Eigen::MatrixXd> A = task.getA();
            Eigen::Ref<const Eigen::VectorXd> b = task.getB();

            // check the matrix A
            REQUIRE(A.middleCols(variablesHandler.getVariable("dummy1").offset,
                                 variablesHandler.getVariable("dummy1").size)
                    .isZero());

            REQUIRE(A.middleCols(variablesHandler.getVariable("dummy2").offset,
                                 variablesHandler.getVariable("dummy2").size)
                    .isZero());

            Eigen::MatrixXd jacobian(6, model.getNrOfDOFs() + 6);
            REQUIRE(kinDyn->getFrameFreeFloatingJacobian(controlledFrame, jacobian));

            REQUIRE(A.middleCols(variablesHandler.getVariable(robotVelocity).offset,
                                 variablesHandler.getVariable(robotVelocity).size)
                    .isApprox(jacobian.topRows<3>()));

            // check the vector b
            LieGroupControllers::ProportionalControllerR3d R3Controller;
            R3Controller.setGains(kp);

            R3Controller.setFeedForward(desiredVelocity);
            R3Controller.setDesiredState(desiredPosition);

            R3Controller.setState(
                iDynTree::toEigen(kinDyn->getWorldTransform(controlledFrame).getPosition()));

            R3Controller.computeControlLaw();

            REQUIRE(b.isApprox(R3Controller.getControl().coeffs()));
        }


        DYNAMIC_SECTION("Model with " << numberOfJoints << " joints - [mask]")
        {
            const auto mask = std::vector<bool>{true, false, true};
            const auto DoFs = std::count(mask.begin(), mask.end(), true);

            parameterHandler->setParameter("mask", mask);

            R3Task task;
            REQUIRE(task.setKinDyn(kinDyn));
            REQUIRE(task.initialize(parameterHandler));
            REQUIRE(task.setVariablesHandler(variablesHandler));
            REQUIRE(task.size() == DoFs);

            const auto desiredPosition = manif::R3d::Random();
            const auto desiredVelocity = manif::R3d::Tangent::Random();

            REQUIRE(task.setSetPoint(desiredPosition.coeffs(), desiredVelocity.coeffs()));

            REQUIRE(task.update());
            REQUIRE(task.isValid());

            // get A and b
            Eigen::Ref<const Eigen::MatrixXd> A = task.getA();
            Eigen::Ref<const Eigen::VectorXd> b = task.getB();

            // check the matrix A
            REQUIRE(A.middleCols(variablesHandler.getVariable("dummy1").offset,
                                 variablesHandler.getVariable("dummy1").size)
                    .isZero());

            REQUIRE(A.middleCols(variablesHandler.getVariable("dummy2").offset,
                                 variablesHandler.getVariable("dummy2").size)
                    .isZero());

            Eigen::MatrixXd jacobian(6, model.getNrOfDOFs() + 6);
            REQUIRE(kinDyn->getFrameFreeFloatingJacobian(controlledFrame, jacobian));

            // extract the
            Eigen::MatrixXd jacobianWithMask(DoFs, model.getNrOfDOFs() + 6);
            std::size_t index = 0;
            for (std::size_t i = 0; i < 3; i++)
            {
                if (mask[i])
                {
                    jacobianWithMask.row(index) = jacobian.row(i);
                    index++;
                }
            }

            REQUIRE(A.middleCols(variablesHandler.getVariable(robotVelocity).offset,
                                 variablesHandler.getVariable(robotVelocity).size)
                    .isApprox(jacobianWithMask));

            // check the vector b
            LieGroupControllers::ProportionalControllerR3d R3Controller;
            R3Controller.setGains(kp);

            R3Controller.setFeedForward(desiredVelocity);
            R3Controller.setDesiredState(desiredPosition);

            R3Controller.setState(
                iDynTree::toEigen(kinDyn->getWorldTransform(controlledFrame).getPosition()));

            R3Controller.computeControlLaw();

            Eigen::VectorXd expectedB(DoFs);
            index = 0;
            for(int i = 0; i < 3; i++)
            {
                if(mask[i])
                {
                    expectedB(index) = R3Controller.getControl().coeffs()[i];
                    index++;
                }
            }

            REQUIRE(b.isApprox(expectedB));
        }
    }
}
