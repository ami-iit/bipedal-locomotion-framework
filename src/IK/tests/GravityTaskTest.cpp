/**
 * @file JointTrackingTaskTest.cpp
 * @authors Giulio Romualdi
 * @copyright 2021 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the BSD-3-Clause license.
 */

// Catch2
#include <catch2/catch_test_macros.hpp>

// BipedalLocomotion
#include <BipedalLocomotion/IK/GravityTask.h>
#include <BipedalLocomotion/ParametersHandler/StdImplementation.h>
#include <BipedalLocomotion/System/VariablesHandler.h>

#include <iDynTree/EigenHelpers.h>
#include <iDynTree/ModelTestUtils.h>

#include <Eigen/Dense>

using namespace BipedalLocomotion::ParametersHandler;
using namespace BipedalLocomotion::System;
using namespace BipedalLocomotion::IK;

TEST_CASE("Distance task")
{
    const std::string robotVelocity = "robotVelocity";

    Eigen::VectorXd kp;

    auto kinDyn = std::make_shared<iDynTree::KinDynComputations>();
    auto parameterHandler = std::make_shared<StdImplementation>();

    parameterHandler->setParameter("robot_velocity_variable_name", robotVelocity);

    // set the velocity representation
    REQUIRE(kinDyn->setFrameVelocityRepresentation(
        iDynTree::FrameVelocityRepresentation::MIXED_REPRESENTATION));

    for (std::size_t numberOfJoints = 6; numberOfJoints < 40; numberOfJoints += 15)
    {
        DYNAMIC_SECTION("Model with " << numberOfJoints << " joints")
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
            double kp = 2.0;
            parameterHandler->setParameter("kp", kp);

            std::string targetName = iDynTree::getRandomLinkOfModel(model);
            parameterHandler->setParameter("target_frame_name", targetName);

            GravityTask task;
            REQUIRE(task.setKinDyn(kinDyn));
            REQUIRE(task.initialize(parameterHandler));
            REQUIRE(task.setVariablesHandler(variablesHandler));

            Eigen::Vector3d desiredDirection({1.0, 2.0, 3.0});
            desiredDirection.normalize();
            Eigen::Vector3d feedforward({0.1, 0.2, 0.3});

            REQUIRE(task.setSetPoint(desiredDirection, feedforward));

            REQUIRE(task.update());
            REQUIRE(task.isValid());

            // get A and b
            Eigen::Ref<const Eigen::MatrixXd> A = task.getA();
            Eigen::Ref<const Eigen::VectorXd> b = task.getB();

            Eigen::Matrix3d targetRotation;
            Eigen::MatrixXd jacobian;
            jacobian.setZero(6, 6 + model.getNrOfDOFs());

            targetRotation = iDynTree::toEigen(kinDyn->getWorldTransform(targetName).getRotation());
            kinDyn->getFrameFreeFloatingJacobian(targetName, jacobian);

            Eigen::MatrixXd expectedA = jacobian.middleRows<2>(3);

            // check the matrix A
            REQUIRE(A.middleCols(variablesHandler.getVariable("dummy1").offset,
                                 variablesHandler.getVariable("dummy1").size)
                        .isZero());

            REQUIRE(A.middleCols(variablesHandler.getVariable("dummy2").offset,
                                 variablesHandler.getVariable("dummy2").size)
                        .isZero());

            REQUIRE(A.middleCols(variablesHandler.getVariable(robotVelocity).offset,
                                 model.getNrOfDOFs() + 6)
                        .isApprox(expectedA));

            // check the vector b
            Eigen::VectorXd expectedB(task.size());
            Eigen::Vector3d axisAbsolute, feedforwardAbsolute;
            axisAbsolute = targetRotation * desiredDirection;
            feedforwardAbsolute = targetRotation * feedforward;
            expectedB(0) = kp * axisAbsolute(1) + feedforwardAbsolute(0);
            expectedB(1) = -kp * axisAbsolute(0) + feedforwardAbsolute(1);
            REQUIRE(b.isApprox(expectedB));
        }
    }
}
