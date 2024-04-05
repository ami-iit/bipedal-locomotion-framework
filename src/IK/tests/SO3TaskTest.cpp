/**
 * @file SO3TaskTest.cpp
 * @authors Giulio Romualdi
 * @copyright 2021 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the BSD-3-Clause license.
 */

// Catch2
#include <catch2/catch_test_macros.hpp>

// BipedalLocomotion
#include <BipedalLocomotion/Conversions/ManifConversions.h>
#include <BipedalLocomotion/ParametersHandler/StdImplementation.h>
#include <BipedalLocomotion/System/VariablesHandler.h>

#include <BipedalLocomotion/IK/SO3Task.h>

#include <iDynTree/EigenHelpers.h>
#include <iDynTree/ModelTestUtils.h>

using namespace BipedalLocomotion::ParametersHandler;
using namespace BipedalLocomotion::System;
using namespace BipedalLocomotion::IK;

TEST_CASE("SO3 Task")
{
    constexpr double kp = 1.0;
    constexpr auto robotVelocity = "robotVelocity";


    auto kinDyn = std::make_shared<iDynTree::KinDynComputations>();
    auto parameterHandler = std::make_shared<StdImplementation>();

    parameterHandler->setParameter("robot_velocity_variable_name",
                                   robotVelocity);

    parameterHandler->setParameter("kp_angular", kp);

    // set the velocity representation
    REQUIRE(kinDyn->setFrameVelocityRepresentation(iDynTree::FrameVelocityRepresentation::MIXED_REPRESENTATION));

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

            const std::string controlledFrame = model.getFrameName(numberOfJoints);
            parameterHandler->setParameter("frame_name", controlledFrame);

            SO3Task task;
            REQUIRE(task.setKinDyn(kinDyn));
            REQUIRE(task.initialize(parameterHandler));
            REQUIRE(task.setVariablesHandler(variablesHandler));

            const auto desiredRotation = manif::SO3d::Random();
            const auto desiredVelocity = manif::SO3d::Tangent::Random();

            REQUIRE(task.setSetPoint(desiredRotation, desiredVelocity));

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
                    .isApprox(jacobian.bottomRows<3>()));

            // check the vector b
            LieGroupControllers::ProportionalControllerSO3d SO3Controller;
            SO3Controller.setGains(kp);

            SO3Controller.setFeedForward(desiredVelocity);
            SO3Controller.setDesiredState(desiredRotation);

            SO3Controller.setState(BipedalLocomotion::Conversions::toManifRot(
                kinDyn->getWorldTransform(controlledFrame).getRotation()));

            SO3Controller.computeControlLaw();

            Eigen::Vector3d expectedB = SO3Controller.getControl().coeffs();

            REQUIRE(b.isApprox(expectedB));
        }
    }
}
