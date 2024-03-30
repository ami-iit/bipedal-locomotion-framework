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
#include <BipedalLocomotion/TSID/R3Task.h>
#include <BipedalLocomotion/ParametersHandler/StdImplementation.h>
#include <BipedalLocomotion/System/VariablesHandler.h>

#include <iDynTree/Core/EigenHelpers.h>
#include <iDynTree/Model/ModelTestUtils.h>

using namespace BipedalLocomotion::ParametersHandler;
using namespace BipedalLocomotion::System;
using namespace BipedalLocomotion::TSID;

TEST_CASE("SE3 Task")
{
    constexpr double kp = 1.0;
    constexpr double kd = 2 * std::sqrt(kp);
    const std::string robotAcceleration = "robotAcceleration";

    auto kinDyn = std::make_shared<iDynTree::KinDynComputations>();
    auto parameterHandler = std::make_shared<StdImplementation>();

    parameterHandler->setParameter("robot_acceleration_variable_name", robotAcceleration);

    parameterHandler->setParameter("kp_linear", kp);
    parameterHandler->setParameter("kd_linear", kd);

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
        variablesHandler.addVariable(robotAcceleration, model.getNrOfDOFs() + 6);
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
            const auto desiredAcceleration = manif::R3d::Tangent::Random();

            REQUIRE(task.setSetPoint(desiredPosition.coeffs(),
                                     desiredVelocity.coeffs(),
                                     desiredAcceleration.coeffs()));

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

            REQUIRE(A.middleCols(variablesHandler.getVariable(robotAcceleration).offset,
                                 variablesHandler.getVariable(robotAcceleration).size)
                        .isApprox(jacobian.topRows<3>()));

            // check the vector b
            LieGroupControllers::ProportionalDerivativeControllerR3d R3Controller;
            R3Controller.setGains(kp, kd);

            R3Controller.setFeedForward(desiredAcceleration);
            R3Controller.setDesiredState(desiredPosition, desiredVelocity);

            R3Controller.setState( //
                iDynTree::toEigen(kinDyn->getWorldTransform(controlledFrame).getPosition()),
                iDynTree::toEigen(kinDyn->getFrameVel(controlledFrame).getLinearVec3()));

            R3Controller.computeControlLaw();

            const Eigen::Vector3d expectedBFull
                = R3Controller.getControl().coeffs() //
                  - toEigen(kinDyn->getFrameBiasAcc(controlledFrame)).head<3>();

            REQUIRE(b.isApprox(expectedBFull));
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
            const auto desiredAcceleration = manif::R3d::Tangent::Random();

            REQUIRE(task.setSetPoint(desiredPosition.coeffs(),
                                     desiredVelocity.coeffs(),
                                     desiredAcceleration.coeffs()));

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

            REQUIRE(A.middleCols(variablesHandler.getVariable(robotAcceleration).offset,
                                 variablesHandler.getVariable(robotAcceleration).size)
                        .isApprox(jacobianWithMask));

            // check the vector b
            LieGroupControllers::ProportionalDerivativeControllerR3d R3Controller;
            R3Controller.setGains(kp, kd);

            R3Controller.setFeedForward(desiredAcceleration);
            R3Controller.setDesiredState(desiredPosition, desiredVelocity);

            R3Controller.setState( //
                iDynTree::toEigen(kinDyn->getWorldTransform(controlledFrame).getPosition()),
                iDynTree::toEigen(kinDyn->getFrameVel(controlledFrame).getLinearVec3()));

            R3Controller.computeControlLaw();

            Eigen::VectorXd expectedB(DoFs);
            const Eigen::Vector3d expectedBFull
                = R3Controller.getControl().coeffs() //
                  - toEigen(kinDyn->getFrameBiasAcc(controlledFrame)).head<3>();
            index = 0;
            for (int i = 0; i < 3; i++)
            {
                if (mask[i])
                {
                    expectedB(index) = expectedBFull(i);
                    index++;
                }
            }

            REQUIRE(b.isApprox(expectedB));
        }
    }
}
