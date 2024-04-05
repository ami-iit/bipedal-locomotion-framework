/**
 * @file JointsTrackingTaskTest.cpp
 * @authors Giulio Romualdi
 * @copyright 2021 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the BSD-3-Clause license.
 */

// Catch2
#include <catch2/catch_test_macros.hpp>

// BipedalLocomotion
#include <BipedalLocomotion/ParametersHandler/StdImplementation.h>
#include <BipedalLocomotion/System/VariablesHandler.h>
#include <BipedalLocomotion/TSID/JointTrackingTask.h>

#include <iDynTree/EigenHelpers.h>
#include <iDynTree/ModelTestUtils.h>

#include <Eigen/Dense>

using namespace BipedalLocomotion::ParametersHandler;
using namespace BipedalLocomotion::System;
using namespace BipedalLocomotion::TSID;

TEST_CASE("Joint Regularization task")
{
    const std::string robotAcceleration = "robotAcceleration";

    Eigen::VectorXd kp, kd;

    auto kinDyn = std::make_shared<iDynTree::KinDynComputations>();
    auto parameterHandler = std::make_shared<StdImplementation>();

    parameterHandler->setParameter("robot_acceleration_variable_name",
                                   robotAcceleration);

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
            Eigen::VectorXd feedforwardDesiredJointAcc = Eigen::VectorXd::Random(model.getNrOfDOFs());

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


            kp.resize(model.getNrOfDOFs());
            kp.setOnes();
            kd.resize(model.getNrOfDOFs());
            kd.setOnes();
            parameterHandler->setParameter("kp", kp);
            parameterHandler->setParameter("kd", kd);


            JointTrackingTask task;
            REQUIRE(task.setKinDyn(kinDyn));
            REQUIRE(task.initialize(parameterHandler));
            REQUIRE(task.setVariablesHandler(variablesHandler));

            REQUIRE(task.setSetPoint(Eigen::VectorXd::Zero(model.getNrOfDOFs()),
                                     Eigen::VectorXd::Zero(model.getNrOfDOFs()),
                                     feedforwardDesiredJointAcc));

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

            REQUIRE(A.middleCols(variablesHandler.getVariable(robotAcceleration).offset + 6,
                                 model.getNrOfDOFs())
                        .isIdentity());

            // check the vector b
            Eigen::VectorXd expectedB(model.getNrOfDOFs());
            expectedB = -(kp.asDiagonal() * iDynTree::toEigen(jointsPos))
                        - (kd.asDiagonal() * iDynTree::toEigen(jointsVel))
                        + feedforwardDesiredJointAcc;
            REQUIRE(b.isApprox(expectedB));
        }
    }
}
