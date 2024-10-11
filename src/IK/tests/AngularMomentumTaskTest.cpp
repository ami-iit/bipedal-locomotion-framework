/**
 * @file AngularMomentumTaskTest.cpp
 * @authors Giulio Romualdi
 * @copyright 2021 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the BSD-3-Clause license.
 */

// Catch2
#include <catch2/catch_test_macros.hpp>

// BipedalLocomotion
#include <BipedalLocomotion/IK/AngularMomentumTask.h>
#include <BipedalLocomotion/ParametersHandler/StdImplementation.h>
#include <BipedalLocomotion/System/VariablesHandler.h>

#include <iDynTree/EigenHelpers.h>
#include <iDynTree/ModelTestUtils.h>

using namespace BipedalLocomotion::ParametersHandler;
using namespace BipedalLocomotion::System;
using namespace BipedalLocomotion::IK;

TEST_CASE("AngularMomentum Task")
{
    const std::string robotVelocity = "robotVelocity";

    auto kinDyn = std::make_shared<iDynTree::KinDynComputations>();
    auto parameterHandler = std::make_shared<StdImplementation>();

    parameterHandler->setParameter("robot_velocity_variable_name",
                                   robotVelocity);


    // set the velocity representation
    REQUIRE(kinDyn->setFrameVelocityRepresentation(
        iDynTree::FrameVelocityRepresentation::MIXED_REPRESENTATION));

    const Eigen::Vector3d desiredAngularMomentum = Eigen::Vector3d::Random();

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

        DYNAMIC_SECTION("Model with " << numberOfJoints << " joints - [All DoF]")
        {
            AngularMomentumTask task;
            REQUIRE(task.setKinDyn(kinDyn));
            REQUIRE(task.initialize(parameterHandler));
            REQUIRE(task.setVariablesHandler(variablesHandler));
            REQUIRE(task.size() == 3);


            REQUIRE(task.setSetPoint(desiredAngularMomentum));

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

            Eigen::MatrixXd centroidalMomentumMatrix(6, model.getNrOfDOFs() + 6);
            REQUIRE(kinDyn->getCentroidalTotalMomentumJacobian(centroidalMomentumMatrix));

            REQUIRE(A.middleCols(variablesHandler.getVariable(robotVelocity).offset,
                                 variablesHandler.getVariable(robotVelocity).size)
                    .isApprox(centroidalMomentumMatrix.bottomRows<3>()));

            // check the vector b
            REQUIRE(b.isApprox(desiredAngularMomentum));
        }


        DYNAMIC_SECTION("Model with " << numberOfJoints << " joints - [mask]")
        {
            const auto mask = std::vector<bool>{true, false, true};
            const auto DoFs = std::count(mask.begin(), mask.end(), true);

            parameterHandler->setParameter("mask", mask);

            AngularMomentumTask task;
            REQUIRE(task.setKinDyn(kinDyn));
            REQUIRE(task.initialize(parameterHandler));
            REQUIRE(task.setVariablesHandler(variablesHandler));
            REQUIRE(task.size() == DoFs);

            REQUIRE(task.setSetPoint(desiredAngularMomentum));

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

            Eigen::MatrixXd centroidalMomentumMatrix(6, model.getNrOfDOFs() + 6);
            REQUIRE(kinDyn->getCentroidalTotalMomentumJacobian(centroidalMomentumMatrix));

            // extract the
            Eigen::MatrixXd centroidalMomentumMatrixWithMask(DoFs, model.getNrOfDOFs() + 6);
            std::size_t index = 0;
            for (std::size_t i = 0; i < 3; i++)
            {
                if (mask[i])
                {
                    centroidalMomentumMatrixWithMask.row(index)
                        = centroidalMomentumMatrix.row(i + 3);
                    index++;
                }
            }

            REQUIRE(A.middleCols(variablesHandler.getVariable(robotVelocity).offset,
                                 variablesHandler.getVariable(robotVelocity).size)
                    .isApprox(centroidalMomentumMatrixWithMask));


            Eigen::VectorXd expectedB(DoFs);
            index = 0;
            for(int i = 0; i < 3; i++)
            {
                if(mask[i])
                {
                    expectedB(index) = desiredAngularMomentum(i);
                    index++;
                }
            }

            REQUIRE(b.isApprox(expectedB));
        }
    }
}
