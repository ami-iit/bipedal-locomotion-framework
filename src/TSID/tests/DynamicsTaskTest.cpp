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
#include <BipedalLocomotion/TSID/BaseDynamicsTask.h>
#include <BipedalLocomotion/TSID/JointDynamicsTask.h>

#include <iDynTree/EigenHelpers.h>
#include <iDynTree/ModelTestUtils.h>

#include <Eigen/Dense>

using namespace BipedalLocomotion::ParametersHandler;
using namespace BipedalLocomotion::System;
using namespace BipedalLocomotion::TSID;

TEST_CASE("Joints and Base dynamics tasks")
{
    const std::string robotAcceleration = "robotAcceleration";
    const std::string jointsTorque = "jointsTorque";

    auto kinDyn = std::make_shared<iDynTree::KinDynComputations>();
    auto parameterHandler = std::make_shared<StdImplementation>();

    parameterHandler->setParameter("robot_acceleration_variable_name",
                                   robotAcceleration);
    parameterHandler->setParameter("joint_torques_variable_name", jointsTorque);

    parameterHandler->setParameter("max_number_of_contacts", 1);

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
            variablesHandler.addVariable(robotAcceleration, model.getNrOfDOFs() + 6);
            variablesHandler.addVariable(jointsTorque, model.getNrOfDOFs());
            variablesHandler.addVariable("dummy2", 15);
            variablesHandler.addVariable("contact0", 6);
            variablesHandler.addVariable("dummy3", 4);


            const std::string frameName = model.getFrameName(numberOfJoints);
            auto contactGroup = std::make_shared<StdImplementation>();
            contactGroup->setParameter("variable_name", "contact0");
            contactGroup->setParameter("frame_name", frameName);
            REQUIRE(parameterHandler->setGroup("CONTACT_0", contactGroup));

            iDynTree::MatrixDynSize massMatrix(model.getNrOfDOFs() + 6, model.getNrOfDOFs() + 6);
            REQUIRE(kinDyn->getFreeFloatingMassMatrix(massMatrix));

            iDynTree::MatrixDynSize jacobian(6, model.getNrOfDOFs() + 6);
            REQUIRE(kinDyn->getFrameFreeFloatingJacobian(frameName, jacobian));

            Eigen::VectorXd generalizedBiasForces(model.getNrOfDOFs() + 6);
            REQUIRE(kinDyn->generalizedBiasForces(generalizedBiasForces));

            DYNAMIC_SECTION("Model with " << numberOfJoints << " joints - Base dynamics test")
            {
                BaseDynamicsTask task;
                REQUIRE(task.setKinDyn(kinDyn));
                REQUIRE(task.initialize(parameterHandler));
                REQUIRE(task.setVariablesHandler(variablesHandler));
                REQUIRE(task.update());


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

                REQUIRE(A.middleCols(variablesHandler.getVariable("dummy3").offset,
                                     variablesHandler.getVariable("dummy3").size)
                            .isZero());

                REQUIRE(A.middleCols(variablesHandler.getVariable(jointsTorque).offset,
                                     variablesHandler.getVariable(jointsTorque).size)
                            .isZero());

                REQUIRE(A.middleCols(variablesHandler.getVariable(robotAcceleration).offset,
                                     variablesHandler.getVariable(robotAcceleration).size)
                            .isApprox(-iDynTree::toEigen(massMatrix).topRows(6)));

                REQUIRE(A.middleCols(variablesHandler.getVariable("contact0").offset,
                                     variablesHandler.getVariable("contact0").size)
                            .isApprox(iDynTree::toEigen(jacobian).transpose().topRows(6)));

                // check the vector b
                REQUIRE(b.isApprox(generalizedBiasForces.head(6)));
            }

            DYNAMIC_SECTION("Model with " << numberOfJoints << " joints - joints dynamics test")
            {
                JointDynamicsTask task;
                REQUIRE(task.setKinDyn(kinDyn));
                REQUIRE(task.initialize(parameterHandler));
                REQUIRE(task.setVariablesHandler(variablesHandler));
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

                REQUIRE(A.middleCols(variablesHandler.getVariable("dummy3").offset,
                                     variablesHandler.getVariable("dummy3").size)
                            .isZero());

                REQUIRE(A.middleCols(variablesHandler.getVariable(jointsTorque).offset,
                                     variablesHandler.getVariable(jointsTorque).size)
                            .isIdentity());

                REQUIRE(A.middleCols(variablesHandler.getVariable(robotAcceleration).offset,
                                 variablesHandler.getVariable(robotAcceleration).size)
                        .isApprox(-iDynTree::toEigen(massMatrix).bottomRows(model.getNrOfDOFs())));

                REQUIRE(A.middleCols(variablesHandler.getVariable("contact0").offset,
                                     variablesHandler.getVariable("contact0").size)
                            .isApprox(iDynTree::toEigen(jacobian).transpose().bottomRows(model.getNrOfDOFs())));

                // check the vector b
                REQUIRE(b.isApprox(generalizedBiasForces.tail(model.getNrOfDOFs())));
            }
        }
    }
}
