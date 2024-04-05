/**
 * @file JointsTrackingTaskTest.cpp
 * @authors Giulio Romualdi
 * @copyright 2021 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the BSD-3-Clause license.
 */

// Catch2
#include <catch2/catch_test_macros.hpp>

// BipedalLocomotion
#include <BipedalLocomotion/Math/ContactWrenchCone.h>
#include <BipedalLocomotion/Math/Wrench.h>
#include <BipedalLocomotion/ParametersHandler/StdImplementation.h>
#include <BipedalLocomotion/System/VariablesHandler.h>
#include <BipedalLocomotion/TSID/FeasibleContactWrenchTask.h>

#include <iDynTree/EigenHelpers.h>
#include <iDynTree/ModelTestUtils.h>

#include <Eigen/Dense>

using namespace BipedalLocomotion::ParametersHandler;
using namespace BipedalLocomotion::System;
using namespace BipedalLocomotion::TSID;
using namespace BipedalLocomotion::Math;

TEST_CASE("Fesasible contact wrench Test")
{
    auto kinDyn = std::make_shared<iDynTree::KinDynComputations>();
    auto parameterHandler = std::make_shared<StdImplementation>();

    std::vector<double>limitsX{-0.05, 0.1};
    std::vector<double>limitsY{-0.02, 0.01};
    parameterHandler->setParameter("number_of_slices", 2);
    parameterHandler->setParameter("static_friction_coefficient", 0.3);
    parameterHandler->setParameter("foot_limits_x", limitsX);
    parameterHandler->setParameter("foot_limits_y", limitsY);

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
            variablesHandler.addVariable("contact0", 6);
            variablesHandler.addVariable("dummy2", 4);

            const std::string frameName = model.getFrameName(numberOfJoints);
            parameterHandler->setParameter("variable_name", "contact0");
            parameterHandler->setParameter("frame_name", frameName);

            FeasibleContactWrenchTask task;
            REQUIRE(task.setKinDyn(kinDyn));
            REQUIRE(task.initialize(parameterHandler));
            REQUIRE(task.setVariablesHandler(variablesHandler));
            REQUIRE(task.update());

            // get A and b
            Eigen::Ref<const Eigen::MatrixXd> A = task.getA();
            Eigen::Ref<const Eigen::VectorXd> b = task.getB();

            // check the matrix A
            //
            BipedalLocomotion::Math::ContactWrenchCone cone;
            cone.initialize(parameterHandler);
            const std::size_t numberOfConstraints = cone.getB().rows();
            Eigen::MatrixXd coneWithNormalForceA(numberOfConstraints + 2, 6);

            coneWithNormalForceA.topRows(numberOfConstraints) = cone.getA();
            coneWithNormalForceA.bottomRows(2) << 0, 0, -1, 0, 0, 0,
                                                  0, 0,  1, 0, 0, 0;

            Eigen::VectorXd coneWithNormalForceB(numberOfConstraints + 2);
            coneWithNormalForceB.head(numberOfConstraints) = cone.getB();
            coneWithNormalForceB.tail<2>() << 0, std::numeric_limits<double>::max();

            const auto frame_R_world = kinDyn->getWorldTransform(frameName).getRotation().inverse();
            const auto frame_T_world_rotationOnly
                = iDynTree::Transform(frame_R_world, iDynTree::Position::Zero());
            const auto adjointMatrix = frame_T_world_rotationOnly.asAdjointTransform();

            REQUIRE(A.middleCols(variablesHandler.getVariable("dummy1").offset,
                                 variablesHandler.getVariable("dummy1").size)
                        .isZero());

            REQUIRE(A.middleCols(variablesHandler.getVariable("dummy2").offset,
                                 variablesHandler.getVariable("dummy2").size)
                        .isZero());

            REQUIRE(A.middleCols(variablesHandler.getVariable("contact0").offset,
                                 variablesHandler.getVariable("contact0").size)
                        .isApprox(coneWithNormalForceA * iDynTree::toEigen(adjointMatrix)));

            // check the vector b
            REQUIRE(b.isApprox(coneWithNormalForceB));

            // disable the task
            const bool isActive = false;
            task.setContactActive(isActive);
            coneWithNormalForceB.tail(1)(0) = 0;
            REQUIRE(b.isApprox(coneWithNormalForceB));

            REQUIRE(A.middleCols(variablesHandler.getVariable("dummy1").offset,
                                 variablesHandler.getVariable("dummy1").size)
                        .isZero());

            REQUIRE(A.middleCols(variablesHandler.getVariable("dummy2").offset,
                                 variablesHandler.getVariable("dummy2").size)
                        .isZero());

            REQUIRE(A.middleCols(variablesHandler.getVariable("contact0").offset,
                                 variablesHandler.getVariable("contact0").size)
                        .isApprox(coneWithNormalForceA * iDynTree::toEigen(adjointMatrix)));
        }
    }
}
