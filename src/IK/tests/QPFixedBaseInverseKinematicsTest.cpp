/**
 * @file QPFixedBaseInverseKinematicsTest.cpp
 * @authors Giulio Romualdi
 * @copyright 2023 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the BSD-3-Clause license.
 */

// Catch2
#include <catch2/catch_test_macros.hpp>

// std
#include <iDynTree/Transform.h>
#include <memory>
#include <random>

// BipedalLocomotion
#include <BipedalLocomotion/Conversions/ManifConversions.h>
#include <BipedalLocomotion/ParametersHandler/StdImplementation.h>

#include <BipedalLocomotion/System/ConstantWeightProvider.h>
#include <BipedalLocomotion/System/VariablesHandler.h>

#include <BipedalLocomotion/IK/IntegrationBasedIK.h>
#include <BipedalLocomotion/IK/JointTrackingTask.h>
#include <BipedalLocomotion/IK/QPFixedBaseInverseKinematics.h>
#include <BipedalLocomotion/IK/SE3Task.h>

#include <BipedalLocomotion/ContinuousDynamicalSystem/FloatingBaseSystemKinematics.h>
#include <BipedalLocomotion/ContinuousDynamicalSystem/ForwardEuler.h>

#include <iDynTree/EigenHelpers.h>
#include <iDynTree/ModelTestUtils.h>

using namespace BipedalLocomotion::ParametersHandler;
using namespace BipedalLocomotion::System;
using namespace BipedalLocomotion::ContinuousDynamicalSystem;
using namespace BipedalLocomotion::IK;
using namespace BipedalLocomotion::Conversions;
using namespace std::chrono_literals;

constexpr auto robotVelocity = "robotVelocity";
constexpr std::chrono::nanoseconds dT = 10ms;

struct InverseKinematicsAndTasks
{
    std::shared_ptr<QPFixedBaseInverseKinematics> ik;
    std::shared_ptr<SE3Task> se3Task;
    std::shared_ptr<JointTrackingTask> regularizationTask;
};

struct DesiredSetPoints
{
    manif::SE3d endEffectorPose;
    Eigen::VectorXd joints;
};

struct System
{
    std::shared_ptr<ForwardEuler<FloatingBaseSystemKinematics>> integrator;
    std::shared_ptr<FloatingBaseSystemKinematics> dynamics;
};

std::shared_ptr<IParametersHandler> createParameterHandler()
{
    constexpr double gain = 20.0;

    auto parameterHandler = std::make_shared<StdImplementation>();
    parameterHandler->setParameter("robot_velocity_variable_name", robotVelocity);

    parameterHandler->setParameter("verbosity", false);

    /////// SE3 Task
    auto SE3ParameterHandler = std::make_shared<StdImplementation>();
    SE3ParameterHandler->setParameter("robot_velocity_variable_name", robotVelocity);

    SE3ParameterHandler->setParameter("kp_linear", gain);
    SE3ParameterHandler->setParameter("kp_angular", gain);
    parameterHandler->setGroup("SE3_TASK", SE3ParameterHandler);

    /////// Joint regularization task
    auto jointRegularizationHandler = std::make_shared<StdImplementation>();
    jointRegularizationHandler->setParameter("robot_velocity_variable_name", robotVelocity);
    parameterHandler->setGroup("REGULARIZATION_TASK", jointRegularizationHandler);

    return parameterHandler;
}

InverseKinematicsAndTasks createIK(std::shared_ptr<IParametersHandler> handler,
                                   std::shared_ptr<iDynTree::KinDynComputations> kinDyn,
                                   const VariablesHandler& variables)
{
    // prepare the parameters related to the size of the system
    const Eigen::VectorXd kpRegularization = Eigen::VectorXd::Ones(kinDyn->model().getNrOfDOFs());
    const Eigen::VectorXd weightRegularization = kpRegularization;
    handler->getGroup("REGULARIZATION_TASK").lock()->setParameter("kp", kpRegularization);

    InverseKinematicsAndTasks out;

    constexpr std::size_t highPriority = 0;
    constexpr std::size_t lowPriority = 1;

    out.ik = std::make_shared<QPFixedBaseInverseKinematics>();
    REQUIRE(out.ik->setKinDyn(kinDyn));
    REQUIRE(out.ik->initialize(handler));

    out.se3Task = std::make_shared<SE3Task>();
    REQUIRE(out.se3Task->setKinDyn(kinDyn));
    REQUIRE(out.se3Task->initialize(handler->getGroup("SE3_TASK")));
    REQUIRE(out.ik->addTask(out.se3Task, "se3_task", highPriority));

    out.regularizationTask = std::make_shared<JointTrackingTask>();

    REQUIRE(out.regularizationTask->setKinDyn(kinDyn));
    REQUIRE(out.regularizationTask->initialize(handler->getGroup("REGULARIZATION_TASK")));
    REQUIRE(out.ik->addTask(out.regularizationTask,
                            "regularization_task",
                            lowPriority,
                            weightRegularization));



    Eigen::VectorXd newWeight = 10 * weightRegularization;
    auto newWeightProvider = std::make_shared<BipedalLocomotion::System::ConstantWeightProvider>(newWeight);
    REQUIRE(out.ik->setTaskWeight("regularization_task", newWeightProvider));

    auto outWeightProvider = out.ik->getTaskWeightProvider("regularization_task").lock();
    REQUIRE(outWeightProvider);
    REQUIRE(outWeightProvider->getOutput().isApprox(newWeight));

    REQUIRE(out.ik->setTaskWeight("regularization_task", weightRegularization));

    REQUIRE(out.ik->finalize(variables));

    return out;
}

DesiredSetPoints getDesiredReference(std::shared_ptr<iDynTree::KinDynComputations> kinDyn,
                                     std::size_t numberOfJoints)
{
    DesiredSetPoints out;

    const auto worldBasePos = iDynTree::Transform::Identity();
    const auto baseVel = iDynTree::Twist::Zero();

    iDynTree::VectorDynSize jointsPos(kinDyn->model().getNrOfDOFs());
    iDynTree::VectorDynSize jointsVel(kinDyn->model().getNrOfDOFs());
    iDynTree::Vector3 gravity;

    for (auto& joint : jointsPos)
    {
        joint = iDynTree::getRandomDouble();
    }

    for (auto& joint : jointsVel)
    {
        joint = 0;
    }

    for (auto& element : gravity)
    {
        element = iDynTree::getRandomDouble();
    }

    REQUIRE(kinDyn->setRobotState(worldBasePos, jointsPos, baseVel, jointsVel, gravity));

    const std::string controlledFrame = kinDyn->model().getFrameName(numberOfJoints);
    out.endEffectorPose = toManifPose(kinDyn->getWorldTransform(controlledFrame));

    out.joints.resize(jointsPos.size());
    out.joints = iDynTree::toEigen(jointsPos);

    return out;
}

System getSystem(std::shared_ptr<iDynTree::KinDynComputations> kinDyn)
{
    System out;

    // random noise in the joints position
    std::default_random_engine generator;
    generator.seed(42);
    std::normal_distribution<double> distribution(0.0, 0.01);

    // create the System
    Eigen::Matrix<double, 6, 1> baseVelocity;
    Eigen::VectorXd jointVelocities(kinDyn->getNrOfDegreesOfFreedom());
    Eigen::Matrix4d basePose;
    Eigen::VectorXd jointPositions(kinDyn->getNrOfDegreesOfFreedom());
    Eigen::Vector3d gravity;

    REQUIRE(kinDyn->getRobotState(basePose, //
                                  jointPositions,
                                  baseVelocity,
                                  jointVelocities,
                                  gravity));

    // perturb the joint position
    for (int i = 0; i < kinDyn->getNrOfDegreesOfFreedom(); i++)
    {
        jointPositions[i] += distribution(generator);
    }

    out.dynamics = std::make_shared<FloatingBaseSystemKinematics>();
    out.dynamics->setState({basePose.topRightCorner<3, 1>(),
                            toManifRot(basePose.topLeftCorner<3, 3>()),
                            jointPositions});

    out.integrator = std::make_shared<ForwardEuler<FloatingBaseSystemKinematics>>();
    REQUIRE(out.integrator->setIntegrationStep(dT));
    out.integrator->setDynamicalSystem(out.dynamics);

    return out;
}

TEST_CASE("QP-IK")
{
    auto kinDyn = std::make_shared<iDynTree::KinDynComputations>();
    auto parameterHandler = createParameterHandler();

    constexpr double tolerance = 1e-1;

    // set the velocity representation
    REQUIRE(kinDyn->setFrameVelocityRepresentation(
        iDynTree::FrameVelocityRepresentation::MIXED_REPRESENTATION));

    for (std::size_t numberOfJoints = 20; numberOfJoints < 40; numberOfJoints += 15)
    {
        DYNAMIC_SECTION("Model with " << numberOfJoints << " joints")
        {
            // create the model
            constexpr std::size_t nrOfAdditionalFrames = 10;
            constexpr bool onlyRevoluteJoints = true;
            const iDynTree::Model model = iDynTree::getRandomChain(numberOfJoints,
                                                                   nrOfAdditionalFrames,
                                                                   onlyRevoluteJoints);
            REQUIRE(kinDyn->loadRobotModel(model));

            const auto desiredSetPoints = getDesiredReference(kinDyn, numberOfJoints);

            // Instantiate the handler
            VariablesHandler variablesHandler;
            variablesHandler.addVariable(robotVelocity, model.getNrOfDOFs() + 6);

            auto system = getSystem(kinDyn);

            // Set the frame name
            const std::string controlledFrame = model.getFrameName(numberOfJoints);
            parameterHandler->getGroup("SE3_TASK")
                .lock()
                ->setParameter("frame_name", controlledFrame);

            // create the IK
            auto ikAndTasks = createIK(parameterHandler, kinDyn, variablesHandler);

            REQUIRE(ikAndTasks.se3Task->setSetPoint(desiredSetPoints.endEffectorPose,
                                                    manif::SE3d::Tangent::Zero()));
            REQUIRE(ikAndTasks.regularizationTask->setSetPoint(desiredSetPoints.joints));

            // propagate the inverse kinematics for
            constexpr std::size_t iterations = 20;
            Eigen::Vector3d gravity;
            gravity << 0, 0, -9.81;
            Eigen::Matrix4d baseTransform = Eigen::Matrix4d::Identity();
            Eigen::Matrix<double, 6, 1> baseVelocity = Eigen::Matrix<double, 6, 1>::Zero();
            Eigen::VectorXd jointVelocity = Eigen::VectorXd::Zero(model.getNrOfDOFs());

            for (std::size_t iteration = 0; iteration < iterations; iteration++)
            {
                // get the solution of the integrator
                const auto& [basePosition, baseRotation, jointPosition]
                    = system.integrator->getSolution();

                // update the KinDynComputations object
                baseTransform.topLeftCorner<3, 3>() = baseRotation.rotation();
                baseTransform.topRightCorner<3, 1>() = basePosition;
                REQUIRE(kinDyn->setRobotState(baseTransform,
                                              jointPosition,
                                              baseVelocity,
                                              jointVelocity,
                                              gravity));

                // solve the IK
                REQUIRE(ikAndTasks.ik->advance());

                // get the output of the IK
                baseVelocity = ikAndTasks.ik->getOutput().baseVelocity.coeffs();
                jointVelocity = ikAndTasks.ik->getOutput().jointVelocity;

                // propagate the dynamical system
                system.dynamics->setControlInput({baseVelocity, jointVelocity});
                system.integrator->integrate(0s, dT);
            }

            // Check the end-effector pose error
            const manif::SE3d endEffectorPose  = toManifPose(kinDyn->getWorldTransform(controlledFrame));

            // please read it as (log(desiredSetPoints.endEffectorPose^-1 * endEffectorPose))^v
            const manif::SE3d::Tangent error = endEffectorPose - desiredSetPoints.endEffectorPose;
            REQUIRE(error.coeffs().isZero(tolerance));
        }
    }
}
