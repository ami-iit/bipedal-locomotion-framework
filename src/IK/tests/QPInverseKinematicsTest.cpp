/**
 * @file QPInverseKinematicsTest.cpp
 * @authors Giulio Romualdi
 * @copyright 2021 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the BSD-3-Clause license.
 */

// Catch2
#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

// std
#include <memory>
#include <ostream>
#include <random>

// BipedalLocomotion
#include <BipedalLocomotion/Conversions/ManifConversions.h>
#include <BipedalLocomotion/ParametersHandler/StdImplementation.h>

#include <BipedalLocomotion/System/ConstantWeightProvider.h>
#include <BipedalLocomotion/System/VariablesHandler.h>

#include <BipedalLocomotion/IK/CoMTask.h>
#include <BipedalLocomotion/IK/DistanceTask.h>
#include <BipedalLocomotion/IK/GravityTask.h>
#include <BipedalLocomotion/IK/IntegrationBasedIK.h>
#include <BipedalLocomotion/IK/JointLimitsTask.h>
#include <BipedalLocomotion/IK/JointTrackingTask.h>
#include <BipedalLocomotion/IK/QPInverseKinematics.h>
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

struct InverseKinematicsTasks
{
    std::shared_ptr<SE3Task> se3Task;
    std::shared_ptr<CoMTask> comTask;
    std::shared_ptr<JointTrackingTask> regularizationTask;
    std::shared_ptr<JointLimitsTask> jointLimitsTask;
    std::shared_ptr<DistanceTask> distanceTask;
    std::shared_ptr<GravityTask> gravityTask;
};

struct DesiredSetPoints
{
    Eigen::Vector3d CoMPosition;
    std::string endEffectorFrame;
    manif::SE3d endEffectorPose;
    Eigen::VectorXd joints;
    std::string targetFrameDistance;
    double targetDistance;
    std::string targetFrameGravity;
    Eigen::Vector3d desiredBodyGravity;
};

struct System
{
    std::shared_ptr<ForwardEuler<FloatingBaseSystemKinematics>> integrator;
    std::shared_ptr<FloatingBaseSystemKinematics> dynamics;
};

std::shared_ptr<IParametersHandler> createParameterHandler()
{

    constexpr double gain = 5.0;
    constexpr double additionalTasksWeight = 10.0;

    auto parameterHandler = std::make_shared<StdImplementation>();
    parameterHandler->setParameter("robot_velocity_variable_name", robotVelocity);
    parameterHandler->setParameter("verbosity", false);

    parameterHandler->setParameter("tasks",
                                   std::vector<std::string>{"SE3_TASK",
                                                            "COM_TASK",
                                                            "REGULARIZATION_TASK",
                                                            "JOINT_LIMITS_TASK",
                                                            "DISTANCE_TASK",
                                                            "GRAVITY_TASK"});

    /////// SE3 Task
    auto SE3ParameterHandler = std::make_shared<StdImplementation>();
    SE3ParameterHandler->setParameter("robot_velocity_variable_name", robotVelocity);

    SE3ParameterHandler->setParameter("kp_linear", gain);
    SE3ParameterHandler->setParameter("kp_angular", gain);
    SE3ParameterHandler->setParameter("type", "SE3Task");
    SE3ParameterHandler->setParameter("priority", 0);
    parameterHandler->setGroup("SE3_TASK", SE3ParameterHandler);

    /////// CoM task
    auto CoMParameterHandler = std::make_shared<StdImplementation>();
    CoMParameterHandler->setParameter("robot_velocity_variable_name", robotVelocity);
    CoMParameterHandler->setParameter("kp_linear", gain);
    CoMParameterHandler->setParameter("type", "CoMTask");
    CoMParameterHandler->setParameter("priority", 0);
    parameterHandler->setGroup("COM_TASK", CoMParameterHandler);

    /////// Joint regularization task
    auto jointRegularizationHandler = std::make_shared<StdImplementation>();
    jointRegularizationHandler->setParameter("robot_velocity_variable_name", robotVelocity);
    jointRegularizationHandler->setParameter("type", "JointTrackingTask");
    jointRegularizationHandler->setParameter("priority", 1);
    parameterHandler->setGroup("REGULARIZATION_TASK", jointRegularizationHandler);

    /////// Joint limits task
    auto jointLimitsHandler = std::make_shared<StdImplementation>();
    jointLimitsHandler->setParameter("robot_velocity_variable_name", robotVelocity);
    jointLimitsHandler->setParameter("sampling_time", dT);
    jointLimitsHandler->setParameter("use_model_limits", false);
    jointLimitsHandler->setParameter("type", "JointLimitsTask");
    jointLimitsHandler->setParameter("priority", 0);
    parameterHandler->setGroup("JOINT_LIMITS_TASK", jointLimitsHandler);

    /////// Distance task
    auto distanceTaskHandler = std::make_shared<StdImplementation>();
    distanceTaskHandler->setParameter("robot_velocity_variable_name", robotVelocity);
    distanceTaskHandler->setParameter("type", "DistanceTask");
    distanceTaskHandler->setParameter("priority", 1);
    distanceTaskHandler->setParameter("kp", gain * 5);
    Eigen::VectorXd distanceWeight(1);
    distanceWeight << additionalTasksWeight;
    distanceTaskHandler->setParameter("weight", distanceWeight);
    parameterHandler->setGroup("DISTANCE_TASK", distanceTaskHandler);

    /////// Gravity task
    auto gravityTaskHandler = std::make_shared<StdImplementation>();
    gravityTaskHandler->setParameter("robot_velocity_variable_name", robotVelocity);
    gravityTaskHandler->setParameter("type", "GravityTask");
    gravityTaskHandler->setParameter("priority", 1);
    gravityTaskHandler->setParameter("kp", gain * 10);
    const Eigen::Vector2d gravityWeight = additionalTasksWeight * Eigen::Vector2d::Ones();
    gravityTaskHandler->setParameter("weight", gravityWeight);
    parameterHandler->setGroup("GRAVITY_TASK", gravityTaskHandler);

    return parameterHandler;
}

void finalizeParameterHandler(std::shared_ptr<IParametersHandler> handler,
                              std::shared_ptr<iDynTree::KinDynComputations> kinDyn,
                              const System& system,
                              const Eigen::Ref<const Eigen::VectorXd>& jointsLimits)
{
    // prepare the parameters related to the size of the system
    const Eigen::VectorXd kpRegularization = Eigen::VectorXd::Ones(kinDyn->model().getNrOfDOFs());
    const Eigen::VectorXd weightRegularization = kpRegularization;
    handler->getGroup("REGULARIZATION_TASK").lock()->setParameter("kp", kpRegularization);
    handler->getGroup("REGULARIZATION_TASK").lock()->setParameter("weight", weightRegularization);

    const Eigen::VectorXd kLimRegularization
        = 0.5 * Eigen::VectorXd::Ones(kinDyn->model().getNrOfDOFs());
    handler->getGroup("JOINT_LIMITS_TASK").lock()->setParameter("klim", kLimRegularization);

    const Eigen::VectorXd jointPositions = std::get<2>(system.dynamics->getState());
    const Eigen::VectorXd upperLimits = jointPositions + jointsLimits;
    const Eigen::VectorXd lowerLimits = jointPositions - jointsLimits;

    handler->getGroup("JOINT_LIMITS_TASK").lock()->setParameter("upper_limits", upperLimits);
    handler->getGroup("JOINT_LIMITS_TASK").lock()->setParameter("lower_limits", lowerLimits);
}

InverseKinematicsTasks createIKTasks(std::shared_ptr<IParametersHandler> handler,
                                     std::shared_ptr<iDynTree::KinDynComputations> kinDyn)
{

    InverseKinematicsTasks out;

    out.se3Task = std::make_shared<SE3Task>();
    REQUIRE(out.se3Task->setKinDyn(kinDyn));
    REQUIRE(out.se3Task->initialize(handler->getGroup("SE3_TASK")));

    out.comTask = std::make_shared<CoMTask>();
    REQUIRE(out.comTask->setKinDyn(kinDyn));
    REQUIRE(out.comTask->initialize(handler->getGroup("COM_TASK")));

    out.regularizationTask = std::make_shared<JointTrackingTask>();
    REQUIRE(out.regularizationTask->setKinDyn(kinDyn));
    REQUIRE(out.regularizationTask->initialize(handler->getGroup("REGULARIZATION_TASK")));

    out.jointLimitsTask = std::make_shared<JointLimitsTask>();
    REQUIRE(out.jointLimitsTask->setKinDyn(kinDyn));
    REQUIRE(out.jointLimitsTask->initialize(handler->getGroup("JOINT_LIMITS_TASK")));

    out.distanceTask = std::make_shared<DistanceTask>();
    REQUIRE(out.distanceTask->setKinDyn(kinDyn));
    REQUIRE(out.distanceTask->initialize(handler->getGroup("DISTANCE_TASK")));

    out.gravityTask = std::make_shared<GravityTask>();
    REQUIRE(out.gravityTask->setKinDyn(kinDyn));
    REQUIRE(out.gravityTask->initialize(handler->getGroup("GRAVITY_TASK")));

    return out;
}

DesiredSetPoints getDesiredReference(std::shared_ptr<iDynTree::KinDynComputations> kinDyn,
                                     std::size_t numberOfJoints)
{
    DesiredSetPoints out;

    const auto worldBasePos = iDynTree::getRandomTransform();
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

    // getCoMPosition and velocity
    out.CoMPosition = iDynTree::toEigen(kinDyn->getCenterOfMassPosition());
    out.endEffectorFrame = kinDyn->model().getFrameName(numberOfJoints);
    out.endEffectorPose = toManifPose(kinDyn->getWorldTransform(out.endEffectorFrame));
    out.joints.resize(jointsPos.size());
    out.joints = iDynTree::toEigen(jointsPos);
    iDynTree::LinkIndex indexDistance, indexGravity;

    // Find a frame sufficiently far away from those used for the distance and SE3 task
    do
    {
        out.targetFrameGravity = iDynTree::getRandomLinkOfModel(kinDyn->model());
        out.targetFrameDistance = iDynTree::getRandomLinkOfModel(kinDyn->model());
        indexGravity = kinDyn->model().getFrameIndex(out.targetFrameGravity);
        indexDistance = kinDyn->model().getFrameIndex(out.targetFrameDistance);
    } while (std::abs(indexGravity - kinDyn->model().getFrameIndex(out.endEffectorFrame)) < 5
             || std::abs(indexDistance - kinDyn->model().getFrameIndex(out.endEffectorFrame)) < 5
             || std::abs(indexGravity - indexDistance) < 5 || indexGravity < 3
             || indexDistance < 3);

    out.targetDistance
        = toManifPose(kinDyn->getWorldTransform(out.targetFrameDistance)).translation().norm();

    // The desired gravity in body frame is the world z axis in body frame,
    // that is the third row of the body rotation matrix
    out.desiredBodyGravity
        = toManifPose(kinDyn->getWorldTransform(out.targetFrameGravity)).rotation().matrix().row(2);

    return out;
}

System getSystem(std::shared_ptr<iDynTree::KinDynComputations> kinDyn)
{
    System out;

    // random noise in the joints position
    std::default_random_engine generator;
    generator.seed(42);
    std::normal_distribution<double> distribution(0.0, 0.1);

    // create the System
    Eigen::Matrix<double, 6, 1> baseVelocity;
    Eigen::VectorXd jointVelocities(kinDyn->getNrOfDegreesOfFreedom());
    Eigen::Matrix4d basePose;
    Eigen::VectorXd jointPositions(kinDyn->getNrOfDegreesOfFreedom());
    Eigen::Vector3d gravity;

    REQUIRE(
        kinDyn->getRobotState(basePose, jointPositions, baseVelocity, jointVelocities, gravity));

    // perturb the joint position
    for (int i = 0; i < kinDyn->getNrOfDegreesOfFreedom(); i++)
    {
        jointPositions[i] += distribution(generator);
    }

    Eigen::AngleAxisd baseAngleAxis(basePose.topLeftCorner<3, 3>());

    double newAngle = baseAngleAxis.angle() + distribution(generator);

    Eigen::AngleAxisd newBaseRotation(newAngle, baseAngleAxis.axis());

    out.dynamics = std::make_shared<FloatingBaseSystemKinematics>();
    out.dynamics->setState(
        {basePose.topRightCorner<3, 1>(), toManifRot(newBaseRotation.matrix()), jointPositions});

    out.integrator = std::make_shared<ForwardEuler<FloatingBaseSystemKinematics>>();
    REQUIRE(out.integrator->setIntegrationStep(dT));
    out.integrator->setDynamicalSystem(out.dynamics);

    REQUIRE(
        kinDyn->setRobotState(basePose, jointPositions, baseVelocity, jointVelocities, gravity));

    return out;
}

inline std::string customInt2string(int i)
{
    std::stringstream ss;

    ss << i;

    return ss.str();
}

// Workaround for https://github.com/ami-iit/bipedal-locomotion-framework/issues/799 and
// https://github.com/robotology/idyntree/pull/1171
inline iDynTree::Model customGetRandomModelWithNoPrismaticJoints(unsigned int nrOfJoints,
                                                                 size_t nrOfAdditionalFrames = 10,
                                                                 bool onlyRevoluteJoints = false)
{
    iDynTree::Model model;

    model.addLink("baseLink", iDynTree::getRandomLink());

    for (unsigned int i = 0; i < nrOfJoints; i++)
    {
        std::string parentLink = iDynTree::getRandomLinkOfModel(model);
        std::string linkName = "link" + customInt2string(i);
        iDynTree::addRandomLinkToModel(model, parentLink, linkName, onlyRevoluteJoints);
    }

    for (unsigned int i = 0; i < nrOfAdditionalFrames; i++)
    {
        std::string parentLink = iDynTree::getRandomLinkOfModel(model);
        std::string frameName = "additionalFrame" + customInt2string(i);
        iDynTree::addRandomAdditionalFrameToModel(model, parentLink, frameName);
    }

    return model;
}

TEST_CASE("QP-IK")
{
    auto kinDyn = std::make_shared<iDynTree::KinDynComputations>();
    auto parameterHandler = createParameterHandler();

    constexpr double tolerance = 2e-1;
    constexpr std::size_t highPriority = 0;
    constexpr std::size_t lowPriority = 1;

    // set the velocity representation
    REQUIRE(kinDyn->setFrameVelocityRepresentation(
        iDynTree::FrameVelocityRepresentation::MIXED_REPRESENTATION));

    for (std::size_t numberOfJoints = 20; numberOfJoints < 40; numberOfJoints += 15)
    {
        DYNAMIC_SECTION("Model with " << numberOfJoints << " joints")
        {
            // create the model
            size_t nrOfAdditionalFrames = 10;
            bool onlyRevoluteJoints = true;
            const iDynTree::Model model
                = customGetRandomModelWithNoPrismaticJoints(numberOfJoints,
                                                            nrOfAdditionalFrames,
                                                            onlyRevoluteJoints);
            REQUIRE(kinDyn->loadRobotModel(model));

            const auto desiredSetPoints = getDesiredReference(kinDyn, numberOfJoints);

            // Instantiate the handler
            VariablesHandler variablesHandler;
            variablesHandler.addVariable(robotVelocity, model.getNrOfDOFs() + 6);

            auto system = getSystem(kinDyn);

            // Set the frame name
            parameterHandler->getGroup("SE3_TASK")
                .lock()
                ->setParameter("frame_name", desiredSetPoints.endEffectorFrame);

            parameterHandler->getGroup("DISTANCE_TASK")
                .lock()
                ->setParameter("target_frame_name", desiredSetPoints.targetFrameDistance);

            parameterHandler->getGroup("GRAVITY_TASK")
                .lock()
                ->setParameter("target_frame_name", desiredSetPoints.targetFrameGravity);

            // create the IK
            constexpr double jointLimitDelta = 0.5;
            finalizeParameterHandler(parameterHandler,
                                     kinDyn,
                                     system,
                                     Eigen::VectorXd::Constant(kinDyn->model().getNrOfDOFs(),
                                                               jointLimitDelta));
            auto ikTasks = createIKTasks(parameterHandler, kinDyn);

            Eigen::VectorXd weightRegularization;

            auto ik = std::make_shared<QPInverseKinematics>();
            REQUIRE(ik->initialize(parameterHandler));
            REQUIRE(ik->addTask(ikTasks.se3Task, "se3_task", highPriority));
            REQUIRE(ik->addTask(ikTasks.comTask, "com_task", highPriority));

            REQUIRE(parameterHandler->getGroup("REGULARIZATION_TASK")
                        .lock()
                        ->getParameter("weight", weightRegularization));
            REQUIRE(ik->addTask(ikTasks.regularizationTask,
                                "regularization_task",
                                lowPriority,
                                weightRegularization));
            REQUIRE(ik->addTask(ikTasks.jointLimitsTask, "joint_limits_task", highPriority));

            const Eigen::VectorXd newWeight = 10 * weightRegularization;
            auto newWeightProvider
                = std::make_shared<BipedalLocomotion::System::ConstantWeightProvider>(newWeight);
            REQUIRE(ik->setTaskWeight("regularization_task", newWeightProvider));

            auto outWeightProvider = ik->getTaskWeightProvider("regularization_task").lock();
            REQUIRE(outWeightProvider);
            REQUIRE(outWeightProvider->getOutput().isApprox(newWeight));

            REQUIRE(ik->setTaskWeight("regularization_task", weightRegularization));

            REQUIRE(ik->finalize(variablesHandler));

            REQUIRE(ikTasks.se3Task->setSetPoint(desiredSetPoints.endEffectorPose,
                                                 manif::SE3d::Tangent::Zero()));
            REQUIRE(ikTasks.comTask->setSetPoint(desiredSetPoints.CoMPosition,
                                                 Eigen::Vector3d::Zero()));
            REQUIRE(ikTasks.regularizationTask->setSetPoint(desiredSetPoints.joints));

            // propagate the inverse kinematics for
            constexpr std::size_t iterations = 30;
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
                REQUIRE(ik->advance());

                // get the output of the IK
                baseVelocity = ik->getOutput().baseVelocity.coeffs();
                jointVelocity = ik->getOutput().jointVelocity;

                // propagate the dynamical system
                system.dynamics->setControlInput({baseVelocity, jointVelocity});
                system.integrator->integrate(0s, dT);
            }

            // check the CoM position
            REQUIRE(
                desiredSetPoints.CoMPosition.isApprox(toEigen(kinDyn->getCenterOfMassPosition()),
                                                      tolerance));

            // Check the end-effector pose error
            const manif::SE3d endEffectorPose
                = toManifPose(kinDyn->getWorldTransform(desiredSetPoints.endEffectorFrame));

            // please read it as (log(desiredSetPoints.endEffectorPose^-1 * endEffectorPose))^v
            const manif::SE3d::Tangent error = endEffectorPose - desiredSetPoints.endEffectorPose;
            REQUIRE(error.coeffs().isZero(tolerance));
        }
    }
}

TEST_CASE("QP-IK [With strict limits]")
{
    auto kinDyn = std::make_shared<iDynTree::KinDynComputations>();
    auto parameterHandler = createParameterHandler();

    constexpr double tolerance = 1e-2;
    constexpr std::size_t highPriority = 0;
    constexpr std::size_t lowPriority = 1;

    // set the velocity representation
    REQUIRE(kinDyn->setFrameVelocityRepresentation(
        iDynTree::FrameVelocityRepresentation::MIXED_REPRESENTATION));

    constexpr std::size_t numberOfJoints = 30;

    // create the model
    size_t nrOfAdditionalFrames = 10;
    bool onlyRevoluteJoints = true;
    const iDynTree::Model model = customGetRandomModelWithNoPrismaticJoints(numberOfJoints,
                                                                            nrOfAdditionalFrames,
                                                                            onlyRevoluteJoints);
    REQUIRE(kinDyn->loadRobotModel(model));

    const auto desiredSetPoints = getDesiredReference(kinDyn, numberOfJoints);

    // Instantiate the handler
    VariablesHandler variablesHandler;
    variablesHandler.addVariable(robotVelocity, model.getNrOfDOFs() + 6);

    auto system = getSystem(kinDyn);

    // Set the frame name
    parameterHandler->getGroup("SE3_TASK")
        .lock()
        ->setParameter("frame_name", desiredSetPoints.endEffectorFrame);

    parameterHandler->getGroup("DISTANCE_TASK")
        .lock()
        ->setParameter("target_frame_name", desiredSetPoints.targetFrameDistance);

    parameterHandler->getGroup("GRAVITY_TASK")
        .lock()
        ->setParameter("target_frame_name", desiredSetPoints.targetFrameGravity);

    // create the IK
    constexpr double jointLimitDelta = 0.5;
    Eigen::VectorXd limitsDelta
        = Eigen::VectorXd::Constant(kinDyn->model().getNrOfDOFs(), jointLimitDelta);

    // we limit the first and the forth joints.
    limitsDelta(0) = 0;
    limitsDelta(3) = 0;

    finalizeParameterHandler(parameterHandler, kinDyn, system, limitsDelta);
    auto ikTasks = createIKTasks(parameterHandler, kinDyn);

    Eigen::VectorXd weightRegularization;

    auto ik = std::make_shared<QPInverseKinematics>();
    REQUIRE(ik->initialize(parameterHandler));
    REQUIRE(ik->addTask(ikTasks.se3Task, "se3_task", highPriority));
    REQUIRE(ik->addTask(ikTasks.comTask, "com_task", highPriority));

    REQUIRE(parameterHandler->getGroup("REGULARIZATION_TASK")
                .lock()
                ->getParameter("weight", weightRegularization));
    REQUIRE(ik->addTask(ikTasks.regularizationTask,
                        "regularization_task",
                        lowPriority,
                        weightRegularization));
    REQUIRE(ik->addTask(ikTasks.jointLimitsTask, "joint_limits_task", highPriority));

    const Eigen::VectorXd newWeight = 10 * weightRegularization;
    auto newWeightProvider
        = std::make_shared<BipedalLocomotion::System::ConstantWeightProvider>(newWeight);
    REQUIRE(ik->setTaskWeight("regularization_task", newWeightProvider));

    auto outWeightProvider = ik->getTaskWeightProvider("regularization_task").lock();
    REQUIRE(outWeightProvider);
    REQUIRE(outWeightProvider->getOutput().isApprox(newWeight));

    REQUIRE(ik->setTaskWeight("regularization_task", weightRegularization));

    REQUIRE(ik->finalize(variablesHandler));

    REQUIRE(ikTasks.se3Task->setSetPoint(desiredSetPoints.endEffectorPose,
                                         manif::SE3d::Tangent::Zero()));
    REQUIRE(ikTasks.comTask->setSetPoint(desiredSetPoints.CoMPosition, Eigen::Vector3d::Zero()));
    REQUIRE(ikTasks.regularizationTask->setSetPoint(desiredSetPoints.joints));

    // propagate the inverse kinematics for
    constexpr std::size_t iterations = 30;
    Eigen::Vector3d gravity;
    gravity << 0, 0, -9.81;
    Eigen::Matrix4d baseTransform = Eigen::Matrix4d::Identity();
    Eigen::Matrix<double, 6, 1> baseVelocity = Eigen::Matrix<double, 6, 1>::Zero();
    Eigen::VectorXd jointVelocity = Eigen::VectorXd::Zero(model.getNrOfDOFs());

    const Eigen::VectorXd initialJointConfiguration = std::get<2>(system.dynamics->getState());

    for (std::size_t iteration = 0; iteration < iterations; iteration++)
    {
        // get the solution of the integrator
        const auto& [basePosition, baseRotation, jointPosition] = system.integrator->getSolution();

        // we check that the joint limit is not broken
        REQUIRE(std::abs(jointPosition(0) - initialJointConfiguration(0)) < tolerance);
        REQUIRE(std::abs(jointPosition(3) - initialJointConfiguration(3)) < tolerance);

        // update the KinDynComputations object
        baseTransform.topLeftCorner<3, 3>() = baseRotation.rotation();
        baseTransform.topRightCorner<3, 1>() = basePosition;
        REQUIRE(kinDyn->setRobotState(baseTransform,
                                      jointPosition,
                                      baseVelocity,
                                      jointVelocity,
                                      gravity));

        // solve the IK
        REQUIRE(ik->advance());

        // get the output of the IK
        baseVelocity = ik->getOutput().baseVelocity.coeffs();
        jointVelocity = ik->getOutput().jointVelocity;

        // given the joint constraint specified we check that the velocity is satisfied. This should
        // be zero
        REQUIRE(std::abs(jointVelocity(0)) < tolerance);
        REQUIRE(std::abs(jointVelocity(3)) < tolerance);

        // propagate the dynamical system
        system.dynamics->setControlInput({baseVelocity, jointVelocity});
        system.integrator->integrate(0s, dT);
    }
}

TEST_CASE("QP-IK [With builder]")
{
    auto kinDyn = std::make_shared<iDynTree::KinDynComputations>();
    auto parameterHandler = createParameterHandler();

    constexpr double tolerance = 2e-1;

    // set the velocity representation
    REQUIRE(kinDyn->setFrameVelocityRepresentation(
        iDynTree::FrameVelocityRepresentation::MIXED_REPRESENTATION));

    constexpr std::size_t numberOfJoints = 30;

    // create the model
    size_t nrOfAdditionalFrames = 10;
    bool onlyRevoluteJoints = true;
    const iDynTree::Model model = customGetRandomModelWithNoPrismaticJoints(numberOfJoints,
                                                                            nrOfAdditionalFrames,
                                                                            onlyRevoluteJoints);
    REQUIRE(kinDyn->loadRobotModel(model));

    // VariableHandler and IK params
    auto variablesParameterHandler = std::make_shared<StdImplementation>();
    variablesParameterHandler->setParameter("variables_name",
                                            std::vector<std::string>{robotVelocity});

    const int generalizedRobotVelocitySize = kinDyn->model().getNrOfDOFs() + 6;
    variablesParameterHandler->setParameter("variables_size",
                                            std::vector<int>{generalizedRobotVelocitySize});
    parameterHandler->setGroup("VARIABLES", variablesParameterHandler);

    auto ikParameterHandler = std::make_shared<StdImplementation>();
    ikParameterHandler->setParameter("robot_velocity_variable_name", robotVelocity);
    parameterHandler->setGroup("IK", ikParameterHandler);

    const auto desiredSetPoints = getDesiredReference(kinDyn, numberOfJoints);

    auto system = getSystem(kinDyn);

    // Set the frame name
    parameterHandler->getGroup("SE3_TASK")
        .lock()
        ->setParameter("frame_name", desiredSetPoints.endEffectorFrame);

    parameterHandler->getGroup("DISTANCE_TASK")
        .lock()
        ->setParameter("target_frame_name", desiredSetPoints.targetFrameDistance);

    parameterHandler->getGroup("GRAVITY_TASK")
        .lock()
        ->setParameter("target_frame_name", desiredSetPoints.targetFrameGravity);

    // finalize the parameter handler
    constexpr double jointLimitDelta = 0.5;
    finalizeParameterHandler(parameterHandler,
                             kinDyn,
                             system,
                             Eigen::VectorXd::Constant(kinDyn->model().getNrOfDOFs(),
                                                       jointLimitDelta));

    // automatic build the IK from parameter handler
    auto [variablesHandler, weights, ik] = QPInverseKinematics::build(parameterHandler, kinDyn);
    REQUIRE_FALSE(ik == nullptr);

    auto se3Task = std::dynamic_pointer_cast<SE3Task>(ik->getTask("SE3_TASK").lock());
    REQUIRE_FALSE(se3Task == nullptr);
    REQUIRE(se3Task->setSetPoint(desiredSetPoints.endEffectorPose, manif::SE3d::Tangent::Zero()));

    auto comTask = std::dynamic_pointer_cast<CoMTask>(ik->getTask("COM_TASK").lock());
    REQUIRE_FALSE(comTask == nullptr);
    REQUIRE(comTask->setSetPoint(desiredSetPoints.CoMPosition, Eigen::Vector3d::Zero()));

    auto regularizationTask
        = std::dynamic_pointer_cast<JointTrackingTask>(ik->getTask("REGULARIZATION_TASK").lock());
    REQUIRE_FALSE(regularizationTask == nullptr);
    REQUIRE(regularizationTask->setSetPoint(desiredSetPoints.joints));

    auto distanceTask
        = std::dynamic_pointer_cast<DistanceTask>(ik->getTask("DISTANCE_TASK").lock());
    REQUIRE_FALSE(distanceTask == nullptr);
    REQUIRE(distanceTask->setDesiredDistance(desiredSetPoints.targetDistance));

    auto gravityTask = std::dynamic_pointer_cast<GravityTask>(ik->getTask("GRAVITY_TASK").lock());
    REQUIRE_FALSE(gravityTask == nullptr);
    REQUIRE(
        gravityTask->setDesiredGravityDirectionInTargetFrame(desiredSetPoints.desiredBodyGravity));

    // propagate the inverse kinematics for
    constexpr std::size_t iterations = 30;
    Eigen::Vector3d gravity;
    gravity << 0, 0, -9.81;
    Eigen::Matrix4d baseTransform = Eigen::Matrix4d::Identity();
    Eigen::Matrix<double, 6, 1> baseVelocity = Eigen::Matrix<double, 6, 1>::Zero();
    Eigen::VectorXd jointVelocity = Eigen::VectorXd::Zero(model.getNrOfDOFs());
    for (std::size_t iteration = 0; iteration < iterations; iteration++)
    {
        // get the solution of the integrator
        const auto& [basePosition, baseRotation, jointPosition] = system.integrator->getSolution();

        // update the KinDynComputations object
        baseTransform.topLeftCorner<3, 3>() = baseRotation.rotation();
        baseTransform.topRightCorner<3, 1>() = basePosition;
        REQUIRE(kinDyn->setRobotState(baseTransform,
                                      jointPosition,
                                      baseVelocity,
                                      jointVelocity,
                                      gravity));

        // solve the IK
        REQUIRE(ik->advance());

        // get the output of the IK
        baseVelocity = ik->getOutput().baseVelocity.coeffs();
        jointVelocity = ik->getOutput().jointVelocity;

        // propagate the dynamical system
        system.dynamics->setControlInput({baseVelocity, jointVelocity});
        system.integrator->integrate(0s, dT);
    }

    auto weightProvider
        = std::dynamic_pointer_cast<const BipedalLocomotion::System::ConstantWeightProvider>(
            ik->getTaskWeightProvider("REGULARIZATION_TASK").lock());

    REQUIRE(weightProvider != nullptr);

    // check the CoM position
    REQUIRE(desiredSetPoints.CoMPosition.isApprox(toEigen(kinDyn->getCenterOfMassPosition()),
                                                  tolerance));

    // Check the end-effector pose error
    const manif::SE3d endEffectorPose
        = toManifPose(kinDyn->getWorldTransform(desiredSetPoints.endEffectorFrame));

    // please read it as (log(desiredSetPoints.endEffectorPose^-1 * endEffectorPose))^v
    const manif::SE3d::Tangent error = endEffectorPose - desiredSetPoints.endEffectorPose;
    REQUIRE(error.coeffs().isZero(tolerance));

    // Check the distance error
    REQUIRE(toManifPose(kinDyn->getWorldTransform(desiredSetPoints.targetFrameDistance))
                .translation()
                .norm()
            == Catch::Approx(desiredSetPoints.targetDistance).epsilon(tolerance * 0.1));

    // Check the gravity error
    Eigen::Vector3d gravityError;
    gravityError = toManifPose(kinDyn->getWorldTransform(desiredSetPoints.targetFrameGravity))
                       .rotation()
                       .matrix()
                       .row(2)
                       .transpose()
                   - desiredSetPoints.desiredBodyGravity;
    REQUIRE(gravityError.isZero(tolerance));
}

TEST_CASE("QP-IK [Distance and Gravity tasks]")
{
    auto kinDyn = std::make_shared<iDynTree::KinDynComputations>();
    auto parameterHandler = createParameterHandler();

    constexpr double tolerance = 2e-2;
    constexpr std::size_t highPriority = 0;
    constexpr std::size_t lowPriority = 1;

    // set the velocity representation
    REQUIRE(kinDyn->setFrameVelocityRepresentation(
        iDynTree::FrameVelocityRepresentation::MIXED_REPRESENTATION));

    constexpr std::size_t numberOfJoints = 30;

    // create the model
    size_t nrOfAdditionalFrames = 10;
    bool onlyRevoluteJoints = true;
    const iDynTree::Model model = customGetRandomModelWithNoPrismaticJoints(numberOfJoints,
                                                                            nrOfAdditionalFrames,
                                                                            onlyRevoluteJoints);
    REQUIRE(kinDyn->loadRobotModel(model));

    const auto desiredSetPoints = getDesiredReference(kinDyn, numberOfJoints);

    // Instantiate the handler
    VariablesHandler variablesHandler;
    variablesHandler.addVariable(robotVelocity, model.getNrOfDOFs() + 6);

    auto system = getSystem(kinDyn);

    // Set the frame name
    parameterHandler->getGroup("SE3_TASK")
        .lock()
        ->setParameter("frame_name", desiredSetPoints.endEffectorFrame);

    parameterHandler->getGroup("DISTANCE_TASK")
        .lock()
        ->setParameter("target_frame_name", desiredSetPoints.targetFrameDistance);

    parameterHandler->getGroup("GRAVITY_TASK")
        .lock()
        ->setParameter("target_frame_name", desiredSetPoints.targetFrameGravity);

    // create the IK
    constexpr double jointLimitDelta = 0.5;
    finalizeParameterHandler(parameterHandler,
                             kinDyn,
                             system,
                             Eigen::VectorXd::Constant(kinDyn->model().getNrOfDOFs(),
                                                       jointLimitDelta));
    auto ikTasks = createIKTasks(parameterHandler, kinDyn);

    Eigen::VectorXd weightRegularization;

    auto ik = std::make_shared<QPInverseKinematics>();
    REQUIRE(ik->initialize(parameterHandler));

    REQUIRE(parameterHandler->getGroup("REGULARIZATION_TASK")
                .lock()
                ->getParameter("weight", weightRegularization));
    REQUIRE(ik->addTask(ikTasks.regularizationTask,
                        "regularization_task",
                        lowPriority,
                        weightRegularization));
    REQUIRE(ik->addTask(ikTasks.distanceTask, "distance_task", highPriority));

    REQUIRE(ik->addTask(ikTasks.gravityTask, "gravity_task", highPriority));

    REQUIRE(ik->finalize(variablesHandler));

    REQUIRE(ikTasks.regularizationTask->setSetPoint(desiredSetPoints.joints));

    REQUIRE(ikTasks.distanceTask->setDesiredDistance(desiredSetPoints.targetDistance));

    REQUIRE(ikTasks.gravityTask->setDesiredGravityDirectionInTargetFrame(
        desiredSetPoints.desiredBodyGravity));

    // propagate the inverse kinematics for
    constexpr std::size_t iterations = 30;
    Eigen::Vector3d gravity;
    gravity << 0, 0, -9.81;
    Eigen::Matrix4d baseTransform = Eigen::Matrix4d::Identity();
    Eigen::Matrix<double, 6, 1> baseVelocity = Eigen::Matrix<double, 6, 1>::Zero();
    Eigen::VectorXd jointVelocity = Eigen::VectorXd::Zero(model.getNrOfDOFs());

    for (std::size_t iteration = 0; iteration < iterations; iteration++)
    {
        // get the solution of the integrator
        const auto& [basePosition, baseRotation, jointPosition] = system.integrator->getSolution();

        // update the KinDynComputations object
        baseTransform.topLeftCorner<3, 3>() = baseRotation.rotation();
        baseTransform.topRightCorner<3, 1>() = basePosition;
        REQUIRE(kinDyn->setRobotState(baseTransform,
                                      jointPosition,
                                      baseVelocity,
                                      jointVelocity,
                                      gravity));

        // solve the IK
        REQUIRE(ik->advance());

        // get the output of the IK
        baseVelocity = ik->getOutput().baseVelocity.coeffs();
        jointVelocity = ik->getOutput().jointVelocity;

        // propagate the dynamical system
        system.dynamics->setControlInput({baseVelocity, jointVelocity});
        system.integrator->integrate(0s, dT);
    }

    // Check the distance error
    REQUIRE(toManifPose(kinDyn->getWorldTransform(desiredSetPoints.targetFrameDistance))
                .translation()
                .norm()
            == Catch::Approx(desiredSetPoints.targetDistance).epsilon(tolerance));

    // Check the gravity error
    Eigen::Vector3d gravityError;
    gravityError = toManifPose(kinDyn->getWorldTransform(desiredSetPoints.targetFrameGravity))
                       .rotation()
                       .matrix()
                       .row(2)
                       .transpose()
                   - desiredSetPoints.desiredBodyGravity;
    REQUIRE(gravityError.isZero(tolerance));
}

TEST_CASE("QP-IK [Distance and Gravity tasks Unconstrained]")
{
    auto kinDyn = std::make_shared<iDynTree::KinDynComputations>();
    auto parameterHandler = createParameterHandler();

    constexpr double tolerance = 2e-2;
    constexpr std::size_t highPriority = 0;
    constexpr std::size_t lowPriority = 1;

    // set the velocity representation
    REQUIRE(kinDyn->setFrameVelocityRepresentation(
        iDynTree::FrameVelocityRepresentation::MIXED_REPRESENTATION));

    constexpr std::size_t numberOfJoints = 30;

    // create the model
    const iDynTree::Model model = iDynTree::getRandomModel(numberOfJoints);
    REQUIRE(kinDyn->loadRobotModel(model));

    const auto desiredSetPoints = getDesiredReference(kinDyn, numberOfJoints);

    // Instantiate the handler
    VariablesHandler variablesHandler;
    variablesHandler.addVariable(robotVelocity, model.getNrOfDOFs() + 6);

    auto system = getSystem(kinDyn);

    // Set the frame name
    parameterHandler->getGroup("SE3_TASK")
        .lock()
        ->setParameter("frame_name", desiredSetPoints.endEffectorFrame);

    parameterHandler->getGroup("DISTANCE_TASK")
        .lock()
        ->setParameter("target_frame_name", desiredSetPoints.targetFrameDistance);

    parameterHandler->getGroup("GRAVITY_TASK")
        .lock()
        ->setParameter("target_frame_name", desiredSetPoints.targetFrameGravity);

    // create the IK
    constexpr double jointLimitDelta = 0.5;
    finalizeParameterHandler(parameterHandler,
                             kinDyn,
                             system,
                             Eigen::VectorXd::Constant(kinDyn->model().getNrOfDOFs(),
                                                       jointLimitDelta));
    auto ikTasks = createIKTasks(parameterHandler, kinDyn);

    Eigen::VectorXd weightRegularization, weightDistance, weightGravity;

    auto ik = std::make_shared<QPInverseKinematics>();
    REQUIRE(ik->initialize(parameterHandler));

    REQUIRE(parameterHandler->getGroup("REGULARIZATION_TASK")
                .lock()
                ->getParameter("weight", weightRegularization));
    REQUIRE(ik->addTask(ikTasks.regularizationTask,
                        "regularization_task",
                        lowPriority,
                        weightRegularization));

    REQUIRE(parameterHandler->getGroup("DISTANCE_TASK")
                .lock()
                ->getParameter("weight", //
                               weightDistance));
    REQUIRE(ik->addTask(ikTasks.distanceTask, "distance_task", lowPriority, weightDistance));

    REQUIRE(parameterHandler->getGroup("GRAVITY_TASK")
                .lock()
                ->getParameter("weight", //
                               weightGravity));
    REQUIRE(ik->addTask(ikTasks.gravityTask, "gravity_task", lowPriority, weightGravity));

    REQUIRE(ik->finalize(variablesHandler));

    REQUIRE(ikTasks.regularizationTask->setSetPoint(desiredSetPoints.joints));

    REQUIRE(ikTasks.distanceTask->setDesiredDistance(desiredSetPoints.targetDistance));

    REQUIRE(ikTasks.gravityTask->setDesiredGravityDirectionInTargetFrame(
        desiredSetPoints.desiredBodyGravity));

    // propagate the inverse kinematics for
    constexpr std::size_t iterations = 30;
    Eigen::Vector3d gravity;
    gravity << 0, 0, -9.81;
    Eigen::Matrix4d baseTransform = Eigen::Matrix4d::Identity();
    Eigen::Matrix<double, 6, 1> baseVelocity = Eigen::Matrix<double, 6, 1>::Zero();
    Eigen::VectorXd jointVelocity = Eigen::VectorXd::Zero(model.getNrOfDOFs());

    for (std::size_t iteration = 0; iteration < iterations; iteration++)
    {
        // get the solution of the integrator
        const auto& [basePosition, baseRotation, jointPosition] = system.integrator->getSolution();

        // update the KinDynComputations object
        baseTransform.topLeftCorner<3, 3>() = baseRotation.rotation();
        baseTransform.topRightCorner<3, 1>() = basePosition;
        REQUIRE(kinDyn->setRobotState(baseTransform,
                                      jointPosition,
                                      baseVelocity,
                                      jointVelocity,
                                      gravity));

        // solve the IK
        REQUIRE(ik->advance());

        // get the output of the IK
        baseVelocity = ik->getOutput().baseVelocity.coeffs();
        jointVelocity = ik->getOutput().jointVelocity;

        // propagate the dynamical system
        system.dynamics->setControlInput({baseVelocity, jointVelocity});
        system.integrator->integrate(0s, dT);
    }

    // Check the distance error
    REQUIRE(toManifPose(kinDyn->getWorldTransform(desiredSetPoints.targetFrameDistance))
                .translation()
                .norm()
            == Catch::Approx(desiredSetPoints.targetDistance).epsilon(tolerance));

    // Check the gravity error
    Eigen::Vector3d gravityError;
    gravityError = toManifPose(kinDyn->getWorldTransform(desiredSetPoints.targetFrameGravity))
                       .rotation()
                       .matrix()
                       .row(2)
                       .transpose()
                   - desiredSetPoints.desiredBodyGravity;
    REQUIRE(gravityError.isZero(tolerance));
}
