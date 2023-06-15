/**
 * @file QPInverseKinematicsTest.cpp
 * @authors Giulio Romualdi
 * @copyright 2021 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the BSD-3-Clause license.
 */

// Catch2
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
#include <BipedalLocomotion/IK/IntegrationBasedIK.h>
#include <BipedalLocomotion/IK/JointTrackingTask.h>
#include <BipedalLocomotion/IK/JointLimitsTask.h>
#include <BipedalLocomotion/IK/QPInverseKinematics.h>
#include <BipedalLocomotion/IK/SE3Task.h>

#include <BipedalLocomotion/ContinuousDynamicalSystem/FloatingBaseSystemKinematics.h>
#include <BipedalLocomotion/ContinuousDynamicalSystem/ForwardEuler.h>

#include <iDynTree/Core/EigenHelpers.h>
#include <iDynTree/Model/ModelTestUtils.h>

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
    std::shared_ptr<IntegrationBasedIK> ik;
    std::shared_ptr<SE3Task> se3Task;
    std::shared_ptr<CoMTask> comTask;
    std::shared_ptr<JointTrackingTask> regularizationTask;
    std::shared_ptr<JointLimitsTask> jointLimitsTask;
};

struct DesiredSetPoints
{
    Eigen::Vector3d CoMPosition;
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

    constexpr double gain = 5.0;

    auto parameterHandler = std::make_shared<StdImplementation>();
    parameterHandler->setParameter("robot_velocity_variable_name", robotVelocity);
    parameterHandler->setParameter("verbosity", false);

    parameterHandler->setParameter("tasks",
                                   std::vector<std::string>{"SE3_TASK",
                                                            "COM_TASK",
                                                            "REGULARIZATION_TASK",
                                                            "JOINT_LIMITS_TASK"});

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


    auto jointLimitsHandler = std::make_shared<StdImplementation>();
    jointLimitsHandler->setParameter("robot_velocity_variable_name", robotVelocity);
    jointLimitsHandler->setParameter("sampling_time", dT);
    jointLimitsHandler->setParameter("use_model_limits", false);
    jointLimitsHandler->setParameter("type", "JointLimitsTask");
    jointLimitsHandler->setParameter("priority", 0);
    parameterHandler->setGroup("JOINT_LIMITS_TASK", jointLimitsHandler);

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

InverseKinematicsAndTasks createIK(std::shared_ptr<IParametersHandler> handler,
                                   std::shared_ptr<iDynTree::KinDynComputations> kinDyn,
                                   const VariablesHandler& variables)
{

    InverseKinematicsAndTasks out;

    constexpr std::size_t highPriority = 0;
    constexpr std::size_t lowPriority = 1;
    Eigen::VectorXd kpRegularization;
    Eigen::VectorXd weightRegularization;

    out.ik = std::make_shared<QPInverseKinematics>();
    REQUIRE(out.ik->initialize(handler));

    out.se3Task = std::make_shared<SE3Task>();
    REQUIRE(out.se3Task->setKinDyn(kinDyn));
    REQUIRE(out.se3Task->initialize(handler->getGroup("SE3_TASK")));
    REQUIRE(out.ik->addTask(out.se3Task, "se3_task", highPriority));

    out.comTask = std::make_shared<CoMTask>();
    REQUIRE(out.comTask->setKinDyn(kinDyn));
    REQUIRE(out.comTask->initialize(handler->getGroup("COM_TASK")));
    REQUIRE(out.ik->addTask(out.comTask, "com_task", highPriority));

    REQUIRE(handler->getGroup("REGULARIZATION_TASK").lock()->getParameter("kp", kpRegularization));
    REQUIRE(handler->getGroup("REGULARIZATION_TASK").lock()->getParameter("weight", weightRegularization));
    out.regularizationTask = std::make_shared<JointTrackingTask>();
    REQUIRE(out.regularizationTask->setKinDyn(kinDyn));
    REQUIRE(out.regularizationTask->initialize(handler->getGroup("REGULARIZATION_TASK")));
    REQUIRE(out.ik->addTask(out.regularizationTask,
                            "regularization_task",
                            lowPriority,
                            weightRegularization));

    out.jointLimitsTask = std::make_shared<JointLimitsTask>();
    REQUIRE(out.jointLimitsTask->setKinDyn(kinDyn));
    REQUIRE(out.jointLimitsTask->initialize(handler->getGroup("JOINT_LIMITS_TASK")));
    REQUIRE(out.ik->addTask(out.jointLimitsTask, "joint_limits_task", highPriority));

    const Eigen::VectorXd newWeight = 10 * weightRegularization;
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
    std::normal_distribution<double> distribution(0.0, 0.1);

    // create the System
    Eigen::Matrix<double, 6, 1> baseVelocity;
    Eigen::VectorXd jointVelocities(kinDyn->getNrOfDegreesOfFreedom());
    Eigen::Matrix4d basePose;
    Eigen::VectorXd jointPositions(kinDyn->getNrOfDegreesOfFreedom());
    Eigen::Vector3d gravity;

    REQUIRE(kinDyn->getRobotState(basePose, jointPositions, baseVelocity, jointVelocities, gravity));

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
            const iDynTree::Model model = iDynTree::getRandomModel(numberOfJoints);
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
            constexpr double jointLimitDelta = 0.5;
            finalizeParameterHandler(parameterHandler,
                                     kinDyn,
                                     system,
                                     Eigen::VectorXd::Constant(kinDyn->model().getNrOfDOFs(),
                                                               jointLimitDelta));
            auto ikAndTasks = createIK(parameterHandler, kinDyn, variablesHandler);

            REQUIRE(ikAndTasks.se3Task->setSetPoint(desiredSetPoints.endEffectorPose,
                                                    manif::SE3d::Tangent::Zero()));
            REQUIRE(ikAndTasks.comTask->setSetPoint(desiredSetPoints.CoMPosition,
                                                    Eigen::Vector3d::Zero()));
            REQUIRE(ikAndTasks.regularizationTask->setSetPoint(desiredSetPoints.joints));

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
                REQUIRE(ikAndTasks.ik->advance());

                // get the output of the IK
                baseVelocity = ikAndTasks.ik->getOutput().baseVelocity.coeffs();
                jointVelocity = ikAndTasks.ik->getOutput().jointVelocity;

                // propagate the dynamical system
                system.dynamics->setControlInput({baseVelocity, jointVelocity});
                system.integrator->integrate(0s, dT);
            }

            // check the CoM position
            REQUIRE(desiredSetPoints.CoMPosition.isApprox(toEigen(kinDyn->getCenterOfMassPosition()),
                                                          tolerance));

            // Check the end-effector pose error
            const manif::SE3d endEffectorPose  = toManifPose(kinDyn->getWorldTransform(controlledFrame));

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
    const std::string controlledFrame = model.getFrameName(numberOfJoints);
    parameterHandler->getGroup("SE3_TASK").lock()->setParameter("frame_name", controlledFrame);

    // create the IK
    constexpr double jointLimitDelta = 0.5;
    Eigen::VectorXd limitsDelta
        = Eigen::VectorXd::Constant(kinDyn->model().getNrOfDOFs(), jointLimitDelta);

    // we limit the first and the forth joints.
    limitsDelta(0) = 0;
    limitsDelta(3) = 0;

    finalizeParameterHandler(parameterHandler, kinDyn, system, limitsDelta);
    auto ikAndTasks = createIK(parameterHandler, kinDyn, variablesHandler);

    REQUIRE(ikAndTasks.se3Task->setSetPoint(desiredSetPoints.endEffectorPose,
                                            manif::SE3d::Tangent::Zero()));
    REQUIRE(ikAndTasks.comTask->setSetPoint(desiredSetPoints.CoMPosition, Eigen::Vector3d::Zero()));
    REQUIRE(ikAndTasks.regularizationTask->setSetPoint(desiredSetPoints.joints));

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
        REQUIRE(ikAndTasks.ik->advance());

        // get the output of the IK
        baseVelocity = ikAndTasks.ik->getOutput().baseVelocity.coeffs();
        jointVelocity = ikAndTasks.ik->getOutput().jointVelocity;

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

    constexpr double tolerance = 1e-1;

    // set the velocity representation
    REQUIRE(kinDyn->setFrameVelocityRepresentation(
        iDynTree::FrameVelocityRepresentation::MIXED_REPRESENTATION));

    constexpr std::size_t numberOfJoints = 30;

    // create the model
    const iDynTree::Model model = iDynTree::getRandomModel(numberOfJoints);
    REQUIRE(kinDyn->loadRobotModel(model));

    /////// VariableHandler and IK params
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
    const std::string controlledFrame = model.getFrameName(numberOfJoints);
    parameterHandler->getGroup("SE3_TASK").lock()->setParameter("frame_name", controlledFrame);

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
        jointVelocity =ik->getOutput().jointVelocity;

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
    const manif::SE3d endEffectorPose = toManifPose(kinDyn->getWorldTransform(controlledFrame));

    // please read it as (log(desiredSetPoints.endEffectorPose^-1 * endEffectorPose))^v
    const manif::SE3d::Tangent error = endEffectorPose - desiredSetPoints.endEffectorPose;
    REQUIRE(error.coeffs().isZero(tolerance));
}
