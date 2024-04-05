/**
 * @file QPFixedBaseTSIDTest.cpp
 * @authors Ines Sorrentino
 * @copyright 2021 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the BSD-3-Clause license.
 */

// Catch2
#include <catch2/catch_test_macros.hpp>

// std
#include <chrono>
#include <memory>
#include <random>

// BipedalLocomotion
#include <BipedalLocomotion/ContinuousDynamicalSystem/FixedBaseDynamics.h>
#include <BipedalLocomotion/ContinuousDynamicalSystem/ForwardEuler.h>
#include <BipedalLocomotion/Conversions/ManifConversions.h>
#include <BipedalLocomotion/ParametersHandler/StdImplementation.h>
#include <BipedalLocomotion/System/ConstantWeightProvider.h>
#include <BipedalLocomotion/System/VariablesHandler.h>
#include <BipedalLocomotion/TSID/JointTrackingTask.h>
#include <BipedalLocomotion/TSID/QPFixedBaseTSID.h>
#include <BipedalLocomotion/TSID/SE3Task.h>

#include <iDynTree/EigenHelpers.h>
#include <iDynTree/ModelTestUtils.h>

using namespace BipedalLocomotion::ParametersHandler;
using namespace BipedalLocomotion::System;
using namespace BipedalLocomotion::ContinuousDynamicalSystem;
using namespace BipedalLocomotion::Conversions;
using namespace BipedalLocomotion::TSID;
using namespace std::chrono_literals;

constexpr auto robotAcceleration = "robotAccelration";
constexpr auto jointTorques = "jointTorques";
constexpr int maxNumOfContacts = 0;
constexpr std::chrono::nanoseconds dT = 10ms;

struct TSIDAndTasks
{
    std::shared_ptr<QPFixedBaseTSID> tsid;
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
    std::shared_ptr<ForwardEuler<FixedBaseDynamics>> integrator;
    std::shared_ptr<FixedBaseDynamics> dynamics;
};

std::shared_ptr<IParametersHandler> createParameterHandler()
{
    auto parameterHandler = std::make_shared<StdImplementation>();
    parameterHandler->setParameter("robot_acceleration_variable_name", robotAcceleration);
    parameterHandler->setParameter("joint_torques_variable_name", jointTorques);
    parameterHandler->setParameter("max_number_of_contacts", maxNumOfContacts);

    parameterHandler->setParameter("verbosity", false);

    /////// SE3 Task
    constexpr double kp_se3task = 300.0;
    const double kd_se3task = 2*std::sqrt(kp_se3task);
    auto SE3ParameterHandler = std::make_shared<StdImplementation>();

    SE3ParameterHandler->setParameter("robot_acceleration_variable_name",
                                      robotAcceleration);
    SE3ParameterHandler->setParameter("kp_linear", kp_se3task);
    SE3ParameterHandler->setParameter("kd_linear", kd_se3task);
    SE3ParameterHandler->setParameter("kp_angular", kp_se3task);
    SE3ParameterHandler->setParameter("kd_angular", kd_se3task);

    parameterHandler->setGroup("EE_SE3_TASK", SE3ParameterHandler);

    /////// Joint regularization task
    auto jointRegularizationHandler = std::make_shared<StdImplementation>();
    jointRegularizationHandler->setParameter("robot_acceleration_variable_name",
                                             robotAcceleration);

    parameterHandler->setGroup("REGULARIZATION_TASK", jointRegularizationHandler);

    return parameterHandler;
}

TSIDAndTasks createTSID(std::shared_ptr<IParametersHandler> handler,
                        std::shared_ptr<iDynTree::KinDynComputations> kinDyn,
                        const VariablesHandler& variables)
{
    // prepare the parameters related to the size of the system
    const Eigen::VectorXd kpRegularization = 1 * Eigen::VectorXd::Ones(kinDyn->model().getNrOfDOFs());
    const Eigen::VectorXd kdRegularization = 2 * Eigen::VectorXd::Ones(kinDyn->model().getNrOfDOFs());
    const Eigen::VectorXd weightRegularization = 1 * Eigen::VectorXd::Ones(kinDyn->model().getNrOfDOFs());

    handler->getGroup("REGULARIZATION_TASK").lock()->setParameter("kp", kpRegularization);
    handler->getGroup("REGULARIZATION_TASK").lock()->setParameter("kd", kdRegularization);

    TSIDAndTasks out;

    constexpr std::size_t lowPriority = 1;
    constexpr std::size_t highPriority = 0;

    out.tsid = std::make_shared<QPFixedBaseTSID>();
    REQUIRE(out.tsid->setKinDyn(kinDyn));
    REQUIRE(out.tsid->initialize(handler));

    out.se3Task = std::make_shared<SE3Task>();
    REQUIRE(out.se3Task->setKinDyn(kinDyn));
    REQUIRE(out.se3Task->initialize(handler->getGroup("EE_SE3_TASK")));
    REQUIRE(out.tsid->addTask(out.se3Task, "se3_task", highPriority));

    out.regularizationTask = std::make_shared<JointTrackingTask>();
    REQUIRE(out.regularizationTask->setKinDyn(kinDyn));
    REQUIRE(out.regularizationTask->initialize(handler->getGroup("REGULARIZATION_TASK")));
    REQUIRE(out.tsid->addTask(out.regularizationTask,
                            "regularization_task",
                            lowPriority,
                            weightRegularization));

    Eigen::VectorXd newWeight = 10 * weightRegularization;
    REQUIRE(out.tsid->setTaskWeight("regularization_task",
                                    std::make_shared<
                                    BipedalLocomotion::System::ConstantWeightProvider>(
                                        newWeight)));
    auto provider = out.tsid->getTaskWeightProvider("regularization_task").lock();
    REQUIRE(provider);
    REQUIRE(provider->getOutput().isApprox(newWeight));
    REQUIRE(out.tsid->setTaskWeight("regularization_task", weightRegularization));

    REQUIRE(out.tsid->finalize(variables));

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

    iDynTree::toEigen(gravity) << 0, 0, -BipedalLocomotion::Math::StandardAccelerationOfGravitation;

    REQUIRE(kinDyn->setRobotState(worldBasePos, jointsPos, baseVel, jointsVel, gravity));

    // end-effector pose
    const std::string controlledFrame = kinDyn->model().getFrameName(numberOfJoints);
    out.endEffectorPose = toManifPose(kinDyn->getWorldTransform(controlledFrame));
    out.joints.resize(jointsPos.size());
    out.joints = iDynTree::toEigen(jointsPos);

    return out;
}

System getSystem(std::shared_ptr<iDynTree::KinDynComputations> kinDyn, const iDynTree::Model model)
{
    System out;

    Eigen::VectorXd jointPositions(kinDyn->getNrOfDegreesOfFreedom());
    Eigen::VectorXd jointVelocities(kinDyn->getNrOfDegreesOfFreedom());

    REQUIRE(kinDyn->getJointPos(jointPositions));
    REQUIRE(kinDyn->getJointVel(jointVelocities));

    out.dynamics = std::make_shared<FixedBaseDynamics>();
    out.dynamics->setState({jointVelocities, jointPositions});
    REQUIRE(out.dynamics->setRobotModel(model));

    out.integrator = std::make_shared<ForwardEuler<FixedBaseDynamics>>();
    REQUIRE(out.integrator->setIntegrationStep(dT));
    out.integrator->setDynamicalSystem(out.dynamics);

    return out;
}

void resetKinDyn(std::shared_ptr<iDynTree::KinDynComputations> kinDyn)
{
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

    REQUIRE(kinDyn->setRobotState(basePose, jointPositions, baseVelocity, jointVelocities, gravity));
}

TEST_CASE("QP-TSID")
{
    auto parameterHandler = createParameterHandler();

    constexpr double tolerance = 1e-1;

    // propagate the inverse dynamics for
    constexpr std::size_t iterations = 200;
    Eigen::Vector3d gravity;
    gravity << 0, 0, -BipedalLocomotion::Math::StandardAccelerationOfGravitation;

    for (std::size_t numberOfJoints = 15; numberOfJoints < 40; numberOfJoints += 15)
    {
        DYNAMIC_SECTION("Model with " << numberOfJoints << " joints")
        {
            auto kinDyn = std::make_shared<iDynTree::KinDynComputations>();

            REQUIRE(kinDyn->setFrameVelocityRepresentation(
            iDynTree::FrameVelocityRepresentation::MIXED_REPRESENTATION));

            // create the model
            const iDynTree::Model model = iDynTree::getRandomChain(numberOfJoints);
            REQUIRE(kinDyn->loadRobotModel(model));

            const auto desiredSetPoints = getDesiredReference(kinDyn, numberOfJoints);

            // Instantiate the handler
            VariablesHandler variablesHandler;
            REQUIRE(variablesHandler.addVariable(robotAcceleration, model.getNrOfDOFs() + 6));
            REQUIRE(variablesHandler.addVariable(jointTorques, model.getNrOfDOFs()));

            resetKinDyn(kinDyn);

            auto system = getSystem(kinDyn, model);

            // Set the frame name
            const std::string controlledFrame = model.getFrameName(numberOfJoints);
            parameterHandler->getGroup("EE_SE3_TASK")
                .lock()
                ->setParameter("frame_name", controlledFrame);

            // create the TSID
            auto tsidAndTasks = createTSID(parameterHandler, kinDyn, variablesHandler);

            REQUIRE(tsidAndTasks.se3Task->setSetPoint(desiredSetPoints.endEffectorPose,
                                                      manif::SE3d::Tangent::Zero(),
                                                      manif::SE3d::Tangent::Zero()));
            REQUIRE(tsidAndTasks.regularizationTask->setSetPoint(desiredSetPoints.joints));


            Eigen::Matrix4d baseTransform = Eigen::Matrix4d::Identity();
            Eigen::Matrix<double, 6, 1> baseVelocity = Eigen::Matrix<double, 6, 1>::Zero();

            for (std::size_t iteration = 0; iteration < iterations; iteration++)
            {
                // get the solution of the integrator
                const auto& [jointVelocity, jointPosition] = system.integrator->getSolution();

                // update the KinDynComputations object
                REQUIRE(kinDyn->setRobotState(baseTransform,
                                              jointPosition,
                                              baseVelocity,
                                              jointVelocity,
                                              gravity));

                // solve the TSID
                REQUIRE(tsidAndTasks.tsid->advance());

                // propagate the dynamical system
                system.dynamics->setControlInput({tsidAndTasks.tsid->getOutput().jointTorques});
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
