/**
 * @file JointVelocityStateDynamicsTest.cpp
 * @authors Ines Sorrentino
 * @copyright 2023 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the BSD-3-Clause license.
 */

#include <catch2/catch_test_macros.hpp>
#include <catch2/generators/catch_generators_all.hpp>
#include <chrono>

#include <iCubModels/iCubModels.h>
#include <yarp/os/ResourceFinder.h>

#include <iDynTree/KinDynComputations.h>
#include <iDynTree/FreeFloatingState.h>
#include <iDynTree/Model.h>
#include <iDynTree/ModelTestUtils.h>
#include <iDynTree/ModelLoader.h>

#include <BipedalLocomotion/Conversions/ManifConversions.h>
#include <BipedalLocomotion/Math/Constants.h>
#include <BipedalLocomotion/ParametersHandler/IParametersHandler.h>
#include <BipedalLocomotion/ParametersHandler/StdImplementation.h>
#include <BipedalLocomotion/System/VariablesHandler.h>

#include <BipedalLocomotion/RobotDynamicsEstimator/JointVelocityStateDynamics.h>

using namespace BipedalLocomotion::Estimators::RobotDynamicsEstimator;
using namespace BipedalLocomotion::ParametersHandler;
using namespace BipedalLocomotion::System;

void createModelLoader(IParametersHandler::shared_ptr group, iDynTree::ModelLoader& mdlLdr)
{
    // List of joints and fts to load the model
    std::vector<SubModel> subModelList;

    const std::string modelPath = iCubModels::getModelFile("iCubGenova09");

    std::vector<std::string> jointList;
    REQUIRE(group->getParameter("joint_list", jointList));

    std::vector<std::string> ftFramesList;
    auto ftGroup = group->getGroup("FT").lock();
    REQUIRE(ftGroup->getParameter("associated_joints", ftFramesList));

    std::vector<std::string> jointsAndFTs;
    jointsAndFTs.insert(jointsAndFTs.begin(), jointList.begin(), jointList.end());

    REQUIRE(mdlLdr.loadReducedModelFromFile(modelPath, jointsAndFTs));
}

void createSubModels(iDynTree::ModelLoader& mdlLdr,
                     std::shared_ptr<iDynTree::KinDynComputations> kinDynFullModel,
                     IParametersHandler::shared_ptr group,
                     SubModelCreator& subModelCreator)
{
    subModelCreator.setModelAndSensors(mdlLdr.model(), mdlLdr.sensors());
    REQUIRE(subModelCreator.setKinDyn(kinDynFullModel));

    REQUIRE(subModelCreator.createSubModels(group));
}

bool setStaticState(std::shared_ptr<iDynTree::KinDynComputations> kinDyn)
{
    size_t dofs = kinDyn->getNrOfDegreesOfFreedom();
    iDynTree::Transform worldTbase;
    Eigen::VectorXd baseVel(6);
    Eigen::Vector3d gravity;

    Eigen::VectorXd qj(dofs), dqj(dofs), ddqj(dofs);

    worldTbase = iDynTree::Transform(iDynTree::Rotation::Identity(), iDynTree::Position::Zero());

    Eigen::Matrix4d transform = toEigen(worldTbase.asHomogeneousTransform());

    gravity.setZero();
    gravity(2) = -BipedalLocomotion::Math::StandardAccelerationOfGravitation;

    baseVel.setZero();

    for (size_t dof = 0; dof < dofs; dof++)
    {
        qj(dof) = 0.0;
        dqj(dof) = 0.0;
        ddqj(dof) = 0.0;
    }

    return kinDyn->setRobotState(transform,
                                 iDynTree::make_span(qj.data(), qj.size()),
                                 iDynTree::make_span(baseVel.data(), baseVel.size()),
                                 iDynTree::make_span(dqj.data(), dqj.size()),
                                 iDynTree::make_span(gravity.data(), gravity.size()));
}

IParametersHandler::shared_ptr createModelParameterHandler()
{
    // Create model variable handler to load the robot model
    auto modelParamHandler = std::make_shared<StdImplementation>();

    const std::vector<std::string> jointList
        = {"r_hip_pitch", "r_hip_roll", "r_hip_yaw", "r_knee", "r_ankle_pitch", "r_ankle_roll"};

    auto emptyGroupNamesFrames = std::make_shared<StdImplementation>();
    std::vector<std::string> emptyVectorString;
    emptyGroupNamesFrames->setParameter("names", emptyVectorString);
    emptyGroupNamesFrames->setParameter("frames", emptyVectorString);
    emptyGroupNamesFrames->setParameter("ukf_names", emptyVectorString);
    emptyGroupNamesFrames->setParameter("associated_joints", emptyVectorString);
    REQUIRE(modelParamHandler->setGroup("FT", emptyGroupNamesFrames));
    REQUIRE(modelParamHandler->setGroup("GYROSCOPE", emptyGroupNamesFrames));
    REQUIRE(modelParamHandler->setGroup("EXTERNAL_CONTACT", emptyGroupNamesFrames));

    auto accGroup = std::make_shared<StdImplementation>();
    std::vector<std::string> accNameList = {"r_leg_ft_acc"};
    std::vector<std::string> accFrameList = {"r_leg_ft"};
    std::vector<std::string> accUkfNameList = {"r_leg_ft_acc"};
    accGroup->setParameter("names", accNameList);
    accGroup->setParameter("frames", accFrameList);
    accGroup->setParameter("ukf_names", accUkfNameList);
    REQUIRE(modelParamHandler->setGroup("ACCELEROMETER", accGroup));

    modelParamHandler->setParameter("joint_list", jointList);

    return modelParamHandler;
}

void createUkfInput(VariablesHandler& stateVariableHandler, UKFInput& input)
{
    Eigen::VectorXd jointPos = Eigen::VectorXd::Random(stateVariableHandler.getVariable("JOINT_VELOCITIES").size);
    input.robotJointPositions = jointPos;

    Eigen::VectorXd jointAcc = Eigen::VectorXd::Random(stateVariableHandler.getVariable("JOINT_VELOCITIES").size);
    input.robotJointAccelerations = jointAcc;

    manif::SE3d basePose
        = BipedalLocomotion::Conversions::toManifPose(iDynTree::getRandomTransform());
    input.robotBasePose = basePose;

    manif::SE3Tangentd baseVel
        = BipedalLocomotion::Conversions::toManifTwist(iDynTree::getRandomTwist());
    input.robotBaseVelocity = baseVel;

    manif::SE3Tangentd baseAcc
        = BipedalLocomotion::Conversions::toManifTwist(iDynTree::getRandomTwist());
    input.robotBaseAcceleration = baseAcc;
}

void createStateVector(UKFInput& input,
                       VariablesHandler& stateVariableHandler,
                       std::shared_ptr<iDynTree::KinDynComputations> kinDyn,
                       Eigen::Ref<Eigen::VectorXd> state)
{
    state.setZero();

    Eigen::VectorXd jointVel = Eigen::VectorXd::Random(stateVariableHandler.getVariable("JOINT_VELOCITIES").size);

    int offset = stateVariableHandler.getVariable("JOINT_VELOCITIES").offset;
    int size = stateVariableHandler.getVariable("JOINT_VELOCITIES").size;
    for (int jointIndex = 0; jointIndex < size; jointIndex++)
    {
        state[offset + jointIndex] = jointVel(jointIndex);
    }

    // Compute joint torques from inverse dynamics on the full model
    offset = stateVariableHandler.getVariable("MOTOR_TORQUES").offset;
    size = stateVariableHandler.getVariable("MOTOR_TORQUES").size;
    iDynTree::LinkNetExternalWrenches extWrench(kinDyn->model());
    extWrench.zero();
    iDynTree::FreeFloatingGeneralizedTorques jointTorques(kinDyn->model());
    kinDyn->inverseDynamics(iDynTree::make_span(input.robotBaseAcceleration.data(),
                                                manif::SE3d::Tangent::DoF),
                            input.robotJointAccelerations,
                            extWrench,
                            jointTorques);
    for (int jointIndex = 0; jointIndex < size; jointIndex++)
    {
        state[offset + jointIndex] = jointTorques.jointTorques()[jointIndex];
    }
}

TEST_CASE("Joint Velocity State Dynamics")
{
    // Create parameter handler
    auto jointVelHandler = std::make_shared<StdImplementation>();

    const std::string name = "JOINT_VELOCITIES";
    Eigen::VectorXd covariance(6);
    covariance << 1e-3, 1e-3, 5e-3, 5e-3, 5e-3, 5e-3;
    const std::string model = "JointVelocityStateDynamics";
    using namespace std::chrono_literals;
    constexpr std::chrono::nanoseconds dT = 10ms;

    jointVelHandler->setParameter("name", name);
    jointVelHandler->setParameter("covariance", covariance);
    jointVelHandler->setParameter("initial_covariance", covariance);
    jointVelHandler->setParameter("dynamic_model", model);
    jointVelHandler->setParameter("sampling_time", dT);

    // Create state variable handler
    constexpr size_t sizeVariable = 6;
    VariablesHandler stateVariableHandler;
    REQUIRE(stateVariableHandler.addVariable("JOINT_VELOCITIES", sizeVariable));
    REQUIRE(stateVariableHandler.addVariable("MOTOR_TORQUES", sizeVariable));
    REQUIRE(stateVariableHandler.addVariable("FRICTION_TORQUES", sizeVariable));

    // Create parameter handler to load the model
    auto modelParamHandler = createModelParameterHandler();

    // Load model
    iDynTree::ModelLoader modelLoader;
    createModelLoader(modelParamHandler, modelLoader);

    auto kinDyn = std::make_shared<iDynTree::KinDynComputations>();
    REQUIRE(kinDyn->loadRobotModel(modelLoader.model()));
    REQUIRE(kinDyn->setFrameVelocityRepresentation(iDynTree::BODY_FIXED_REPRESENTATION));

    REQUIRE(setStaticState(kinDyn));

    SubModelCreator subModelCreator;
    createSubModels(modelLoader, kinDyn, modelParamHandler, subModelCreator);

    std::vector<std::shared_ptr<KinDynWrapper>> kinDynWrapperList;
    std::vector<SubModel> subModelList = subModelCreator.getSubModelList();

    for (int idx = 0; idx < subModelCreator.getSubModelList().size(); idx++)
    {
        kinDynWrapperList.emplace_back(std::make_shared<KinDynWrapper>());
        REQUIRE(kinDynWrapperList[idx]->setModel(subModelList[idx]));
    }

    // Create the dynamics
    JointVelocityStateDynamics dsDynamics;
    REQUIRE(dsDynamics.setSubModels(subModelList, kinDynWrapperList));
    REQUIRE(dsDynamics.initialize(jointVelHandler, "JOINT_VELOCITIES"));
    REQUIRE(dsDynamics.finalize(stateVariableHandler));

    // Create an input for the ukf state
    UKFInput input;
    createUkfInput(stateVariableHandler, input);

    // Create the state vector
    Eigen::VectorXd state;
    state.resize(stateVariableHandler.getNumberOfVariables());
    createStateVector(input, stateVariableHandler, kinDyn, state);

    // Set input and state to the dynamics
    dsDynamics.setInput(input);
    dsDynamics.setState(state);

    // Update the dynamics
    REQUIRE(dsDynamics.update());

    // Compute joint velocity at step k+1 by using the numerical integration
    Eigen::VectorXd jointVel = state.segment(stateVariableHandler.getVariable("JOINT_VELOCITIES").offset,
                                             stateVariableHandler.getVariable("JOINT_VELOCITIES").size);
    jointVel = jointVel + std::chrono::duration<double>(dT).count() * input.robotJointAccelerations;

    // Check the output
    for (int idx = 0; idx < dsDynamics.getUpdatedVariable().size(); idx++)
    {
        REQUIRE(std::abs(dsDynamics.getUpdatedVariable()(idx)
                         - jointVel(idx))
                < 0.1);
    }
}
