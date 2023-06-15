/**
 * @file FrictionTorqueDynamicsTest.cpp
 * @authors Ines Sorrentino
 * @copyright 2023 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the BSD-3-Clause license.
 */

#include <catch2/catch_test_macros.hpp>
#include <chrono>

#include <ConfigFolderPath.h>
#include <iCubModels/iCubModels.h>
#include <yarp/os/ResourceFinder.h>

// iDynTree
#include <iDynTree/KinDynComputations.h>
#include <iDynTree/ModelIO/ModelLoader.h>

#include <BipedalLocomotion/ParametersHandler/IParametersHandler.h>
#include <BipedalLocomotion/ParametersHandler/YarpImplementation.h>
#include <BipedalLocomotion/ParametersHandler/StdImplementation.h>
#include <BipedalLocomotion/System/VariablesHandler.h>
#include <BipedalLocomotion/Math/Constants.h>

#include <BipedalLocomotion/RobotDynamicsEstimator/SubModel.h>
#include <BipedalLocomotion/RobotDynamicsEstimator/SubModelKinDynWrapper.h>
#include <BipedalLocomotion/RobotDynamicsEstimator/JointVelocityStateDynamics.h>

using namespace BipedalLocomotion::Estimators::RobotDynamicsEstimator;
using namespace BipedalLocomotion::ParametersHandler;
using namespace BipedalLocomotion::System;

IParametersHandler::shared_ptr loadParameterHandler()
{
    std::shared_ptr<YarpImplementation> originalHandler = std::make_shared<YarpImplementation>();
    IParametersHandler::shared_ptr parameterHandler = originalHandler;

    yarp::os::ResourceFinder& rf = yarp::os::ResourceFinder::getResourceFinderSingleton();
    rf.setDefaultConfigFile("/config/model.ini");

    std::vector<std::string> arguments = {" ", "--from ", getConfigPath()};

    std::vector<char*> argv;
    for (const auto& arg : arguments)
        argv.push_back((char*)arg.data());
    argv.push_back(nullptr);

    rf.configure(argv.size() - 1, argv.data());

    REQUIRE_FALSE(rf.isNull());
    parameterHandler->clear();
    REQUIRE(parameterHandler->isEmpty());
    originalHandler->set(rf);

    auto group = parameterHandler->getGroup("MODEL").lock();
    REQUIRE(group != nullptr);

    return group;
}

void createModelLoader(IParametersHandler::shared_ptr group,
                       iDynTree::ModelLoader& mdlLdr,
                       bool useFT)
{
    // List of joints and fts to load the model
    std::vector<SubModel> subModelList;

    const std::string modelPath = iCubModels::getModelFile("iCubGenova09");

    std::vector<std::string> jointList;
    REQUIRE(group->getParameter("joint_list", jointList));

    std::vector<std::string> jointsAndFTs;
    jointsAndFTs.insert(jointsAndFTs.begin(), jointList.begin(), jointList.end());

    if (useFT)
    {
        std::vector<std::string> ftFramesList;
        auto ftGroup = group->getGroup("FT").lock();
        REQUIRE(ftGroup->getParameter("associated_joints", ftFramesList));
        jointsAndFTs.insert(jointsAndFTs.end(), ftFramesList.begin(), ftFramesList.end());
    }

    REQUIRE(mdlLdr.loadReducedModelFromFile(modelPath, jointsAndFTs));
}

void createSubModels(iDynTree::ModelLoader& mdlLdr,
                     std::shared_ptr<iDynTree::KinDynComputations> kinDynFullModel,
                     IParametersHandler::shared_ptr group,
                     bool useFTSensors,
                     SubModelCreator& subModelCreator)
{
    subModelCreator.setModelAndSensors(mdlLdr.model(), mdlLdr.sensors());
    REQUIRE(subModelCreator.setKinDyn(kinDynFullModel));

    if (useFTSensors)
    {
        REQUIRE(subModelCreator.createSubModels(group));
    }
    else
    {
        auto groupEmpty = group->clone();
        groupEmpty->clear();

        std::vector<std::string> jointList;
        group->getParameter("joint_list", jointList);
        groupEmpty->setParameter("joint_list", jointList);

        std::shared_ptr<IParametersHandler> emptySubGroup = groupEmpty->clone();
        emptySubGroup->clear();
        std::vector<std::string> emptyVector;
        emptySubGroup->setParameter("names", emptyVector);
        emptySubGroup->setParameter("frames", emptyVector);
        emptySubGroup->setParameter("associated_joints", emptyVector);
        groupEmpty->setGroup("FT", emptySubGroup);
        groupEmpty->setGroup("ACCELEROMETER", emptySubGroup);
        groupEmpty->setGroup("GYROSCOPE", emptySubGroup);
        groupEmpty->setGroup("EXTERNAL_CONTACT", emptySubGroup);

        REQUIRE(subModelCreator.createSubModels(groupEmpty));
    }
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

TEST_CASE("Joint Velocity Dynamics Without FT")
{
    using namespace std::chrono_literals;

    // Create parameter handler
    auto parameterHandler = std::make_shared<StdImplementation>();

    const std::string name = "ds";
    Eigen::VectorXd covariance(6);
    covariance << 1e-3, 1e-3, 5e-3, 5e-3, 5e-3, 5e-3;
    std::string model = "JointVelocityStateDynamics";
    const std::vector<std::string> elements = {"r_hip_pitch", "r_hip_roll", "r_hip_yaw", "r_knee", "r_ankle_pitch", "r_ankle_roll"};

    parameterHandler->setParameter("name", name);
    parameterHandler->setParameter("covariance", covariance);
    parameterHandler->setParameter("initial_covariance", covariance);
    parameterHandler->setParameter("dynamic_model", model);
    parameterHandler->setParameter("elements", elements);
    parameterHandler->setParameter("sampling_time", 10ms);

    // Create variable handler
    constexpr size_t sizeVariable = 6;
    VariablesHandler variableHandler;
    REQUIRE(variableHandler.addVariable("ds", sizeVariable));
    REQUIRE(variableHandler.addVariable("tau_m", sizeVariable));
    REQUIRE(variableHandler.addVariable("tau_F", sizeVariable));
    REQUIRE(variableHandler.addVariable("r_leg_ft_acc_bias", 3));
    REQUIRE(variableHandler.addVariable("r_foot_front_ft_acc_bias", 3));
    REQUIRE(variableHandler.addVariable("r_foot_rear_ft_acc_bias", 3));
    REQUIRE(variableHandler.addVariable("r_leg_ft_gyro_bias", 3));
    REQUIRE(variableHandler.addVariable("r_foot_front_ft_gyro_bias", 3));
    REQUIRE(variableHandler.addVariable("r_foot_rear_ft_gyro_bias", 3));

    auto handlerFromConfig = loadParameterHandler();

    bool useFT = false;

    iDynTree::ModelLoader modelLoader;
    createModelLoader(handlerFromConfig, modelLoader, useFT);

    auto kinDyn = std::make_shared<iDynTree::KinDynComputations>();
    REQUIRE(kinDyn->loadRobotModel(modelLoader.model()));

    REQUIRE(setStaticState(kinDyn));

    SubModelCreator subModelCreatorWithFT;
    createSubModels(modelLoader, kinDyn, handlerFromConfig, useFT, subModelCreatorWithFT);

    std::vector<std::shared_ptr<SubModelKinDynWrapper>> kinDynWrapperList;
    const auto & subModelListWithFT = subModelCreatorWithFT.getSubModelList();

    for (int idx = 0; idx < subModelCreatorWithFT.getSubModelList().size(); idx++)
    {
        kinDynWrapperList.emplace_back(std::make_shared<SubModelKinDynWrapper>());
        REQUIRE(kinDynWrapperList.at(idx)->setKinDyn(kinDyn));
        REQUIRE(kinDynWrapperList.at(idx)->initialize(subModelListWithFT[idx]));
    }

    manif::SE3d::Tangent robotBaseAcceleration;
    robotBaseAcceleration.setZero();

    Eigen::VectorXd robotJointAcceleration(kinDyn->model().getNrOfDOFs());
    robotJointAcceleration.setZero();

    iDynTree::LinkNetExternalWrenches extWrench(kinDyn->model());
    extWrench.zero();

    // Compute joint torques in static configuration from inverse dynamics on the full model
    iDynTree::FreeFloatingGeneralizedTorques jointTorques(kinDyn->model());

    kinDyn->inverseDynamics(iDynTree::make_span(robotBaseAcceleration.data(),
                                                manif::SE3d::Tangent::DoF),
                            robotJointAcceleration,
                            extWrench,
                            jointTorques);

    JointVelocityStateDynamics ds;
    model = "JointVelocityStateDynamics";
    parameterHandler->setParameter("dynamic_model", model);
    REQUIRE(ds.setSubModels(subModelListWithFT, kinDynWrapperList));
    REQUIRE(ds.initialize(parameterHandler));
    REQUIRE(ds.finalize(variableHandler));

    Eigen::VectorXd state;
    state.resize(variableHandler.getNumberOfVariables());
    state.setZero();

    int offset = variableHandler.getVariable("tau_m").offset;
    int size = variableHandler.getVariable("tau_m").size;
    for (int jointIndex = 0; jointIndex < size; jointIndex++)
    {
        state[offset+jointIndex] = jointTorques.jointTorques()[jointIndex];
    }

    // Create an input for the ukf state
    UKFInput input;

    // Define joint positions
    Eigen::VectorXd jointPos;
    jointPos.resize(kinDyn->model().getNrOfDOFs());
    jointPos.setZero();
    input.robotJointPositions = jointPos;

    // Define base pose
    manif::SE3d basePose;
    basePose.setIdentity();
    input.robotBasePose = basePose;

    // Define base velocity and acceleration
    manif::SE3d::Tangent baseVelocity, baseAcceleration;
    baseVelocity.setZero();
    baseAcceleration.setZero();
    input.robotBaseVelocity = baseVelocity;
    input.robotBaseAcceleration = baseAcceleration;
    input.robotJointAccelerations = Eigen::VectorXd(kinDyn->model().getNrOfDOFs()).setZero();

    for (int idx = 0; idx < subModelCreatorWithFT.getSubModelList().size(); idx++)
    {
        REQUIRE(kinDynWrapperList.at(idx)->updateState(baseAcceleration,
                                                       input.robotJointAccelerations,
                                                       BipedalLocomotion::Estimators::RobotDynamicsEstimator::UpdateMode::Full));
    }

    ds.setInput(input);

    ds.setState(state);

    REQUIRE(ds.update());
}

TEST_CASE("Joint Velocity Dynamics With FT")
{
    using namespace std::chrono_literals;

    // Create parameter handler
    auto parameterHandler = std::make_shared<StdImplementation>();

    const std::string name = "ds";
    Eigen::VectorXd covariance(6);
    covariance << 1e-3, 1e-3, 5e-3, 5e-3, 5e-3, 5e-3;
    std::string model = "JointVelocityStateDynamics";
    const std::vector<std::string> elements = {"r_hip_pitch", "r_hip_roll", "r_hip_yaw", "r_knee", "r_ankle_pitch", "r_ankle_roll"};

    parameterHandler->setParameter("name", name);
    parameterHandler->setParameter("covariance", covariance);
    parameterHandler->setParameter("initial_covariance", covariance);
    parameterHandler->setParameter("dynamic_model", model);
    parameterHandler->setParameter("elements", elements);
    parameterHandler->setParameter("sampling_time", 10ms);

    // Create variable handler
    constexpr size_t sizeVariable = 6;
    VariablesHandler variableHandler;
    REQUIRE(variableHandler.addVariable("ds", sizeVariable));
    REQUIRE(variableHandler.addVariable("tau_m", sizeVariable));
    REQUIRE(variableHandler.addVariable("tau_F", sizeVariable));
    REQUIRE(variableHandler.addVariable("r_leg_ft", 6));
    REQUIRE(variableHandler.addVariable("r_foot_front_ft", 6));
    REQUIRE(variableHandler.addVariable("r_foot_rear_ft", 6));
    REQUIRE(variableHandler.addVariable("r_leg_ft_bias", 6));
    REQUIRE(variableHandler.addVariable("r_foot_front_ft_bias", 6));
    REQUIRE(variableHandler.addVariable("r_foot_rear_ft_bias", 6));
    REQUIRE(variableHandler.addVariable("r_leg_ft_acc_bias", 3));
    REQUIRE(variableHandler.addVariable("r_foot_front_ft_acc_bias", 3));
    REQUIRE(variableHandler.addVariable("r_foot_rear_ft_acc_bias", 3));
    REQUIRE(variableHandler.addVariable("r_leg_ft_gyro_bias", 3));
    REQUIRE(variableHandler.addVariable("r_foot_front_ft_gyro_bias", 3));
    REQUIRE(variableHandler.addVariable("r_foot_rear_ft_gyro_bias", 3));

    auto handlerFromConfig = loadParameterHandler();

    iDynTree::ModelLoader modelLoader;
    createModelLoader(handlerFromConfig, modelLoader, true);

    auto kinDyn = std::make_shared<iDynTree::KinDynComputations>();
    REQUIRE(kinDyn->loadRobotModel(modelLoader.model()));

    REQUIRE(setStaticState(kinDyn));

    bool useFT = true;

    SubModelCreator subModelCreatorWithFT;
    createSubModels(modelLoader, kinDyn, handlerFromConfig, useFT, subModelCreatorWithFT);

    std::vector<std::shared_ptr<SubModelKinDynWrapper>> kinDynWrapperListWithFT;
    const auto & subModelListWithFT = subModelCreatorWithFT.getSubModelList();

    for (int idx = 0; idx < subModelCreatorWithFT.getSubModelList().size(); idx++)
    {
        kinDynWrapperListWithFT.emplace_back(std::make_shared<SubModelKinDynWrapper>());
        REQUIRE(kinDynWrapperListWithFT.at(idx)->setKinDyn(kinDyn));
        REQUIRE(kinDynWrapperListWithFT.at(idx)->initialize(subModelListWithFT[idx]));
    }

    manif::SE3d::Tangent robotBaseAcceleration;
    robotBaseAcceleration.setZero();

    Eigen::VectorXd robotJointAcceleration(kinDyn->model().getNrOfDOFs());
    robotJointAcceleration.setZero();

    iDynTree::LinkNetExternalWrenches extWrench(kinDyn->model());
    extWrench.zero();

    // Compute joint torques in static configuration from inverse dynamics on the full model
    iDynTree::FreeFloatingGeneralizedTorques jointTorques(kinDyn->model());

    kinDyn->inverseDynamics(iDynTree::make_span(robotBaseAcceleration.data(),
                                                robotBaseAcceleration.coeffs().size()),
                            robotJointAcceleration,
                            extWrench,
                            jointTorques);

    JointVelocityStateDynamics dsSplit;
    model = "JointVelocityStateDynamics";
    parameterHandler->setParameter("dynamic_model", model);
    REQUIRE(dsSplit.setSubModels(subModelListWithFT, kinDynWrapperListWithFT));
    REQUIRE(dsSplit.initialize(parameterHandler));
    REQUIRE(dsSplit.finalize(variableHandler));

    Eigen::VectorXd state;
    state.resize(variableHandler.getNumberOfVariables());
    state.setZero();

    int offset = variableHandler.getVariable("tau_m").offset;
    int size = variableHandler.getVariable("tau_m").size;
    for (int jointIndex = 0; jointIndex < size; jointIndex++)
    {
        state[offset+jointIndex] = jointTorques.jointTorques()[jointIndex];
    }

    auto massSecondSubModel = subModelListWithFT.at(1).getModel().getTotalMass();
    state(variableHandler.getVariable("r_leg_ft").offset+2) = massSecondSubModel * BipedalLocomotion::Math::StandardAccelerationOfGravitation;

    // Create an input for the ukf state
    UKFInput input;

    // Define joint positions
    Eigen::VectorXd jointPos;
    jointPos.resize(kinDyn->model().getNrOfDOFs());
    jointPos.setZero();
    input.robotJointPositions = jointPos;

    // Define base pose
    manif::SE3d basePose;
    basePose.setIdentity();
    input.robotBasePose = basePose;

    // Define base velocity and acceleration
    manif::SE3d::Tangent baseVelocity, baseAcceleration;
    baseVelocity.setZero();
    baseAcceleration.setZero();
    input.robotBaseVelocity = baseVelocity;
    input.robotBaseAcceleration = baseAcceleration;

    for (int idx = 0; idx < subModelCreatorWithFT.getSubModelList().size(); idx++)
    {
        REQUIRE(kinDynWrapperListWithFT.at(idx)->updateState(baseAcceleration,
                                                             Eigen::VectorXd(kinDyn->model().getNrOfDOFs()).setZero(),
                                                             BipedalLocomotion::Estimators::RobotDynamicsEstimator::UpdateMode::RobotDynamicsOnly));
    }

    input.robotJointAccelerations = Eigen::VectorXd(kinDyn->model().getNrOfDOFs()).setZero();

    dsSplit.setInput(input);

    dsSplit.setState(state);

    REQUIRE(dsSplit.update());
}
