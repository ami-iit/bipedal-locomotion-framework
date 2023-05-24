/**
 * @file UkfStateTest.cpp
 * @authors Ines Sorrentino
 * @copyright 2023 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the BSD-3-Clause license.
 */

#include <ConfigFolderPath.h>
#include <catch2/catch_test_macros.hpp>
#include <iCubModels/iCubModels.h>
#include <yarp/os/ResourceFinder.h>

// BLF
#include <BipedalLocomotion/ParametersHandler/IParametersHandler.h>
#include <BipedalLocomotion/ParametersHandler/YarpImplementation.h>
#include <BipedalLocomotion/System/VariablesHandler.h>

// iDynTree
#include <iDynTree/KinDynComputations.h>
#include <iDynTree/ModelIO/ModelLoader.h>

// LIBRARYTOTEST
#include <BipedalLocomotion/RobotDynamicsEstimator/Dynamics.h>
#include <BipedalLocomotion/RobotDynamicsEstimator/SubModel.h>
#include <BipedalLocomotion/RobotDynamicsEstimator/SubModelKinDynWrapper.h>
#include <BipedalLocomotion/RobotDynamicsEstimator/UkfState.h>

using namespace BipedalLocomotion::ParametersHandler;
using namespace BipedalLocomotion::System;
using namespace BipedalLocomotion::Estimators::RobotDynamicsEstimator;

IParametersHandler::shared_ptr loadParameterHandler()
{
    std::shared_ptr<YarpImplementation> originalHandler = std::make_shared<YarpImplementation>();
    IParametersHandler::shared_ptr parameterHandler = originalHandler;

    yarp::os::ResourceFinder& rf = yarp::os::ResourceFinder::getResourceFinderSingleton();
    rf.setDefaultConfigFile("/config/config.ini");

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

    return parameterHandler;
}

void createModelLoader(IParametersHandler::weak_ptr group, iDynTree::ModelLoader& mdlLdr)
{
    // List of joints and fts to load the model
    std::vector<SubModel> subModelList;

    const std::string modelPath = iCubModels::getModelFile("iCubGenova09");

    std::vector<std::string> jointList;
    REQUIRE(group.lock()->getParameter("joint_list", jointList));

    std::vector<std::string> ftFramesList;
    auto ftGroup = group.lock()->getGroup("FT").lock();
    REQUIRE(ftGroup->getParameter("associated_joints", ftFramesList));

    std::vector<std::string> jointsAndFTs;
    jointsAndFTs.insert(jointsAndFTs.begin(), jointList.begin(), jointList.end());
    jointsAndFTs.insert(jointsAndFTs.end(), ftFramesList.begin(), ftFramesList.end());

    REQUIRE(mdlLdr.loadReducedModelFromFile(modelPath, jointsAndFTs));
}

TEST_CASE("UkfState")
{
    auto parameterHandler = loadParameterHandler();

    iDynTree::ModelLoader modelLoader;
    auto modelHandler = parameterHandler->getGroup("MODEL").lock();

    REQUIRE_FALSE(modelHandler == nullptr);

    createModelLoader(modelHandler, modelLoader);

    auto kinDyn = std::make_shared<iDynTree::KinDynComputations>();
    REQUIRE(kinDyn->loadRobotModel(modelLoader.model()));

    std::vector<std::pair<std::shared_ptr<SubModel>, std::shared_ptr<SubModelKinDynWrapper>>>
        subModelMap;

    SubModelCreator subModelCreator;
    subModelCreator.setModelAndSensors(kinDyn->model(), modelLoader.sensors());
    REQUIRE(subModelCreator.setKinDyn(kinDyn));

    REQUIRE(subModelCreator.createSubModels(modelHandler));

    std::vector<std::shared_ptr<SubModelKinDynWrapper>> kinDynWrapperList;

    const auto& subModelList = subModelCreator.getSubModelList();

    for (int idx = 0; idx < subModelList.size(); idx++)
    {
        kinDynWrapperList.emplace_back(std::make_shared<SubModelKinDynWrapper>());
        REQUIRE(kinDynWrapperList.at(idx)->setKinDyn(kinDyn));
        REQUIRE(kinDynWrapperList.at(idx)->initialize(subModelList[idx]));
    }

    // Build the UkfState
    auto groupUKF = parameterHandler->getGroup("UKF").lock();
    REQUIRE_FALSE(groupUKF == nullptr);

    auto groupUKFStateTmp = groupUKF->getGroup("UKF_STATE").lock();
    REQUIRE_FALSE(groupUKFStateTmp == nullptr);

    auto groupUKFState = groupUKFStateTmp->clone();
    std::chrono::nanoseconds dT;
    REQUIRE(parameterHandler->getGroup("GENERAL").lock()->getParameter("sampling_time", dT));
    groupUKFState->setParameter("sampling_time", dT);

    std::unique_ptr<UkfState> stateModel
        = UkfState::build(groupUKFState, kinDyn, subModelList, kinDynWrapperList);

    // Create an input for the ukf state
    UKFInput input;

    // Define joint positions
    Eigen::VectorXd jointPos = Eigen::VectorXd::Zero(kinDyn->model().getNrOfDOFs());
    input.robotJointPositions = jointPos;

    // Define base pose
    manif::SE3d basePose = manif::SE3d::Identity();
    input.robotBasePose = basePose;

    // Define base velocity and acceleration
    manif::SE3d::Tangent baseVelocity, baseAcceleration;
    baseVelocity.setZero();
    baseAcceleration.setZero();
    input.robotBaseVelocity = baseVelocity;
    input.robotBaseAcceleration = baseAcceleration;

    for (int idx = 0; idx < subModelCreator.getSubModelList().size(); idx++)
    {
        REQUIRE(kinDynWrapperList.at(idx)
                    ->updateState(baseAcceleration,
                                  Eigen::VectorXd(kinDyn->model().getNrOfDOFs()).setZero(),
                                  UpdateMode::Full));
    }

    std::shared_ptr<UkfInputProvider> inputProvider = std::make_shared<UkfInputProvider>();

    BipedalLocomotion::System::VariablesHandler stateHandler
        = stateModel->getStateVariableHandler();
    int stateSize = stateModel->getStateSize();

    stateModel->setUkfInputProvider(inputProvider);

    Eigen::VectorXd currentState;
    currentState.resize(stateSize);
    currentState.setZero();

    Eigen::VectorXd motorTorques;
    motorTorques.resize(kinDyn->model().getNrOfDOFs());
    motorTorques << -1.6298, -1.10202, 0, -0.74, 0.0877, -0.00173;
    Eigen::VectorXd wrenchFTtLeg;
    wrenchFTtLeg.resize(6);
    wrenchFTtLeg << 0, 0, 100.518, 0.748, 0.91, 0;
    Eigen::VectorXd wrenchFTFootFront;
    wrenchFTFootFront.resize(6);
    wrenchFTFootFront << 0, 0, 1.761, -0.001, 0.0003, 0;
    Eigen::VectorXd wrenchFTFootRear;
    wrenchFTFootRear.resize(6);
    wrenchFTFootRear << 0, 0, 1.752, 0.000876, 0.000649, 0;

    currentState.segment(stateHandler.getVariable("tau_m").offset,
                         stateHandler.getVariable("tau_m").size)
        = motorTorques;
    currentState.segment(stateHandler.getVariable("r_leg_ft").offset,
                         stateHandler.getVariable("r_leg_ft").size)
        = wrenchFTtLeg;
    currentState.segment(stateHandler.getVariable("r_foot_front_ft").offset,
                         stateHandler.getVariable("r_foot_front_ft").size)
        = wrenchFTFootFront;
    currentState.segment(stateHandler.getVariable("r_foot_rear_ft").offset,
                         stateHandler.getVariable("r_foot_rear_ft").size)
        = wrenchFTFootRear;

    Eigen::VectorXd updatedState;
    updatedState.resize(stateSize);

    REQUIRE(inputProvider->setInput(input));

    stateModel->propagate(currentState, updatedState);

    constexpr double deltaState = 1e1;
    REQUIRE(updatedState.isApprox(currentState, deltaState));
}
