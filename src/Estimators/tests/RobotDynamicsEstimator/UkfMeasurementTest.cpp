/**
 * @file UkfMeasurementTest.cpp
 * @authors Ines Sorrentino
 * @copyright 2023 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the BSD-3-Clause license.
 */

#include <catch2/catch.hpp>
#include <iCubModels/iCubModels.h>
#include <yarp/os/ResourceFinder.h>
#include <ConfigFolderPath.h>

//BLF
#include <BipedalLocomotion/System/VariablesHandler.h>
#include <BipedalLocomotion/ParametersHandler/IParametersHandler.h>
#include <BipedalLocomotion/ParametersHandler/YarpImplementation.h>

// iDynTree
#include <iDynTree/ModelIO/ModelLoader.h>
#include <iDynTree/KinDynComputations.h>

//LIBRARYTOTEST
#include <BipedalLocomotion/RobotDynamicsEstimator/SubModel.h>
#include <BipedalLocomotion/RobotDynamicsEstimator/SubModelKinDynWrapper.h>
#include <BipedalLocomotion/RobotDynamicsEstimator/Dynamics.h>
#include <BipedalLocomotion/RobotDynamicsEstimator/UkfMeasurement.h>

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

void createModelLoader(IParametersHandler::weak_ptr group,
                       iDynTree::ModelLoader& mdlLdr)
{
    // List of joints and fts to load the model
    std::vector<SubModel> subModelList;

    const std::string modelPath = iCubModels::getModelFile("iCubGenova09");

    std::vector<std::string> jointList;
    REQUIRE(group.lock()->getParameter("joint_list", jointList));

    std::vector<std::string> ftFramesList;
    auto ftGroup = group.lock()->getGroup("FT").lock();
    REQUIRE(ftGroup->getParameter("frames", ftFramesList));

    std::vector<std::string> jointsAndFTs;
    jointsAndFTs.insert(jointsAndFTs.begin(), jointList.begin(), jointList.end());
    jointsAndFTs.insert(jointsAndFTs.end(), ftFramesList.begin(), ftFramesList.end());

    REQUIRE(mdlLdr.loadReducedModelFromFile(modelPath, jointsAndFTs));
}

TEST_CASE("UkfMeasurement")
{
    auto parameterHandler = loadParameterHandler();

    iDynTree::ModelLoader modelLoader;
    auto modelHandler = parameterHandler->getGroup("MODEL").lock();

    REQUIRE_FALSE(modelHandler == nullptr);

    createModelLoader(modelHandler, modelLoader);

    auto kinDyn = std::make_shared<iDynTree::KinDynComputations>();
    REQUIRE(kinDyn->loadRobotModel(modelLoader.model()));

    std::vector<std::pair<std::shared_ptr<SubModel>, std::shared_ptr<SubModelKinDynWrapper>>> subModelMap;

    SubModelCreator subModelCreator;
    subModelCreator.setModelAndSensors(kinDyn->model(), modelLoader.sensors());
    REQUIRE(subModelCreator.setKinDyn(kinDyn));

    REQUIRE(subModelCreator.createSubModels(modelHandler));

    std::vector<std::shared_ptr<SubModelKinDynWrapper>> kinDynWrapperList;

    const auto & subModelList = subModelCreator.getSubModelList();

    for (int idx = 0; idx < subModelList.size(); idx++)
    {
        kinDynWrapperList.emplace_back(std::make_shared<SubModelKinDynWrapper>());
        REQUIRE(kinDynWrapperList.at(idx)->setKinDyn(kinDyn));
        REQUIRE(kinDynWrapperList.at(idx)->initialize(subModelList[idx]));
    }

    // Build the UkfState
    auto groupUKF = parameterHandler->getGroup("UKF").lock();
    REQUIRE_FALSE(groupUKF == nullptr);

    auto groupUKFMeasurementTmp = groupUKF->getGroup("UKF_MEASUREMENT").lock();
    REQUIRE_FALSE(groupUKFMeasurementTmp == nullptr);

    auto groupUKFMeasurement = groupUKFMeasurementTmp->clone();
    double dT;
    REQUIRE(parameterHandler->getGroup("GENERAL").lock()->getParameter("sampling_time", dT));
    groupUKFMeasurement->setParameter("sampling_time", dT);

    // Create the state variable handler
    constexpr size_t sizeVariable = 6;
    VariablesHandler stateVariableHandler;
    REQUIRE(stateVariableHandler.addVariable("ds", sizeVariable));
    REQUIRE(stateVariableHandler.addVariable("tau_m", sizeVariable));
    REQUIRE(stateVariableHandler.addVariable("tau_F", sizeVariable));
    REQUIRE(stateVariableHandler.addVariable("r_leg_ft_sensor", 6));
    REQUIRE(stateVariableHandler.addVariable("r_foot_front_ft_sensor", 6));
    REQUIRE(stateVariableHandler.addVariable("r_foot_rear_ft_sensor", 6));
    REQUIRE(stateVariableHandler.addVariable("r_leg_ft_sensor_bias", 6));
    REQUIRE(stateVariableHandler.addVariable("r_foot_front_ft_sensor_bias", 6));
    REQUIRE(stateVariableHandler.addVariable("r_foot_rear_ft_sensor_bias", 6));
    REQUIRE(stateVariableHandler.addVariable("r_leg_ft_acc_bias", 3));
    REQUIRE(stateVariableHandler.addVariable("r_foot_front_ft_acc_bias", 3));
    REQUIRE(stateVariableHandler.addVariable("r_foot_rear_ft_acc_bias", 3));
    REQUIRE(stateVariableHandler.addVariable("r_leg_ft_gyro_bias", 3));
    REQUIRE(stateVariableHandler.addVariable("r_foot_front_ft_gyro_bias", 3));
    REQUIRE(stateVariableHandler.addVariable("r_foot_rear_ft_gyro_bias", 3));

    std::unique_ptr<UkfMeasurement> measurementModel = UkfMeasurement::build(groupUKFMeasurement,
                                                                             stateVariableHandler,
                                                                             kinDyn,
                                                                             subModelList,
                                                                             kinDynWrapperList);

    REQUIRE(measurementModel != nullptr);

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

    for (int idx = 0; idx < subModelCreator.getSubModelList().size(); idx++)
    {
        REQUIRE(kinDynWrapperList.at(idx)->updateState(baseAcceleration,
                                                       Eigen::VectorXd(kinDyn->model().getNrOfDOFs()).setZero(),
                                                       true));
    }

    std::shared_ptr<UkfInputProvider> inputProvider = std::make_shared<UkfInputProvider>();

    BipedalLocomotion::System::VariablesHandler measurementHandler = measurementModel->getMeasurementVariableHandler();
//    int measurementSize = measurementModel->getMeasurementSize();
    int stateSize = stateVariableHandler.getNumberOfVariables();

    measurementModel->setUkfInputProvider(inputProvider);

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

    currentState.segment(stateVariableHandler.getVariable("tau_m").offset, stateVariableHandler.getVariable("tau_m").size) = motorTorques;
    currentState.segment(stateVariableHandler.getVariable("r_leg_ft_sensor").offset, stateVariableHandler.getVariable("r_leg_ft_sensor").size) = wrenchFTtLeg;
    currentState.segment(stateVariableHandler.getVariable("r_foot_front_ft_sensor").offset, stateVariableHandler.getVariable("r_foot_front_ft_sensor").size) = wrenchFTFootFront;
    currentState.segment(stateVariableHandler.getVariable("r_foot_rear_ft_sensor").offset, stateVariableHandler.getVariable("r_foot_rear_ft_sensor").size) = wrenchFTFootRear;
    currentState.segment(stateVariableHandler.getVariable("r_foot_rear_ft_sensor").offset, stateVariableHandler.getVariable("r_foot_rear_ft_sensor").size) = wrenchFTFootRear;

    REQUIRE(inputProvider->setInput(input));

    std::map<std::string, Eigen::VectorXd> measurement;

    Eigen::VectorXd i_m;
    i_m.resize(kinDyn->model().getNrOfDOFs());
    i_m << -0.1468, 0.2345, 0, -0.0667, 0.008, -0.000692;
    measurement["i_m"] = i_m;
    measurement["r_leg_ft_sensor"] = wrenchFTtLeg;
    measurement["r_foot_front_ft_sensor"] = wrenchFTFootFront;
    measurement["r_foot_rear_ft_sensor"] = wrenchFTFootRear;

    Eigen::Vector3d zeroVec3;
    zeroVec3.setZero();

    measurement["r_leg_ft_gyro"] = zeroVec3;
    measurement["r_foot_front_ft_gyro"] = zeroVec3;
    measurement["r_foot_rear_ft_gyro"] = zeroVec3;

    Eigen::Vector3d gravVec;
    gravVec.setZero();
    gravVec(2) = 9.8066;

    measurement["r_leg_ft_acc"] = gravVec;
    measurement["r_foot_front_ft_acc"] = gravVec;
    measurement["r_foot_rear_ft_acc"] = gravVec;

    Eigen::VectorXd zeroVector6(stateVariableHandler.getVariable("tau_m").size);
    zeroVector6.setZero();
    measurement["ds"] = zeroVector6;

    bfl::Data updatedMeasurementTmp;

    bool resultOk;

    REQUIRE(measurementModel->freeze(measurement));

    std::tie(resultOk, updatedMeasurementTmp) = measurementModel->predictedMeasure(currentState);

    REQUIRE(resultOk);

    Eigen::VectorXd updatedMeasurement = bfl::any::any_cast<Eigen::MatrixXd&&>(std::move(updatedMeasurementTmp));

    for (int idx = 0; idx < updatedMeasurement.size(); idx++)
    {
        REQUIRE(std::abs(updatedMeasurement[idx]) < 200);
    }
}
