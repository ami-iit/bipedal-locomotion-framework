/**
 * @file RobotDynamicsEstimatorTest.cpp
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
#include <BipedalLocomotion/System/Clock.h>

#include <BipedalLocomotion/ParametersHandler/IParametersHandler.h>
#include <BipedalLocomotion/ParametersHandler/YarpImplementation.h>
#include <BipedalLocomotion/Math/Constants.h>

// iDynTree
#include <iDynTree/ModelIO/ModelLoader.h>
#include <iDynTree/KinDynComputations.h>

// RDE
#include <BipedalLocomotion/RobotDynamicsEstimator/SubModel.h>
#include <BipedalLocomotion/RobotDynamicsEstimator/SubModelKinDynWrapper.h>
#include <BipedalLocomotion/RobotDynamicsEstimator/RobotDynamicsEstimator.h>

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
    REQUIRE(ftGroup->getParameter("associated_joints", ftFramesList));

    std::vector<std::string> jointsAndFTs;
    jointsAndFTs.insert(jointsAndFTs.begin(), jointList.begin(), jointList.end());
    jointsAndFTs.insert(jointsAndFTs.end(), ftFramesList.begin(), ftFramesList.end());

    REQUIRE(mdlLdr.loadReducedModelFromFile(modelPath, jointsAndFTs));
}

void createInitialState(std::shared_ptr<iDynTree::KinDynComputations> kinDynFullModel,
                        Eigen::Ref<Eigen::VectorXd> jointTorques,
                        IParametersHandler::weak_ptr modelHandler,
                        const std::vector<SubModel>& subModelList,
                        RobotDynamicsEstimatorOutput& initialState)
{
    // All the values set here come from an experiment on the real robot

    // Resize variables
    initialState.ds.resize(kinDynFullModel->model().getNrOfDOFs()); // ds
    initialState.tau_m.resize(kinDynFullModel->model().getNrOfDOFs()); // tau_m
    initialState.tau_F.resize(kinDynFullModel->model().getNrOfDOFs()); // tau_F
    std::vector<std::string> ftList;
    auto ftGroup = modelHandler.lock()->getGroup("FT").lock();
    REQUIRE(ftGroup->getParameter("names", ftList));
    for (auto ft : ftList)
    {
        initialState.ftWrenches[ft] = Eigen::VectorXd(6).setZero(); // FT

        std::string ftBias = ft + "_bias";
        initialState.ftWrenchesBiases[ftBias] = Eigen::VectorXd(6).setZero(); // FT bias
    }
    std::vector<std::string> accList;
    auto accGroup = modelHandler.lock()->getGroup("ACCELEROMETER").lock();
    REQUIRE(accGroup->getParameter("names", accList));
    for (auto acc : accList)
    {
        std::string accBias = acc + "_bias";
        initialState.accelerometerBiases[accBias] = Eigen::VectorXd(3).setZero(); // ACC BIAS
    }
    std::vector<std::string> gyroList;
    auto gyroGroup = modelHandler.lock()->getGroup("GYROSCOPE").lock();
    REQUIRE(gyroGroup->getParameter("names", gyroList));
    for (auto gyro : gyroList)
    {
        std::string gyroBias = gyro + "_bias";
        initialState.gyroscopeBiases[gyroBias] = Eigen::VectorXd(3).setZero(); // GYRO BIAS
    }


    // Set values
    initialState.ds.setZero();
    initialState.tau_m << -1.24764e+01, 1.03400e-01, -4.70000e-03, -3.16350e+00, 4.21800e-01, 4.85000e-01;
    initialState.tau_F.setZero();
    initialState.ftWrenches["r_leg_ft"] << -3.83709731e+01, -8.45653659e+00,  9.25208925e+01,  3.12676978e+00, -9.79714657e+00,  4.01285264e-01;
    initialState.ftWrenches["r_foot_front_ft"] << 5.22305136e-01, -4.88067107e-01, 1.61011089e+00, -9.13330381e-03, -8.39265034e-03,  4.18725767e-04;
    initialState.ftWrenches["r_foot_rear_ft"] << 5.19688377e-01, -4.85621881e-01,  1.60204420e+00, -8.32204636e-03, -9.16952530e-03, -7.99299795e-05;
    initialState.ftWrenchesBiases["r_leg_ft_bias"] << -1.89410386e+01,  7.33674253e+01, -6.04774355e+01,  1.34890092e-02, -3.02023625e+00,  7.01925185e-01;
    initialState.ftWrenchesBiases["r_foot_front_ft_bias"] << -4.85874907e+01, -3.90627141e+01, -3.89636265e+01, -1.83127438e-01,  9.51385814e-01,  2.97127661e-01;
    initialState.ftWrenchesBiases["r_foot_rear_ft_bias"] << -3.75985458e+01, -6.87282453e+01, -1.41893873e+00, -2.24845286e+00, 1.18104453e+00,  3.75446141e-01;
}

void createInput(std::shared_ptr<iDynTree::KinDynComputations> kinDynFullModel,
                 Eigen::Ref<Eigen::VectorXd> jointTorques,
                 IParametersHandler::weak_ptr modelHandler,
                 const std::vector<SubModel>& subModelList,
                 RobotDynamicsEstimatorInput& input)
{
    // All the values set here come from an experiment on the real robot

    manif::SE3d basePose;
    basePose.setIdentity();
    input.basePose = basePose;

    manif::SE3d::Tangent baseVel;
    baseVel.setZero();
    input.baseVelocity = baseVel;

    manif::SE3d::Tangent baseAcc;
    baseAcc.setZero();
    input.baseAcceleration = baseAcc;

    input.jointPositions.resize(kinDynFullModel->model().getNrOfDOFs());
    input.jointPositions << -3.81866275e-01, 1.27512464e-01, 3.83496133e-04, -2.67488553e-02, -9.77915140e-03, 9.58740333e-05;

    input.jointVelocities.resize(kinDynFullModel->model().getNrOfDOFs());
    input.jointVelocities.setZero();

    // Use gearbox ratio and torque constants to convert joint torques into motor currents
    Eigen::VectorXd gearRatio(kinDynFullModel->model().getNrOfDOFs());
    gearRatio << 100.0, -100.0, 100.0, 100.0, 100.0, 100.0;

    Eigen::VectorXd torqueConstant(kinDynFullModel->model().getNrOfDOFs());
    torqueConstant << 0.111, 0.047, 0.047, 0.111, 0.111, 0.025;

//    input.motorCurrents = jointTorques.array() / (gearRatio.array() * torqueConstant.array());
    input.motorCurrents.resize(kinDynFullModel->model().getNrOfDOFs());
    input.motorCurrents << -1.124e+00, -2.200e-02, -1.000e-03, -2.850e-01,  3.800e-02, 1.940e-01;

    input.ftWrenches["r_leg_ft"] = Eigen::VectorXd(6).setZero();
    input.ftWrenches["r_leg_ft"] << -57.31201172, 64.91088867, 32.04345703, 3.14025879, -12.81738281, 1.10321045;

    input.ftWrenches["r_foot_front_ft"] = Eigen::VectorXd(6).setZero();
    input.ftWrenches["r_foot_front_ft"] << -48.06518555, -39.55078125, -37.35351562, -0.19226074, 0.94299316, 0.29754639;

    input.ftWrenches["r_foot_rear_ft"] = Eigen::VectorXd(6).setZero();
    input.ftWrenches["r_foot_rear_ft"] << -37.07885742, -69.21386719, 0.18310547, -2.2567749, 1.171875, 0.37536621;

    std::vector<std::string> accList;
    auto accGroup = modelHandler.lock()->getGroup("ACCELEROMETER").lock();
    REQUIRE(accGroup->getParameter("names", accList));
    for (auto acc : accList)
    {
        input.linearAccelerations[acc] = Eigen::VectorXd(3).setZero();
        input.linearAccelerations[acc](2) = - BipedalLocomotion::Math::StandardAccelerationOfGravitation;
    }
    input.linearAccelerations["r_leg_ft_acc"] << 3.72, 0.8, -9.33;
    input.linearAccelerations["r_foot_front_ft_acc"] << 4.01, -2.38, 8.74;
    input.linearAccelerations["r_foot_rear_ft_acc"] << 3.77, -2.39, 8.94;

    std::vector<std::string> gyroList;
    auto gyroGroup = modelHandler.lock()->getGroup("GYROSCOPE").lock();
    REQUIRE(gyroGroup->getParameter("names", gyroList));
    for (auto gyro : gyroList)
    {
        input.angularVelocities[gyro] = Eigen::VectorXd(3).setZero();
    }
    input.angularVelocities["r_leg_ft_gyro"] << 0.03708825, -0.06654068, -0.00109083;
    input.angularVelocities["r_foot_front_ft_gyro"] << 0.0567232 ,  0.04799655, -0.01308997;
    input.angularVelocities["r_foot_rear_ft_gyro"] << 0.06108652,  0.04690572, -0.01090831;
}

TEST_CASE("RobotDynamicsEstimator")
{
    auto parameterHandler = loadParameterHandler();

    iDynTree::ModelLoader modelLoader;
    auto modelHandler = parameterHandler->getGroup("MODEL").lock();

    REQUIRE_FALSE(modelHandler == nullptr);

    createModelLoader(modelHandler, modelLoader);

    auto kinDyn = std::make_shared<iDynTree::KinDynComputations>();
    REQUIRE(kinDyn->loadRobotModel(modelLoader.model()));
    REQUIRE(kinDyn->setFrameVelocityRepresentation(iDynTree::BODY_FIXED_REPRESENTATION));

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

    // automatic build the Estimator from parameter handler
    auto estimator = RobotDynamicsEstimator::build(parameterHandler, kinDyn, subModelList, kinDynWrapperList);
    REQUIRE_FALSE(estimator == nullptr);

    manif::SE3d::Tangent robotBaseAcceleration;
    robotBaseAcceleration.setZero();

    Eigen::VectorXd robotJointAcceleration(kinDyn->model().getNrOfDOFs());
    robotJointAcceleration.setZero();

    for (int idx = 0; idx < subModelCreator.getSubModelList().size(); idx++)
    {
        REQUIRE(kinDynWrapperList.at(idx)->updateState(robotBaseAcceleration,
                                                       robotJointAcceleration,
                                                       true));
    }

    iDynTree::LinkNetExternalWrenches extWrench(kinDyn->model());
    extWrench.zero();

    // Compute joint torques in static configuration from inverse dynamics on the full model
    iDynTree::FreeFloatingGeneralizedTorques jointTorques(kinDyn->model());

    kinDyn->inverseDynamics(iDynTree::make_span(robotBaseAcceleration.data(),
                                                manif::SE3d::Tangent::DoF),
                            robotJointAcceleration,
                            extWrench,
                            jointTorques);

    RobotDynamicsEstimatorOutput initialState;
    createInitialState(kinDyn,
                       iDynTree::toEigen(jointTorques.jointTorques()),
                       modelHandler,
                       subModelList,
                       initialState);

    REQUIRE(estimator->setInitialState(initialState));

    RobotDynamicsEstimatorInput measurement;
    createInput(kinDyn,
                iDynTree::toEigen(jointTorques.jointTorques()),
                modelHandler,
                subModelList,
                measurement);

    auto tic = BipedalLocomotion::clock().now();
    REQUIRE(estimator->setInput(measurement));
    REQUIRE(estimator->advance());
    auto toc = BipedalLocomotion::clock().now();

    BipedalLocomotion::log()->error("{}", toc - tic);

    RobotDynamicsEstimatorOutput result = estimator->getOutput();
}
