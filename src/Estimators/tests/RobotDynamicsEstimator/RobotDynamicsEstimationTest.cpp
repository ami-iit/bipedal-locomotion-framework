/**
 * @file RobotDynamicsEstimationTest.cpp
 * @authors Ines Sorrentino
 * @copyright 2023 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the BSD-3-Clause license.
 */

#include <catch2/catch_test_macros.hpp>
#include <catch2/generators/catch_generators_all.hpp>

#include <iCubModels/iCubModels.h>
#include <matioCpp/matioCpp.h>
#include <yarp/os/ResourceFinder.h>

#include <ConfigFolderPath.h>

#include <iDynTree/KinDynComputations.h>
#include <iDynTree/FreeFloatingState.h>
#include <iDynTree/Model.h>
#include <iDynTree/ModelTestUtils.h>
#include <iDynTree/ModelLoader.h>

#include <BipedalLocomotion/Conversions/ManifConversions.h>
#include <BipedalLocomotion/Conversions/matioCppConversions.h>
#include <BipedalLocomotion/Math/Constants.h>
#include <BipedalLocomotion/ParametersHandler/IParametersHandler.h>
#include <BipedalLocomotion/ParametersHandler/StdImplementation.h>
#include <BipedalLocomotion/ParametersHandler/YarpImplementation.h>
#include <BipedalLocomotion/System/VariablesHandler.h>

#include <BipedalLocomotion/RobotDynamicsEstimator/KinDynWrapper.h>
#include <BipedalLocomotion/RobotDynamicsEstimator/RobotDynamicsEstimator.h>
#include <BipedalLocomotion/RobotDynamicsEstimator/SubModel.h>

using namespace BipedalLocomotion::Estimators::RobotDynamicsEstimator;
using namespace BipedalLocomotion::System;
using namespace BipedalLocomotion;

struct Dataset
{
    Eigen::MatrixXd s;
    Eigen::MatrixXd ds;
    Eigen::MatrixXd dds;
    Eigen::MatrixXd im;
    Eigen::MatrixXd expectedTauj;
    Eigen::MatrixXd expectedTaum;
    Eigen::MatrixXd expectedTauF;
    std::map<std::string, Eigen::MatrixXd> fts;
    std::map<std::string, Eigen::MatrixXd> accs;
    std::map<std::string, Eigen::MatrixXd> gyros;
};

struct Output
{
    Eigen::MatrixXd ds;
    Eigen::MatrixXd tauj;
    Eigen::MatrixXd taum;
    Eigen::MatrixXd tauF;
    std::map<std::string, Eigen::MatrixXd> fts;
};

// Struct to represent a sensor
struct SensorProperty
{
    std::string sensorName;
    std::string sensorFrame;
};

ParametersHandler::IParametersHandler::shared_ptr loadConfiguration()
{
    std::shared_ptr<ParametersHandler::YarpImplementation> originalHandler
        = std::make_shared<ParametersHandler::YarpImplementation>();
    ParametersHandler::IParametersHandler::shared_ptr parameterHandler = originalHandler;

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

std::unordered_map<std::string, std::vector<SensorProperty>>
loadSensors(std::weak_ptr<const ParametersHandler::IParametersHandler> handler)
{
    std::unordered_map<std::string, std::vector<SensorProperty>> sensors;

    auto ptr = handler.lock();
    auto groupModel = ptr->getGroup("MODEL").lock();
    REQUIRE(groupModel != nullptr);

    auto ftGroup = groupModel->getGroup("FT").lock();
    REQUIRE(ftGroup != nullptr);
    std::unordered_map<std::string, std::string> ftSensors;
    std::vector<std::string> names, frames;
    REQUIRE(ftGroup->getParameter("names", names));
    REQUIRE(ftGroup->getParameter("frames", frames));
    std::vector<SensorProperty> ftSensorPairs;
    for (int idx = 0; idx < names.size(); idx++)
    {
        SensorProperty temp;
        temp.sensorName = names[idx];
        temp.sensorFrame = frames[idx];
        ftSensorPairs.emplace_back(temp);
    }
    sensors["ft"] = ftSensorPairs;

    auto contactGroup = groupModel->getGroup("EXTERNAL_CONTACT").lock();
    REQUIRE(contactGroup != nullptr);
    std::unordered_map<std::string, std::string> contactSensors;
    REQUIRE(contactGroup->getParameter("names", names));
    REQUIRE(contactGroup->getParameter("frames", frames));
    std::vector<SensorProperty> contactSensorPairs;
    for (int idx = 0; idx < names.size(); idx++)
    {
        SensorProperty temp;
        temp.sensorName = names[idx];
        temp.sensorFrame = frames[idx];
        contactSensorPairs.emplace_back(temp);
    }
    sensors["contact"] = contactSensorPairs;

    auto accGroup = groupModel->getGroup("ACCELEROMETER").lock();
    REQUIRE(accGroup != nullptr);
    std::unordered_map<std::string, std::string> accSensors;
    REQUIRE(accGroup->getParameter("names", names));
    REQUIRE(accGroup->getParameter("frames", frames));
    std::vector<SensorProperty> accSensorPairs;
    for (int idx = 0; idx < names.size(); idx++)
    {
        SensorProperty temp;
        temp.sensorName = names[idx];
        temp.sensorFrame = frames[idx];
        accSensorPairs.emplace_back(temp);
    }
    sensors["acc"] = accSensorPairs;

    auto gyroGroup = groupModel->getGroup("GYROSCOPE").lock();
    REQUIRE(gyroGroup != nullptr);
    std::unordered_map<std::string, std::string> gyroSensors;
    REQUIRE(gyroGroup->getParameter("names", names));
    REQUIRE(gyroGroup->getParameter("frames", frames));
    std::vector<SensorProperty> gyroSensorPairs;
    for (int idx = 0; idx < names.size(); idx++)
    {
        SensorProperty temp;
        temp.sensorName = names[idx];
        temp.sensorFrame = frames[idx];
        gyroSensorPairs.emplace_back(temp);
    }
    sensors["gyro"] = gyroSensorPairs;

    return sensors;
}

void loadRobotModel(std::weak_ptr<const ParametersHandler::IParametersHandler> handler,
                    std::shared_ptr<iDynTree::KinDynComputations> kindyn,
                    SubModelCreator& subModelCreator)
{
    auto ptr = handler.lock();

    auto groupModel = ptr->getGroup("MODEL").lock();
    REQUIRE(groupModel != nullptr);

    // List of joints and fts to load the model
    std::vector<SubModel> subModelList;

    std::vector<std::string> jointList;
    REQUIRE(groupModel->getParameter("joint_list", jointList));

    std::vector<std::string> ftFramesList;
    auto ftGroup = groupModel->getGroup("FT").lock();
    REQUIRE(ftGroup->getParameter("associated_joints", ftFramesList));

    std::vector<std::string> jointsAndFTs;
    jointsAndFTs.insert(jointsAndFTs.begin(), jointList.begin(), jointList.end());
    jointsAndFTs.insert(jointsAndFTs.end(), ftFramesList.begin(), ftFramesList.end());

    iDynTree::ModelLoader mdlLdr;
    REQUIRE(mdlLdr.loadReducedModelFromFile(getRobotModelPath(), jointsAndFTs));

    REQUIRE(kindyn->loadRobotModel(mdlLdr.model()));

    REQUIRE(kindyn->setFrameVelocityRepresentation(iDynTree::BODY_FIXED_REPRESENTATION));

    subModelCreator.setModelAndSensors(mdlLdr.model(), mdlLdr.sensors());
    REQUIRE(subModelCreator.setKinDyn(kindyn));

    // Split model
    REQUIRE(subModelCreator.createSubModels(groupModel));
}

Dataset& loadData()
{
    matioCpp::File input;
    REQUIRE(input.open(getDatasetPath()));
    matioCpp::Struct outStruct = input.read("robot").asStruct();

    std::vector<std::string> listVar = input.variableNames();
    REQUIRE(listVar.size() > 0);

    REQUIRE(outStruct.numberOfFields() > 0);

    // Unpack data
    static Dataset dataset;
    auto temp = outStruct("s").asMultiDimensionalArray<double>();
    dataset.s = Conversions::toEigen(temp);

    temp = outStruct("ds").asMultiDimensionalArray<double>();
    dataset.ds = Conversions::toEigen(temp);

    temp = outStruct("dds").asMultiDimensionalArray<double>();
    dataset.dds = Conversions::toEigen(temp);

    temp = outStruct("i_m").asMultiDimensionalArray<double>();
    dataset.im = Conversions::toEigen(temp);

    temp = outStruct("tau_j").asMultiDimensionalArray<double>();
    dataset.expectedTauj = Conversions::toEigen(temp);

    temp = outStruct("tau_m").asMultiDimensionalArray<double>();
    dataset.expectedTaum = Conversions::toEigen(temp);

    temp = outStruct("tau_F").asMultiDimensionalArray<double>();
    dataset.expectedTauF = Conversions::toEigen(temp);

    matioCpp::Struct outStruct2 = outStruct("fts").asStruct();
    for (const auto& ft : outStruct2.fields())
    {
        temp = outStruct2[ft].asMultiDimensionalArray<double>();
        dataset.fts[ft] = Conversions::toEigen(temp);
    }

    matioCpp::Struct outStruct3 = outStruct("accelerometers").asStruct();
    for (const auto& acc : outStruct3.fields())
    {
        temp = outStruct3[acc].asMultiDimensionalArray<double>();
        if (acc == "base_imu_0")
        {
            dataset.accs[acc + "_acc"] = Conversions::toEigen(temp);
        } else
        {
            dataset.accs[acc] = Conversions::toEigen(temp);
        }
    }

    matioCpp::Struct outStruct4 = outStruct("gyros").asStruct();
    for (const auto& gyro : outStruct4.fields())
    {
        temp = outStruct4[gyro].asMultiDimensionalArray<double>();
        if (gyro == "base_imu_0")
        {
            dataset.gyros[gyro + "_gyro"] = Conversions::toEigen(temp);
        } else
        {
            dataset.gyros[gyro] = Conversions::toEigen(temp);
        }
    }

    return dataset;
}

void createInitialState(Dataset& dataset,
                        std::weak_ptr<const ParametersHandler::IParametersHandler> handler,
                        RobotDynamicsEstimatorOutput& output)
{
    output.ds = dataset.ds.row(0);
    output.tau_F = dataset.expectedTauF.row(0) * 0.0;
    output.tau_m = dataset.expectedTaum.row(0);

    auto ptr = handler.lock();

    auto groupModel = ptr->getGroup("MODEL").lock();
    REQUIRE(groupModel != nullptr);

    auto ftGroup = groupModel->getGroup("FT").lock();
    REQUIRE(ftGroup != nullptr);

    std::vector<std::string> ftNames, ftFrames;
    REQUIRE(ftGroup->getParameter("names", ftNames));
    REQUIRE(ftGroup->getParameter("frames", ftFrames));
    for (int idx = 0; idx < ftNames.size(); idx++)
    {
        output.ftWrenches[ftNames[idx]] = dataset.fts[ftFrames[idx]].row(0);
    }

    auto contactGroup = groupModel->getGroup("EXTERNAL_CONTACT").lock();
    REQUIRE(contactGroup != nullptr);

    std::vector<std::string> contactNames;
    REQUIRE(contactGroup->getParameter("names", contactNames));
    for (int idx = 0; idx < contactNames.size(); idx++)
    {
        output.contactWrenches[contactNames[idx]] = Eigen::VectorXd::Zero(6);
    }

    auto accGroup = groupModel->getGroup("ACCELEROMETER").lock();
    REQUIRE(accGroup != nullptr);

    std::vector<std::string> accNames;
    REQUIRE(accGroup->getParameter("names", accNames));
    for (int idx = 0; idx < accNames.size(); idx++)
    {
        output.linearAccelerations[accNames[idx]] = dataset.accs[accNames[idx]].row(0);
    }

    auto gyroGroup = groupModel->getGroup("GYROSCOPE").lock();
    REQUIRE(gyroGroup != nullptr);

    std::vector<std::string> gyroNames;
    REQUIRE(gyroGroup->getParameter("names", gyroNames));
    for (int idx = 0; idx < gyroNames.size(); idx++)
    {
        output.angularVelocities[gyroNames[idx]] = dataset.gyros[gyroNames[idx]].row(0);
    }
}

void setInput(Dataset& dataset,
              int sample,
              RobotDynamicsEstimatorInput& input,
              std::unordered_map<std::string, std::vector<SensorProperty>>& sensors)
{
    // Set input
    input.jointPositions = dataset.s.row(sample);
    input.jointVelocities = dataset.ds.row(sample);
    input.motorCurrents = dataset.im.row(sample);
    input.frictionTorques.resize(input.motorCurrents.size());
    input.frictionTorques.setZero();

    for (int idx = 0; idx < sensors["ft"].size(); idx++)
    {
        input.ftWrenches[sensors["ft"][idx].sensorName]
            = dataset.fts[sensors["ft"][idx].sensorFrame].row(sample);
    }

    for (int idx = 0; idx < sensors["acc"].size(); idx++)
    {
        input.linearAccelerations[sensors["acc"][idx].sensorName]
            = dataset.accs[sensors["acc"][idx].sensorName].row(sample);
    }

    for (int idx = 0; idx < sensors["gyro"].size(); idx++)
    {
        input.angularVelocities[sensors["gyro"][idx].sensorName]
            = dataset.gyros[sensors["gyro"][idx].sensorName].row(sample);
    }
}

TEST_CASE("RobotDynamicsEstimator Test")
{
    // Load configuration
    auto parameterHandler = loadConfiguration();

    // Save sensors
    std::unordered_map<std::string, std::vector<SensorProperty>> sensors
        = loadSensors(parameterHandler);

    // Load robot model and create kindyn object
    SubModelCreator subModelCreator;
    auto kindyn = std::make_shared<iDynTree::KinDynComputations>();
    loadRobotModel(parameterHandler, kindyn, subModelCreator);

    // Get submodels and create kindynwrapper pointers
    std::vector<std::shared_ptr<KinDynWrapper>> kinDynWrapperList;
    std::vector<SubModel> subModelList = subModelCreator.getSubModelList();
    for (int idx = 0; idx < subModelCreator.getSubModelList().size(); idx++)
    {
        kinDynWrapperList.emplace_back(std::make_shared<KinDynWrapper>());
        REQUIRE(kinDynWrapperList[idx]->setModel(subModelList[idx]));
    }

    // Setup RDE
    std::unique_ptr<RobotDynamicsEstimator> estimator
        = RobotDynamicsEstimator::build(parameterHandler, kindyn, subModelList, kinDynWrapperList);
    REQUIRE_FALSE(estimator == nullptr);

    // Load dataset
    Dataset dataset = loadData();

    // Set estimator initial state
    RobotDynamicsEstimatorOutput output;
    createInitialState(dataset, parameterHandler, output);
    REQUIRE(estimator->setInitialState(output));

    // Starting estimation

    RobotDynamicsEstimatorInput input;

    // Set here input that are constant
    input.basePose.setIdentity();
    input.baseVelocity.setZero();
    input.baseAcceleration.setZero();

    int numOfSamples = 10;
    for (int sample_ = 0; sample_ < numOfSamples; sample_++)
    {
        int sample = sample_;

        setInput(dataset, sample, input, sensors);

        // Set input
        REQUIRE(estimator->setInput(input));

        // Estimate
        REQUIRE(estimator->advance());

        // Get output
        output = estimator->getOutput();

        // Check output
        REQUIRE((output.ds - dataset.ds.row(sample).transpose()).isZero(0.1));
        for (int idx = 0; idx < output.tau_F.size(); idx++)
        {
            REQUIRE(std::abs(output.tau_F(idx) - dataset.expectedTauF.row(sample)(idx)) < 0.2);
        }
        REQUIRE((output.tau_m - dataset.expectedTaum.row(sample).transpose()).isZero(0.1));
        for (int idx = 0; idx < output.tau_F.size(); idx++)
        {
            REQUIRE(std::abs((output.tau_m(idx) - output.tau_F(idx))
                             - dataset.expectedTauj.row(sample)(idx))
                    < 0.2);
        }
        for (int idx = 0; idx < sensors["ft"].size(); idx++)
        {
            REQUIRE(
                output.ftWrenches[sensors["ft"][idx].sensorName]
                    .isApprox(dataset.fts[sensors["ft"][idx].sensorFrame].row(sample).transpose(),
                              0.1));
        }
        for (int idx = 0; idx < sensors["contact"].size(); idx++)
        {
            REQUIRE((output.contactWrenches[sensors["contact"][idx].sensorFrame]).isZero(0.1));
        }
    }
}
