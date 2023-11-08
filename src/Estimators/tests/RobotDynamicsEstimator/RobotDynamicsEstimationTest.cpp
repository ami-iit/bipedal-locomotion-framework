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
#include <iDynTree/Model/FreeFloatingState.h>
#include <iDynTree/Model/Model.h>
#include <iDynTree/Model/ModelTestUtils.h>
#include <iDynTree/ModelIO/ModelLoader.h>

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
    log()->info("Unpacking data...");
    auto temp = outStruct("s").asMultiDimensionalArray<double>();
    dataset.s = Conversions::toEigen(temp);
    log()->info("Size of s: {}x{}", dataset.s.rows(), dataset.s.cols());

    temp = outStruct("ds").asMultiDimensionalArray<double>();
    dataset.ds = Conversions::toEigen(temp);
    log()->info("Size of ds: {}x{}", dataset.ds.rows(), dataset.ds.cols());

    temp = outStruct("dds").asMultiDimensionalArray<double>();
    dataset.dds = Conversions::toEigen(temp);
    log()->info("Size of dds: {}x{}", dataset.dds.rows(), dataset.dds.cols());

    temp = outStruct("i_m").asMultiDimensionalArray<double>();
    dataset.im = Conversions::toEigen(temp);
    log()->info("Size of im: {}x{}", dataset.im.rows(), dataset.im.cols());

    temp = outStruct("tau_j").asMultiDimensionalArray<double>();
    dataset.expectedTauj = Conversions::toEigen(temp);
    log()->info("Size of expected_tauj: {}x{}",
                dataset.expectedTauj.rows(),
                dataset.expectedTauj.cols());

    temp = outStruct("tau_m").asMultiDimensionalArray<double>();
    dataset.expectedTaum = Conversions::toEigen(temp);
    log()->info("Size of expected_taum: {}x{}",
                dataset.expectedTaum.rows(),
                dataset.expectedTaum.cols());

    temp = outStruct("tau_F").asMultiDimensionalArray<double>();
    dataset.expectedTauF = Conversions::toEigen(temp);
    log()->info("Size of expected_tauF: {}x{}",
                dataset.expectedTauF.rows(),
                dataset.expectedTauF.cols());

    matioCpp::Struct outStruct2 = outStruct("fts").asStruct();
    for (const auto& ft : outStruct2.fields())
    {
        temp = outStruct2[ft].asMultiDimensionalArray<double>();
        dataset.fts[ft] = Conversions::toEigen(temp);
        log()->info("Size of fts[{}]: {}x{}", ft, dataset.fts[ft].rows(), dataset.fts[ft].cols());
    }

    matioCpp::Struct outStruct3 = outStruct("accelerometers").asStruct();
    for (const auto& acc : outStruct3.fields())
    {
        temp = outStruct3[acc].asMultiDimensionalArray<double>();
        dataset.accs[acc] = Conversions::toEigen(temp);
        log()->info("Size of accs[{}]: {}x{}",
                    acc,
                    dataset.accs[acc].rows(),
                    dataset.accs[acc].cols());
    }

    matioCpp::Struct outStruct4 = outStruct("gyros").asStruct();
    for (const auto& gyro : outStruct4.fields())
    {
        temp = outStruct4[gyro].asMultiDimensionalArray<double>();
        dataset.gyros[gyro] = Conversions::toEigen(temp);
        log()->info("Size of gyros[{}]: {}x{}",
                    gyro,
                    dataset.gyros[gyro].rows(),
                    dataset.gyros[gyro].cols());
    }

    return dataset;
}

void createInitialState(Dataset& dataset, RobotDynamicsEstimatorOutput& output)
{
    output.ds = dataset.ds.row(0);
    output.tau_F = dataset.expectedTauF.row(0) * 0.0;
    output.tau_m = dataset.expectedTaum.row(0);
    for (auto const& [key, value] : dataset.fts)
    {
        output.ftWrenches[key] = value.row(0);
    }
}

TEST_CASE("RobotDynamicsEstimator Test")
{
    // Load configuration
    auto parameterHandler = loadConfiguration();

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
    createInitialState(dataset, output);
    REQUIRE(estimator->setInitialState(output));

    // Starting estimation
    log()->info("Starting estimation...");

    RobotDynamicsEstimatorInput input;

    // Set here input that are constant
    input.basePose.setIdentity();
    input.baseVelocity.setZero();
    input.baseAcceleration.setZero();

    int numOfSamples = 10;
    for (int sample = 0; sample < numOfSamples; sample++)
    {
        // Set input
        input.jointPositions = dataset.s.row(sample);
        input.jointVelocities = dataset.ds.row(sample);
        input.motorCurrents = dataset.im.row(sample);

        for (auto const& [key, value] : dataset.fts)
        {
            input.ftWrenches[key] = value.row(sample);
        }

        for (auto const& [key, value] : dataset.accs)
        {
            input.linearAccelerations[key] = value.row(sample);
        }

        for (auto const& [key, value] : dataset.gyros)
        {
            input.angularVelocities[key] = value.row(sample);
        }

        // Set input
        REQUIRE(estimator->setInput(input));

        // Estimate
        REQUIRE(estimator->advance());

        // Get output
        output = estimator->getOutput();

        // Check output
        REQUIRE((output.ds - dataset.ds.row(sample).transpose()).isZero(0.1));
        REQUIRE((output.tau_F - dataset.expectedTauF.row(sample).transpose()).isZero(0.1));
        REQUIRE((output.tau_m - dataset.expectedTaum.row(sample).transpose()).isZero(0.1));
        REQUIRE(((output.tau_m - output.tau_F) - dataset.expectedTauj.row(sample).transpose()).isZero(0.1));
        for (auto const& [key, value] : dataset.fts)
        {
            REQUIRE((output.ftWrenches[key] - value.row(sample).transpose()).isZero(0.1));
        }
    }
}
