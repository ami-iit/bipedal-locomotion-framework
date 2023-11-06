/**
 * @file RobotDynamicsEstimationTest.cpp
 * @authors Ines Sorrentino
 * @copyright 2023 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the BSD-3-Clause license.
 */

#include <catch2/catch_test_macros.hpp>
#include <catch2/generators/catch_generators_all.hpp>

#include <iCubModels/iCubModels.h>
#include <yarp/os/ResourceFinder.h>
#include <matioCpp/matioCpp.h>

#include <ConfigFolderPath.h>

#include <iDynTree/KinDynComputations.h>
#include <iDynTree/Model/FreeFloatingState.h>
#include <iDynTree/Model/Model.h>
#include <iDynTree/Model/ModelTestUtils.h>
#include <iDynTree/ModelIO/ModelLoader.h>

#include <BipedalLocomotion/Conversions/ManifConversions.h>
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
    std::unique_ptr<BipedalLocomotion::Estimators::RobotDynamicsEstimator::RobotDynamicsEstimator>
        estimator = BipedalLocomotion::Estimators::RobotDynamicsEstimator::RobotDynamicsEstimator::
            build(parameterHandler, kindyn, subModelList, kinDynWrapperList);
    REQUIRE_FALSE(estimator == nullptr);

    // Load dataset
    matioCpp::File input;
    REQUIRE(input.open(getDatasetPath()));
    matioCpp::Struct outStruct = input.read("robot").asStruct();

    std::vector<std::string> listVar = input.variableNames();
    log()->info("Num variables --> {}", listVar.size());
    for (auto & name : listVar)
        log()->info("{}", name);

    std::vector<std::string> fields = outStruct.fields();

    log()->info("Number of fields --> {}", outStruct.numberOfFields());
    
    for (auto & field : fields)
        log()->info("{}", field);

    // Set estimator initial state
}
