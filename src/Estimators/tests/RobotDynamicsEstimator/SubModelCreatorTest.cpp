/**
 * @file SubModelCreatorTest.cpp
 * @authors Ines Sorrentino
 * @copyright 2023 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the BSD-3-Clause license.
 */

#include <string>
// Catch2
#include <catch2/catch_test_macros.hpp>

// YARP
#include <iCubModels/iCubModels.h>
#include <yarp/os/ResourceFinder.h>

// BLF
#include <BipedalLocomotion/ParametersHandler/IParametersHandler.h>
#include <BipedalLocomotion/ParametersHandler/YarpImplementation.h>
#include <BipedalLocomotion/RobotDynamicsEstimator/SubModel.h>

// iDynTree
#include <iDynTree/KinDynComputations.h>
#include <iDynTree/ModelLoader.h>

#include <ConfigFolderPath.h>

using namespace BipedalLocomotion::ParametersHandler;
namespace RDE = BipedalLocomotion::Estimators::RobotDynamicsEstimator;

TEST_CASE("SubModel Creation")
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

    auto groupModel = parameterHandler->getGroup("MODEL").lock();
    REQUIRE(groupModel != nullptr);

    // List of joints and fts to load the model
    std::vector<RDE::SubModel> subModelList;

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

    auto kinDyn = std::make_shared<iDynTree::KinDynComputations>();
    REQUIRE(kinDyn->loadRobotModel(mdlLdr.model()));

    // Case with FT, the model will be splitted
    BipedalLocomotion::log()->info("Case FT");
    RDE::SubModelCreator subModelCreatorWithFT;
    subModelCreatorWithFT.setModelAndSensors(mdlLdr.model(), mdlLdr.sensors());
    REQUIRE(subModelCreatorWithFT.setKinDyn(kinDyn));
    REQUIRE(subModelCreatorWithFT.createSubModels(groupModel));

    REQUIRE(subModelCreatorWithFT.getNrOfSubModels() == (1 + mdlLdr.sensors().getNrOfSensors(iDynTree::SIX_AXIS_FORCE_TORQUE)));

    // Case without FT, the model is not splitted and the only sub-model is the full model
    BipedalLocomotion::log()->info("Case without FT");

    jointsAndFTs.clear();
    jointsAndFTs.insert(jointsAndFTs.begin(), jointList.begin(), jointList.end());

    REQUIRE(mdlLdr.loadReducedModelFromFile(getRobotModelPath(), jointsAndFTs));

    REQUIRE(kinDyn->loadRobotModel(mdlLdr.model()));
    RDE::SubModelCreator subModelCreatorWithoutFT;
    subModelCreatorWithoutFT.setModelAndSensors(mdlLdr.model(), mdlLdr.sensors());
    REQUIRE(subModelCreatorWithoutFT.setKinDyn(kinDyn));
    auto groupFT = groupModel->getGroup("FT").lock()->clone();
    groupFT->clear();
    std::vector<std::string> emptyVector;
    groupFT->setParameter("names", emptyVector);
    groupFT->setParameter("frames", emptyVector);
    groupFT->setParameter("ukf_names", emptyVector);
    groupFT->setParameter("associated_joints", emptyVector);
    groupModel->setGroup("FT", groupFT);
    REQUIRE(subModelCreatorWithoutFT.createSubModels(groupModel));

    REQUIRE(subModelCreatorWithoutFT.getNrOfSubModels() == 1);
}
