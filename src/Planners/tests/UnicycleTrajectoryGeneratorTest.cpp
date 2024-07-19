/**
 * @file UnicycleTrajectoryGeneratorTest.cpp
 * @copyright 2020 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the BSD-3-Clause license.
 */
#include <catch2/catch_test_macros.hpp>

#include <manif/manif.h>

#include <iDynTree/Model.h>
#include <iDynTree/ModelLoader.h>

#include <BipedalLocomotion/ParametersHandler/StdImplementation.h>
#include <BipedalLocomotion/Planners/UnicycleTrajectoryGenerator.h>
#include <BipedalLocomotion/TextLogging/Logger.h>

#include <FolderPath.h>

using namespace BipedalLocomotion::Planners;
using namespace BipedalLocomotion::ParametersHandler;

std::shared_ptr<IParametersHandler> params()
{
    // Set the non-default parameters of the planner
    std::shared_ptr<IParametersHandler> handler = std::make_shared<StdImplementation>();

    handler->setParameter("referencePosition", Eigen::Vector2d(0.1, 0.0));
    handler->setParameter("saturationFactors", Eigen::Vector2d(0.7, 0.7));
    handler->setParameter("leftZMPDelta", Eigen::Vector2d(0.0, 0.0));
    handler->setParameter("rightZMPDelta", Eigen::Vector2d(0.0, 0.0));
    handler->setParameter("mergePointRatios", Eigen::Vector2d(0.4, 0.4));
    handler->setParameter("leftContactFrameName", "l_sole");
    handler->setParameter("rightContactFrameName", "r_sole");

    return handler;
}

TEST_CASE("UnicycleTrajectoryGeneratorTest")
{

    auto jointsList
        = std::vector<std::string>{"l_hip_pitch",      "l_hip_roll",       "l_hip_yaw",
                                   "l_knee",           "l_ankle_pitch",    "l_ankle_roll",
                                   "r_hip_pitch",      "r_hip_roll",       "r_hip_yaw",
                                   "r_knee",           "r_ankle_pitch",    "r_ankle_roll",
                                   "torso_pitch",      "torso_roll",       "torso_yaw",
                                   "neck_pitch",       "neck_roll",        "neck_yaw",
                                   "l_shoulder_pitch", "l_shoulder_roll",  "l_shoulder_yaw",
                                   "l_elbow",          "r_shoulder_pitch", "r_shoulder_roll",
                                   "r_shoulder_yaw",   "r_elbow"};

    iDynTree::ModelLoader ml;
    REQUIRE(ml.loadReducedModelFromFile(getRobotModelPath(), jointsList));

    const auto handler = params();

    BipedalLocomotion::Planners::UnicycleTrajectoryGenerator unicycleTrajectoryGenerator;

    REQUIRE(unicycleTrajectoryGenerator.initialize(handler));

    REQUIRE(unicycleTrajectoryGenerator.setRobotContactFrames(ml.model()));

    UnicycleTrajectoryGeneratorInput input
        = UnicycleTrajectoryGeneratorInput::generateDummyUnicycleTrajectoryGeneratorInput();

    UnicycleTrajectoryGeneratorOutput output;

    REQUIRE(unicycleTrajectoryGenerator.setInput(input));
    REQUIRE(unicycleTrajectoryGenerator.advance());
    REQUIRE(unicycleTrajectoryGenerator.isOutputValid());

    output = unicycleTrajectoryGenerator.getOutput();

    REQUIRE(output.contactPhaseList.size() == 1);
}
