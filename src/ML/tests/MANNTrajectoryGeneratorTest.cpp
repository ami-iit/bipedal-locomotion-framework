/**
 * @file MANNTrajectoryGeneratorTest.cpp
 * @authors Paolo Maria Viceconte, Giulio Romualdi
 * @copyright 2023 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the BSD-3-Clause license.
 */

#include <memory>

#include <iDynTree/Model.h>
#include <iDynTree/ModelLoader.h>

// Catch2
#include <catch2/catch_test_macros.hpp>

#include <BipedalLocomotion/Contacts/Contact.h>
#include <BipedalLocomotion/ML/MANNTrajectoryGenerator.h>
#include <BipedalLocomotion/ParametersHandler/StdImplementation.h>

#include <FolderPath.h>

using namespace BipedalLocomotion::ML;
using namespace BipedalLocomotion::Contacts;
using namespace BipedalLocomotion::ParametersHandler;

TEST_CASE("MANNTrajectoryGenerator")
{
    using namespace std::chrono_literals;

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

    auto handler = std::make_shared<StdImplementation>();
    handler->setParameter("joints_list", jointsList);
    handler->setParameter("root_link_frame_name", "root_link");
    handler->setParameter("chest_link_frame_name", "chest");
    handler->setParameter("left_foot_frame_name", "l_sole");
    handler->setParameter("right_foot_frame_name", "r_sole");
    handler->setParameter("sampling_time", 20ms);
    handler->setParameter("time_horizon", 10s);
    handler->setParameter("slow_down_factor", 1.0);
    handler->setParameter("scaling_factor", 1.0);
    handler->setParameter("forward_direction", "x");
    handler->setParameter("mocap_frame_rate", 50);
    handler->setParameter("past_projected_base_horizon", 1s);

    auto leftFootGroup = std::make_shared<StdImplementation>();
    leftFootGroup->setParameter("number_of_corners", 4);
    leftFootGroup->setParameter("corner_0", std::vector<double>{+0.08, +0.03, 0.0});
    leftFootGroup->setParameter("corner_1", std::vector<double>{+0.08, -0.03, 0.0});
    leftFootGroup->setParameter("corner_2", std::vector<double>{-0.08, -0.03, 0.0});
    leftFootGroup->setParameter("corner_3", std::vector<double>{-0.08, +0.03, 0.0});
    leftFootGroup->setParameter("on_threshold", 0.01);
    leftFootGroup->setParameter("off_threshold", 0.01);
    leftFootGroup->setParameter("switch_on_after", 40ms);
    leftFootGroup->setParameter("switch_off_after", 40ms);

    auto rightFootGroup = std::make_shared<StdImplementation>();
    rightFootGroup->setParameter("number_of_corners", 4);
    rightFootGroup->setParameter("corner_0", std::vector<double>{+0.08, +0.03, 0.0});
    rightFootGroup->setParameter("corner_1", std::vector<double>{+0.08, -0.03, 0.0});
    rightFootGroup->setParameter("corner_2", std::vector<double>{-0.08, -0.03, 0.0});
    rightFootGroup->setParameter("corner_3", std::vector<double>{-0.08, +0.03, 0.0});
    rightFootGroup->setParameter("on_threshold", 0.01);
    rightFootGroup->setParameter("off_threshold", 0.01);
    rightFootGroup->setParameter("switch_on_after", 40ms);
    rightFootGroup->setParameter("switch_off_after", 40ms);

    auto mannGroup = std::make_shared<StdImplementation>();
    mannGroup->setParameter("projected_base_datapoints", 12);
    mannGroup->setParameter("onnx_model_path", getMANNModelPath());

    handler->setGroup("LEFT_FOOT", leftFootGroup);
    handler->setGroup("RIGHT_FOOT", rightFootGroup);
    handler->setGroup("MANN", mannGroup);

    // input to generate a forward direction
    MANNTrajectoryGeneratorInput generatorInput;
    generatorInput.desiredFutureBaseTrajectory.resize(2, 7);
    generatorInput.desiredFutureBaseVelocities.resize(2, 7);
    generatorInput.desiredFutureFacingDirections.resize(2, 7);
    generatorInput.desiredFutureBaseTrajectory << 0, 0.12, 0.22, 0.3, 0.35, 0.39, 0.4, 0, 0, 0, 0,
        0, 0, 0;
    generatorInput.mergePointIndex = 0;

    for (int i = 0; i < generatorInput.desiredFutureFacingDirections.cols(); i++)
    {
        generatorInput.desiredFutureFacingDirections.col(i) << 1.0, 0;
        generatorInput.desiredFutureBaseVelocities.col(i) << 0.4, 0;
    }

    const manif::SE3d basePose = manif::SE3d(Eigen::Vector3d{0, 0, 0.7748},
                                             Eigen::AngleAxis(0.0, Eigen::Vector3d::UnitY()));

    Eigen::VectorXd jointPositions(26);
    jointPositions << -0.10922017141063572, 0.05081325960010118, 0.06581966291990003,
        -0.0898053099824925, -0.09324922528169599, -0.05110058859172172, -0.11021232812838086,
        0.054291515925228385, 0.0735575862560208, -0.09509332143185895, -0.09833823347493076,
        -0.05367281245082792, 0.1531558711397399, -0.001030634273454133, 0.0006584764419034815,
        -0.0016821925351926288, -0.004284529460797688, 0.030389771690123243, -0.040592118429752494,
        -0.1695472679986807, -0.20799422095574033, 0.045397975984119654, -0.03946672931050908,
        -0.16795588539580256, -0.20911090583076936, 0.0419854257806720;

    MANNTrajectoryGenerator generator;
    REQUIRE(generator.setRobotModel(ml.model()));
    REQUIRE(generator.initialize(handler));
    REQUIRE(generator.setInitialState(jointPositions, basePose));
    REQUIRE(generator.setInput(generatorInput));
    REQUIRE(generator.advance());

    // here we check that the robot was able to walk straight
    Eigen::Vector3d finalCoMPosition = generator.getOutput().comTrajectory.back();
    Eigen::Vector3d initialCoMPosition = generator.getOutput().comTrajectory.front();

    constexpr double forwardTolerance = 2;
    constexpr double lateralTolerance = 0.6;
    constexpr double verticalTolerance = 0.1;
    REQUIRE(std::abs(finalCoMPosition[0] - initialCoMPosition[0]) > forwardTolerance);
    REQUIRE(std::abs(finalCoMPosition[1] - initialCoMPosition[1]) < lateralTolerance);
    REQUIRE(std::abs(finalCoMPosition[2] - initialCoMPosition[2]) < verticalTolerance);

    // test the reset
    generatorInput.mergePointIndex = 10;
    REQUIRE(generator.setInput(generatorInput));
    REQUIRE(generator.advance());

    // here we check that the robot was able to walk straight
    finalCoMPosition = generator.getOutput().comTrajectory.back();
    initialCoMPosition = generator.getOutput().comTrajectory.front();
    REQUIRE(std::abs(finalCoMPosition[0] - initialCoMPosition[0]) > forwardTolerance);
    REQUIRE(std::abs(finalCoMPosition[1] - initialCoMPosition[1]) < lateralTolerance);
    REQUIRE(std::abs(finalCoMPosition[2] - initialCoMPosition[2]) < verticalTolerance);
}
