/**
 * @file VelMANNTrajectoryGeneratorTest.cpp
 * @authors Evelyn D'Elia
 * @copyright 2024 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the BSD-3-Clause license.
 */

#include <memory>

#include <iDynTree/Model/Model.h>
#include <iDynTree/ModelIO/ModelLoader.h>

// Catch2
#include <catch2/catch_test_macros.hpp>

#include <BipedalLocomotion/Contacts/Contact.h>
#include <BipedalLocomotion/ML/VelMANNTrajectoryGenerator.h>
#include <BipedalLocomotion/ParametersHandler/StdImplementation.h>

#include <FolderPath.h>

using namespace BipedalLocomotion::ML;
using namespace BipedalLocomotion::Contacts;
using namespace BipedalLocomotion::ParametersHandler;


//TODO change to 26 joints

TEST_CASE("VelMANNTrajectoryGenerator")
{
    using namespace std::chrono_literals;

    auto jointsList
        = std::vector<std::string>{"l_hip_pitch", "l_hip_roll", "l_hip_yaw", "l_knee", "l_ankle_pitch", "l_ankle_roll",
                     "r_hip_pitch", "r_hip_roll", "r_hip_yaw", "r_knee", "r_ankle_pitch", "r_ankle_roll",
                     "torso_pitch", "torso_roll", "torso_yaw",
                     "neck_pitch", "neck_roll", "neck_yaw",
                     "l_shoulder_pitch", "l_shoulder_roll", "l_shoulder_yaw", "l_elbow",
                     "r_shoulder_pitch", "r_shoulder_roll", "r_shoulder_yaw", "r_elbow"};

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
    handler->setParameter("threshold_radius", 0.3);
    handler->setParameter("linear_pid_gain", 0.2);
    handler->setParameter("rotational_pid_gain", 2.0);

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
    mannGroup->setParameter("onnx_model_path", getVelMANNModelPath());

    handler->setGroup("LEFT_FOOT", leftFootGroup);
    handler->setGroup("RIGHT_FOOT", rightFootGroup);
    handler->setGroup("MANN", mannGroup);

    // input to generate a forward direction
    VelMANNTrajectoryGeneratorInput generatorInput;
    generatorInput.desiredFutureBaseTrajectory.resize(2, 7);
    generatorInput.desiredFutureBaseVelocities.resize(2, 7);
    generatorInput.desiredFutureBaseDirections.resize(2, 7);
    generatorInput.desiredFutureBaseAngVelocities.resize(7);
    generatorInput.desiredFutureBaseTrajectory << 0, 0.12, 0.22, 0.3, 0.35, 0.39, 0.4, 0, 0, 0, 0,
        0, 0, 0;
    generatorInput.desiredFutureBaseAngVelocities << 0, 0, 0, 0, 0, 0, 0;
    generatorInput.mergePointIndex = 0;

    for (int i = 0; i < generatorInput.desiredFutureBaseDirections.cols(); i++)
    {
        generatorInput.desiredFutureBaseDirections.col(i) << 1.0, 0;
        generatorInput.desiredFutureBaseVelocities.col(i) << 0.4, 0;
    }

    const manif::SE3d basePose = manif::SE3d(Eigen::Vector3d{0, 0, 0.7748},
                                             Eigen::AngleAxis(0.0, Eigen::Vector3d::UnitY()));

    Eigen::VectorXd jointPositions(26);
    jointPositions << -0.08914329577232137, 0.025767620112200747, 0.016600125582731447, -0.10205569019576242,
            -0.10115357046332556, -0.02590094449134414, -0.10954097755732813, -0.021888724926318617,
            0.06819316643211669, -0.07852679097651347, -0.10034170556770548, 0.020710812052444683,
            0.2413038455611485, 0.010535350226309968, 0.006275324178053386, -0.14908520025814864,
            0.08533421586871431, 0.10281322318047023, 0.32297257397277707, 0.14790247588361516,
            -0.3129451427487485, 0.04242320961879248, -0.12993538373842523, 0.018668904587540704,
            0.033567343049341246, 0.3631242921725555;

    VelMANNTrajectoryGenerator generator;
    REQUIRE(generator.setRobotModel(ml.model()));
    REQUIRE(generator.initialize(handler));
    REQUIRE(generator.setInitialState(jointPositions, basePose));
    REQUIRE(generator.setInput(generatorInput));
    REQUIRE(generator.advance());
    Eigen::Vector3d finalCoMPosition = generator.getOutput().comTrajectory.back();
    Eigen::Vector3d initialCoMPosition = generator.getOutput().comTrajectory.front();

    constexpr double forwardTolerance = 0.5;
    constexpr double lateralTolerance = 1.4;
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
