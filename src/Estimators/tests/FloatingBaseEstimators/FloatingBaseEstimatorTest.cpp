/**
 * @file FloatingBaseEstimatorTest.cpp
 * @authors Prashanth Ramadoss
 * @copyright 2020 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the BSD-3-Clause license.
 */

#include <catch2/catch_test_macros.hpp>

#include <Eigen/Dense>

#include <BipedalLocomotion/Conversions/ManifConversions.h>
#include <BipedalLocomotion/FloatingBaseEstimators/FloatingBaseEstimator.h>
#include <BipedalLocomotion/ParametersHandler/IParametersHandler.h>
#include <BipedalLocomotion/ParametersHandler/StdImplementation.h>

#include <iDynTree/TestUtils.h>
#include <iDynTree/ModelLoader.h>

#include <iCubModels/iCubModels.h>

using namespace BipedalLocomotion::Estimators;
using namespace BipedalLocomotion::ParametersHandler;
using namespace BipedalLocomotion::Conversions;

bool populateConfig(std::weak_ptr<IParametersHandler> handler)
{
    auto handle = handler.lock();
    if (handle == nullptr) {return false;}
    handle->setParameter("sampling_period_in_s", 0.01);

    auto modelInfoGroup = std::make_shared<StdImplementation>();
    handle->setGroup("ModelInfo", modelInfoGroup);
    modelInfoGroup->setParameter("base_link", "root_link");
    modelInfoGroup->setParameter("base_link_imu", "root_link_imu_acc");
    modelInfoGroup->setParameter("left_foot_contact_frame", "l_sole");
    modelInfoGroup->setParameter("right_foot_contact_frame", "r_sole");

    return true;
}

TEST_CASE("Bare Bones Base Estimator")
{
    std::shared_ptr<StdImplementation> originalHandler = std::make_shared<StdImplementation>();
    IParametersHandler::shared_ptr parameterHandler = originalHandler;

    // Populate the input configuration to be passed to the estimator
    REQUIRE(populateConfig(parameterHandler));

    // Load the reduced iDynTree model to be passed to the estimator
    const std::string model_path = iCubModels::getModelFile("iCubGenova02");
    std::cout << model_path << std::endl;
    std::vector<std::string> joints_list = {"neck_pitch", "neck_roll", "neck_yaw",
        "torso_pitch", "torso_roll", "torso_yaw",
        "l_shoulder_pitch", "l_shoulder_roll", "l_shoulder_yaw", "l_elbow",
        "r_shoulder_pitch", "r_shoulder_roll", "r_shoulder_yaw", "r_elbow",
        "l_hip_pitch", "l_hip_roll", "l_hip_yaw",
        "l_knee", "l_ankle_pitch", "l_ankle_roll",
        "r_hip_pitch", "r_hip_roll", "r_hip_yaw",
        "r_knee", "r_ankle_pitch", "r_ankle_roll"};

    iDynTree::ModelLoader mdl_ldr;
    REQUIRE(mdl_ldr.loadReducedModelFromFile(model_path, joints_list));

    auto model = mdl_ldr.model().copy();

    auto kinDyn = std::make_shared<iDynTree::KinDynComputations>();
    kinDyn->loadRobotModel(model);
    // Instantiate the estimator
    FloatingBaseEstimator estimator;
    REQUIRE(estimator.initialize(parameterHandler, kinDyn));
    REQUIRE(estimator.modelComputations().nrJoints() == joints_list.size());
    REQUIRE(estimator.modelComputations().baseLink() == "root_link");
    REQUIRE(estimator.modelComputations().baseLinkIMU() == "root_link_imu_acc");
    REQUIRE(estimator.modelComputations().leftFootContactFrame() == "l_sole");
    REQUIRE(estimator.modelComputations().rightFootContactFrame() == "r_sole");

    auto b_H_imu = toManifPose(model.getFrameTransform(model.getFrameIndex("root_link_imu_acc")));
    constexpr double tolerance = 1e-5;
    REQUIRE (b_H_imu.coeffs().isApprox(estimator.modelComputations().base_H_IMU().coeffs(), tolerance));

    // IMU measures
    Eigen::Vector3d acc, gyro;
    acc << 0.0,   -7.9431,   -5.7513;
    gyro << 0.0, 0.0, 0.0;

    // contact states
    bool lf_contact{true}, rf_contact{true};

    // kinematic measures
    Eigen::VectorXd encoders(joints_list.size()), encoder_speeds(joints_list.size());
    encoders << -0.0001, 0.0000, 0.0000,
    0.1570, 0.0003, -0.0000,
    -0.0609, 0.4350, 0.1833, 0.5375,
    -0.0609,    0.4349, 0.1834, 0.5375,
    0.0895, 0.0090, -0.0027,
    -0.5694, -0.3771, -0.0211,
    0.0896, 0.0090, -0.0027,
    -0.5695, -0.3771, -0.0211;

    encoder_speeds.setZero();

    // set measurements and advance the estimator
    for (int i = 0; i < 10000; i++)
    {
        REQUIRE(estimator.setIMUMeasurement(acc, gyro));
        REQUIRE(estimator.setContacts(lf_contact, rf_contact));
        REQUIRE(estimator.setKinematics(encoders, encoder_speeds));
        REQUIRE(estimator.advance());
    }

}
