/*
 * @file BaseEstimatorFromFootIMUTest.cpp
 * @authors Guglielmo Cervettini
 * @copyright 2023 Istituto Italiano di Tecnologia (IIT). This software may be
 * modified and distributed under the terms of the BSD-3-Clause license.
 */

// Catch2
#include <BipedalLocomotion/Conversions/ManifConversions.h>
#include <BipedalLocomotion/FloatingBaseEstimators/BaseEstimatorFromFootIMU.h>
#include <BipedalLocomotion/ParametersHandler/IParametersHandler.h>
#include <BipedalLocomotion/ParametersHandler/StdImplementation.h>
#include <iCubModels/iCubModels.h>
#include <iDynTree/TestUtils.h>
#include <iDynTree/ModelLoader.h>

#include <Eigen/Dense>
#include <catch2/catch_test_macros.hpp>

using namespace BipedalLocomotion::Estimators;
using namespace BipedalLocomotion::ParametersHandler;
using namespace BipedalLocomotion::Conversions;

bool populateConfig(std::weak_ptr<IParametersHandler> handler, const std::string& footFrameName)
{
    auto handle = handler.lock();
    if (handle == nullptr)
    {
        return false;
    }

    handle->setParameter("foot_width_in_m", 0.1);
    handle->setParameter("foot_length_in_m", 0.236);

    auto modelInfoGroup = std::make_shared<StdImplementation>();
    handle->setGroup("MODEL_INFO", modelInfoGroup);
    modelInfoGroup->setParameter("base_frame", "root_link");
    modelInfoGroup->setParameter("foot_frame", footFrameName);

    return true;
}

TEST_CASE("BaseEstimatorFromFootIMU")
{
    std::shared_ptr<StdImplementation> originalHandler = std::make_shared<StdImplementation>();
    IParametersHandler::shared_ptr parameterHandler = originalHandler;

    const std::string footFrameName = "r_sole";

    // Populate the input configuration to be passed to the estimator
    REQUIRE(populateConfig(parameterHandler, footFrameName));

    // Load the reduced iDynTree model to be passed to the estimator
    const std::string model_path = iCubModels::getModelFile("iCubGazeboV3");
    std::vector<std::string> joints_list
        = {"neck_pitch",     "neck_roll",   "neck_yaw",         "torso_pitch",
           "torso_roll",     "torso_yaw",   "l_shoulder_pitch", "l_shoulder_roll",
           "l_shoulder_yaw", "l_elbow",     "r_shoulder_pitch", "r_shoulder_roll",
           "r_shoulder_yaw", "r_elbow",     "l_hip_pitch",      "l_hip_roll",
           "l_hip_yaw",      "l_knee",      "l_ankle_pitch",    "l_ankle_roll",
           "r_hip_pitch",    "r_hip_roll",  "r_hip_yaw",        "r_knee",
           "r_ankle_pitch",  "r_ankle_roll"};

    iDynTree::ModelLoader mdl_ldr;
    REQUIRE(mdl_ldr.loadReducedModelFromFile(model_path, joints_list));

    auto model = mdl_ldr.model().copy();

    auto kinDyn = std::make_shared<iDynTree::KinDynComputations>();
    kinDyn->loadRobotModel(model);

    // Instantiate the estimator
    BaseEstimatorFromFootIMU estimator;

    REQUIRE(estimator.setModel(model));
    REQUIRE(estimator.initialize(parameterHandler));

    // kinematic measures
    Eigen::VectorXd encoders(joints_list.size()), encoder_speeds(joints_list.size());

    encoders << 0.0000, 0.0000, 0.0000, 0.0000, 0.0000, 0.0000, 0.0000, 0.0000, 0.0000, 0.0000,
        0.0000, 0.0000, 0.0000, 0.0000, 0.0000, 0.0000, 0.0000, -1.5694, -0.3771, -0.0000, 0.0000,
        0.0000, 0.0000, -1.5694, -0.3771, -0.0000;

    encoder_speeds.setZero();

    Eigen::VectorXd baseVelocity(6);
    baseVelocity.setZero();
    Eigen::Vector3d gravity;
    gravity << 0, 0, -BipedalLocomotion::Math::StandardAccelerationOfGravitation;
    manif::SE3d I = manif::SE3d::Identity();

    REQUIRE(kinDyn->setRobotState(I.transform(), encoders, baseVelocity, encoder_speeds, gravity));

    BaseEstimatorFromFootIMUInput input;
    input.jointPositions = encoders;
    input.jointVelocities = encoder_speeds;
    input.desiredFootPose
        = BipedalLocomotion::Conversions::toManifPose(kinDyn->getWorldTransform(footFrameName));
    input.measuredRotation = BipedalLocomotion::Conversions::toManifRot(
        kinDyn->getWorldTransform(footFrameName).getRotation());

    REQUIRE(estimator.setInput(input));
    REQUIRE(estimator.advance());
    REQUIRE(estimator.isOutputValid());

    constexpr double tolerance = 1e-3;
    REQUIRE(estimator.getOutput().basePose.coeffs().isApprox(I.coeffs(), tolerance));
}
