/**
 * @file AccelerometerMeasurementDynamicsTest.cpp
 * @authors Ines Sorrentino
 * @copyright 2023 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the BSD-3-Clause license.
 */

#include <catch2/catch_test_macros.hpp>

#include <ConfigFolderPath.h>
#include <iCubModels/iCubModels.h>
#include <yarp/os/ResourceFinder.h>

#include <BipedalLocomotion/Math/Constants.h>
#include <BipedalLocomotion/ParametersHandler/IParametersHandler.h>
#include <BipedalLocomotion/ParametersHandler/StdImplementation.h>
#include <BipedalLocomotion/ParametersHandler/YarpImplementation.h>
#include <BipedalLocomotion/System/VariablesHandler.h>

#include <BipedalLocomotion/RobotDynamicsEstimator/AccelerometerMeasurementDynamics.h>

using namespace BipedalLocomotion::Estimators::RobotDynamicsEstimator;
using namespace BipedalLocomotion::ParametersHandler;
using namespace BipedalLocomotion::System;

void createModelLoader(IParametersHandler::shared_ptr group, iDynTree::ModelLoader& mdlLdr)
{
    // List of joints and fts to load the model
    std::vector<SubModel> subModelList;

    const std::string modelPath = iCubModels::getModelFile("iCubGenova09");

    std::vector<std::string> jointList;
    REQUIRE(group->getParameter("joint_list", jointList));

    std::vector<std::string> ftFramesList;
    auto ftGroup = group->getGroup("FT").lock();
    REQUIRE(ftGroup->getParameter("associated_joints", ftFramesList));

    std::vector<std::string> jointsAndFTs;
    jointsAndFTs.insert(jointsAndFTs.begin(), jointList.begin(), jointList.end());

    REQUIRE(mdlLdr.loadReducedModelFromFile(modelPath, jointsAndFTs));
}

void createSubModels(iDynTree::ModelLoader& mdlLdr,
                     std::shared_ptr<iDynTree::KinDynComputations> kinDynFullModel,
                     IParametersHandler::shared_ptr group,
                     SubModelCreator& subModelCreator)
{
    subModelCreator.setModelAndSensors(mdlLdr.model(), mdlLdr.sensors());
    REQUIRE(subModelCreator.setKinDyn(kinDynFullModel));

    REQUIRE(subModelCreator.createSubModels(group));
}

bool setStaticState(std::shared_ptr<iDynTree::KinDynComputations> kinDyn)
{
    size_t dofs = kinDyn->getNrOfDegreesOfFreedom();
    iDynTree::Transform worldTbase;
    Eigen::VectorXd baseVel(6);
    Eigen::Vector3d gravity;

    Eigen::VectorXd qj(dofs), dqj(dofs), ddqj(dofs);

    worldTbase = iDynTree::Transform(iDynTree::Rotation::Identity(), iDynTree::Position::Zero());

    Eigen::Matrix4d transform = toEigen(worldTbase.asHomogeneousTransform());

    gravity.setZero();
    gravity(2) = -BipedalLocomotion::Math::StandardAccelerationOfGravitation;

    baseVel.setZero();

    for (size_t dof = 0; dof < dofs; dof++)
    {
        qj(dof) = 0.0;
        dqj(dof) = 0.0;
        ddqj(dof) = 0.0;
    }

    return kinDyn->setRobotState(transform,
                                 iDynTree::make_span(qj.data(), qj.size()),
                                 iDynTree::make_span(baseVel.data(), baseVel.size()),
                                 iDynTree::make_span(dqj.data(), dqj.size()),
                                 iDynTree::make_span(gravity.data(), gravity.size()));
}

TEST_CASE("Friction Torque Dynamics")
{
    // Create parameter handler
    auto parameterHandler = std::make_shared<StdImplementation>();

    const std::string name = "r_leg_ft_acc";
    Eigen::VectorXd covariance(3);
    covariance << 2.3e-3, 1.9e-3, 3.1e-3;
    const std::string model = "AccelerometerMeasurementDynamics";
    const bool useBias = true;

    const std::vector<std::string> jointList
        = {"r_hip_pitch", "r_hip_roll", "r_hip_yaw", "r_knee", "r_ankle_pitch", "r_ankle_roll"};

    parameterHandler->setParameter("name", name);
    parameterHandler->setParameter("covariance", covariance);
    parameterHandler->setParameter("dynamic_model", model);
    parameterHandler->setParameter("use_bias", useBias);

    // Create state variable handler
    constexpr size_t sizeVariable = 6;
    VariablesHandler variableHandler;
    REQUIRE(variableHandler.addVariable("ds", sizeVariable));
    REQUIRE(variableHandler.addVariable("tau_m", sizeVariable));
    REQUIRE(variableHandler.addVariable("tau_F", sizeVariable));
    REQUIRE(variableHandler.addVariable("r_leg_ft_acc_bias", 3));

    // Create model variable handler to load the robot model
    auto modelParamHandler = std::make_shared<StdImplementation>();
    auto emptyGroupNamesFrames = std::make_shared<StdImplementation>();
    std::vector<std::string> emptyVectorString;
    emptyGroupNamesFrames->setParameter("names", emptyVectorString);
    emptyGroupNamesFrames->setParameter("frames", emptyVectorString);
    emptyGroupNamesFrames->setParameter("associated_joints", emptyVectorString);
    REQUIRE(modelParamHandler->setGroup("FT", emptyGroupNamesFrames));
    REQUIRE(modelParamHandler->setGroup("GYROSCOPE", emptyGroupNamesFrames));

    auto accGroup = std::make_shared<StdImplementation>();
    std::vector<std::string> accNameList = {"r_leg_ft_acc"};
    std::vector<std::string> accFrameList = {"r_leg_ft"};
    accGroup->setParameter("names", accNameList);
    accGroup->setParameter("frames", accFrameList);
    REQUIRE(modelParamHandler->setGroup("ACCELEROMETER", accGroup));

    auto emptyGroupFrames = std::make_shared<StdImplementation>();
    emptyGroupFrames->setParameter("frames", emptyVectorString);
    REQUIRE(modelParamHandler->setGroup("EXTERNAL_CONTACT", emptyGroupFrames));

    modelParamHandler->setParameter("joint_list", jointList);

    // Load model
    iDynTree::ModelLoader modelLoader;
    createModelLoader(modelParamHandler, modelLoader);

    auto kinDyn = std::make_shared<iDynTree::KinDynComputations>();
    REQUIRE(kinDyn->loadRobotModel(modelLoader.model()));
    REQUIRE(kinDyn->setFrameVelocityRepresentation(iDynTree::BODY_FIXED_REPRESENTATION));

    REQUIRE(setStaticState(kinDyn));

    SubModelCreator subModelCreator;
    createSubModels(modelLoader, kinDyn, modelParamHandler, subModelCreator);

    std::vector<std::shared_ptr<SubModelKinDynWrapper>> kinDynWrapperList;
    const auto& subModelList = subModelCreator.getSubModelList();

    for (int idx = 0; idx < subModelCreator.getSubModelList().size(); idx++)
    {
        kinDynWrapperList.emplace_back(std::make_shared<SubModelKinDynWrapper>());
        REQUIRE(kinDynWrapperList.at(idx)->setKinDyn(kinDyn));
        REQUIRE(kinDynWrapperList.at(idx)->initialize(subModelList[idx]));
    }

    AccelerometerMeasurementDynamics accDynamics;
    REQUIRE(accDynamics.setSubModels(subModelList, kinDynWrapperList));
    REQUIRE(accDynamics.initialize(parameterHandler));
    REQUIRE(accDynamics.finalize(variableHandler));

    manif::SE3d::Tangent robotBaseAcceleration;
    robotBaseAcceleration.setZero();

    Eigen::VectorXd robotJointAcceleration(kinDyn->model().getNrOfDOFs());
    robotJointAcceleration.setZero();

    iDynTree::LinkNetExternalWrenches extWrench(kinDyn->model());
    extWrench.zero();

    // Compute joint torques in static configuration from inverse dynamics on the full model
    iDynTree::FreeFloatingGeneralizedTorques jointTorques(kinDyn->model());

    kinDyn->inverseDynamics(iDynTree::make_span(robotBaseAcceleration.data(),
                                                manif::SE3d::Tangent::DoF),
                            robotJointAcceleration,
                            extWrench,
                            jointTorques);

    Eigen::VectorXd state;
    state.resize(variableHandler.getNumberOfVariables());
    state.setZero();

    int offset = variableHandler.getVariable("tau_m").offset;
    int size = variableHandler.getVariable("tau_m").size;
    for (int jointIndex = 0; jointIndex < size; jointIndex++)
    {
        state[offset + jointIndex] = jointTorques.jointTorques()[jointIndex];
    }

    // Create an input for the ukf state
    UKFInput input;

    // Define joint positions
    Eigen::VectorXd jointPos;
    jointPos.resize(kinDyn->model().getNrOfDOFs());
    jointPos.setZero();
    input.robotJointPositions = jointPos;

    // Define base pose
    manif::SE3d basePose;
    basePose.setIdentity();
    input.robotBasePose = basePose;

    // Define base velocity and acceleration
    manif::SE3d::Tangent baseVelocity, baseAcceleration;
    baseVelocity.setZero();
    baseAcceleration.setZero();
    input.robotBaseVelocity = baseVelocity;
    input.robotBaseAcceleration = baseAcceleration;

    input.robotJointAccelerations = Eigen::VectorXd(kinDyn->model().getNrOfDOFs()).setZero();

    for (int idx = 0; idx < subModelCreator.getSubModelList().size(); idx++)
    {
        REQUIRE(kinDynWrapperList.at(idx)->updateState(baseAcceleration,
                                                       input.robotJointAccelerations,
                                                       UpdateMode::Full));
    }

    accDynamics.setInput(input);
    accDynamics.setState(state);

    REQUIRE(accDynamics.update());
    for (int idx = 0; idx < accDynamics.getUpdatedVariable().size(); idx++)
    {
        REQUIRE(std::abs(accDynamics.getUpdatedVariable()(idx)) < 10);
    }
}
