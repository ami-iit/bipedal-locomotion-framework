/**
 * @file FrictionTorqueDynamicsTest.cpp
 * @authors Ines Sorrentino
 * @copyright 2023 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the BSD-3-Clause license.
 */

#include <catch2/catch_test_macros.hpp>
#include <catch2/generators/catch_generators_all.hpp>

#include <ConfigFolderPath.h>
#include <iCubModels/iCubModels.h>
#include <yarp/os/ResourceFinder.h>

#include <BipedalLocomotion/Math/Constants.h>
#include <BipedalLocomotion/ParametersHandler/IParametersHandler.h>
#include <BipedalLocomotion/ParametersHandler/StdImplementation.h>
#include <BipedalLocomotion/ParametersHandler/YarpImplementation.h>
#include <BipedalLocomotion/System/VariablesHandler.h>

#include <BipedalLocomotion/RobotDynamicsEstimator/FrictionTorqueStateDynamics.h>

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
    //    jointsAndFTs.insert(jointsAndFTs.end(), ftFramesList.begin(), ftFramesList.end());

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

    const std::string name = "tau_F";
    Eigen::VectorXd covariance(6);
    covariance << 1e-3, 1e-3, 5e-3, 5e-3, 5e-3, 5e-3;
    const std::string model = "FrictionTorqueStateDynamics";
    const std::vector<std::string> elements
        = {"r_hip_pitch", "r_hip_roll", "r_hip_yaw", "r_knee", "r_ankle_pitch", "r_ankle_roll"};
    Eigen::VectorXd k0(6);
    k0 << 9.106, 5.03, 4.93, 12.88, 14.34, 1.12;
    Eigen::VectorXd k1(6);
    k1 << 200.0, 6.9, 200.0, 59.87, 200.0, 200.0;
    Eigen::VectorXd k2(6);
    k2 << 1.767, 5.64, 0.27, 2.0, 3.0, 0.0;
    double dT = 0.01;

    parameterHandler->setParameter("name", name);
    parameterHandler->setParameter("covariance", covariance);
    parameterHandler->setParameter("initial_covariance", covariance);
    parameterHandler->setParameter("dynamic_model", model);
    parameterHandler->setParameter("elements", elements);
    parameterHandler->setParameter("friction_k0", k0);
    parameterHandler->setParameter("friction_k1", k1);
    parameterHandler->setParameter("friction_k2", k2);
    parameterHandler->setParameter("sampling_time", dT);

    // Create variable handler
    constexpr size_t sizeVariable = 6;
    VariablesHandler variableHandler;
    REQUIRE(variableHandler.addVariable("ds", sizeVariable));
    REQUIRE(variableHandler.addVariable("tau_m", sizeVariable));
    REQUIRE(variableHandler.addVariable("tau_F", sizeVariable));

    auto modelParamHandler = std::make_shared<StdImplementation>();
    auto emptyGroupNamesFrames = std::make_shared<StdImplementation>();
    std::vector<std::string> emptyVectorString;
    emptyGroupNamesFrames->setParameter("names", emptyVectorString);
    emptyGroupNamesFrames->setParameter("frames", emptyVectorString);
    emptyGroupNamesFrames->setParameter("associated_joints", emptyVectorString);
    REQUIRE(modelParamHandler->setGroup("FT", emptyGroupNamesFrames));
    REQUIRE(modelParamHandler->setGroup("ACCELEROMETER", emptyGroupNamesFrames));
    REQUIRE(modelParamHandler->setGroup("GYROSCOPE", emptyGroupNamesFrames));
    auto emptyGroupFrames = std::make_shared<StdImplementation>();
    emptyGroupFrames->setParameter("frames", emptyVectorString);
    REQUIRE(modelParamHandler->setGroup("EXTERNAL_CONTACT", emptyGroupFrames));
    modelParamHandler->setParameter("joint_list", elements);

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

    manif::SE3d::Tangent baseVelocity, baseAcceleration;
    baseVelocity.setZero();
    baseAcceleration.setZero();

    for (int idx = 0; idx < subModelCreator.getSubModelList().size(); idx++)
    {
        kinDynWrapperList.emplace_back(std::make_shared<SubModelKinDynWrapper>());
        REQUIRE(kinDynWrapperList.at(idx)->setKinDyn(kinDyn));
        REQUIRE(kinDynWrapperList.at(idx)->initialize(subModelList[idx]));
        REQUIRE(
            kinDynWrapperList.at(idx)
                ->updateState(baseAcceleration,
                              Eigen::VectorXd(subModelList[idx].getModel().getNrOfDOFs()).setZero(),
                              UpdateMode::Full));
    }

    FrictionTorqueStateDynamics tau_F_dynamics;

    REQUIRE(tau_F_dynamics.setSubModels(subModelList, kinDynWrapperList));
    REQUIRE(tau_F_dynamics.initialize(parameterHandler));
    REQUIRE(tau_F_dynamics.finalize(variableHandler));

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
    state.resize(variableHandler.getVariable("tau_F").offset
                 + variableHandler.getVariable("tau_F").size + 1);
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
    input.robotBaseVelocity = baseVelocity;
    input.robotBaseAcceleration = baseAcceleration;

    input.robotJointAccelerations = Eigen::VectorXd(kinDyn->model().getNrOfDOFs()).setZero();

    input.robotJointAccelerations = Eigen::VectorXd(kinDyn->model().getNrOfDOFs()).setZero();

    for (int idx = 0; idx < subModelCreator.getSubModelList().size(); idx++)
    {
        REQUIRE(kinDynWrapperList.at(idx)->updateState(input.robotBaseAcceleration,
                                                       input.robotJointAccelerations,
                                                       UpdateMode::Full));
    }

    tau_F_dynamics.setInput(input);
    tau_F_dynamics.setState(state);

    REQUIRE(tau_F_dynamics.update());
    for (int idx = 0; idx < tau_F_dynamics.getUpdatedVariable().size(); idx++)
    {
        REQUIRE(std::abs(tau_F_dynamics.getUpdatedVariable()(idx)) < 0.1);
    }

    // Set random friction torque
    Eigen::VectorXd currentStateTauF(covariance.size());
    for (int index = 0; index < currentStateTauF.size(); index++)
    {
        currentStateTauF(index) = GENERATE(take(1, random(-30, 30)));
    }

    state.segment(variableHandler.getVariable("tau_F").offset,
                  variableHandler.getVariable("tau_F").size)
        = currentStateTauF;

    for (int idx = 0; idx < subModelCreator.getSubModelList().size(); idx++)
    {
        REQUIRE(kinDynWrapperList.at(idx)->updateState(input.robotBaseAcceleration,
                                                       input.robotJointAccelerations,
                                                       UpdateMode::Full));
    }

    tau_F_dynamics.setInput(input);

    tau_F_dynamics.setState(state);

    REQUIRE(tau_F_dynamics.update());
}
