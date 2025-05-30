/**
 * @file GyroscopeMeasurementDynamicsTest.cpp
 * @authors Ines Sorrentino
 * @copyright 2023 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the BSD-3-Clause license.
 */

#include <catch2/catch_test_macros.hpp>
#include <catch2/generators/catch_generators_all.hpp>

#include <yarp/os/ResourceFinder.h>
#include <ConfigFolderPath.h>

#include <iDynTree/KinDynComputations.h>
#include <iDynTree/FreeFloatingState.h>
#include <iDynTree/Model.h>
#include <iDynTree/ModelTestUtils.h>
#include <iDynTree/ModelLoader.h>

#include <BipedalLocomotion/Conversions/ManifConversions.h>
#include <BipedalLocomotion/Math/Constants.h>
#include <BipedalLocomotion/ParametersHandler/IParametersHandler.h>
#include <BipedalLocomotion/ParametersHandler/StdImplementation.h>
#include <BipedalLocomotion/System/VariablesHandler.h>

#include <BipedalLocomotion/RobotDynamicsEstimator/GyroscopeMeasurementDynamics.h>

using namespace BipedalLocomotion::Estimators::RobotDynamicsEstimator;
using namespace BipedalLocomotion::ParametersHandler;
using namespace BipedalLocomotion::System;

void createModelLoader(IParametersHandler::shared_ptr group, iDynTree::ModelLoader& mdlLdr)
{
    // List of joints and fts to load the model
    std::vector<SubModel> subModelList;

    std::vector<std::string> jointList;
    REQUIRE(group->getParameter("joint_list", jointList));

    std::vector<std::string> ftFramesList;
    auto ftGroup = group->getGroup("FT").lock();
    REQUIRE(ftGroup->getParameter("associated_joints", ftFramesList));

    std::vector<std::string> jointsAndFTs;
    jointsAndFTs.insert(jointsAndFTs.begin(), jointList.begin(), jointList.end());

    REQUIRE(mdlLdr.loadReducedModelFromFile(getRobotModelPath(), jointsAndFTs));
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

IParametersHandler::shared_ptr createModelParameterHandler()
{
    // Create model variable handler to load the robot model
    auto modelParamHandler = std::make_shared<StdImplementation>();

    const std::vector<std::string> jointList
        = {"r_hip_pitch", "r_hip_roll", "r_hip_yaw", "r_knee", "r_ankle_pitch", "r_ankle_roll"};

    auto emptyGroupNamesFrames = std::make_shared<StdImplementation>();
    std::vector<std::string> emptyVectorString;
    emptyGroupNamesFrames->setParameter("names", emptyVectorString);
    emptyGroupNamesFrames->setParameter("frames", emptyVectorString);

    REQUIRE(modelParamHandler->setGroup("ACCELEROMETER", emptyGroupNamesFrames));
    REQUIRE(modelParamHandler->setGroup("EXTERNAL_CONTACT", emptyGroupNamesFrames));

    emptyGroupNamesFrames->setParameter("associated_joints", emptyVectorString);

    REQUIRE(modelParamHandler->setGroup("FT", emptyGroupNamesFrames));

    auto gyroGroup = std::make_shared<StdImplementation>();
    std::vector<std::string> gyroNameList = {"r_foot_rear_ft_imu"};
    std::vector<std::string> gyroFrameList = {"r_foot_rear_ft_sensor"};
    gyroGroup->setParameter("names", gyroNameList);
    gyroGroup->setParameter("frames", gyroFrameList);
    REQUIRE(modelParamHandler->setGroup("GYROSCOPE", gyroGroup));

    modelParamHandler->setParameter("joint_list", jointList);

    return modelParamHandler;
}

void createUkfInput(VariablesHandler& stateVariableHandler, UKFInput& input)
{
    Eigen::VectorXd jointPos = Eigen::VectorXd::Random(stateVariableHandler.getVariable("JOINT_VELOCITIES").size);
    input.robotJointPositions = jointPos;

    Eigen::VectorXd jointAcc = Eigen::VectorXd::Random(stateVariableHandler.getVariable("JOINT_VELOCITIES").size);
    input.robotJointAccelerations = jointAcc;

    manif::SE3d basePose
        = BipedalLocomotion::Conversions::toManifPose(iDynTree::getRandomTransform());
    input.robotBasePose = basePose;

    manif::SE3Tangentd baseVel
        = BipedalLocomotion::Conversions::toManifTwist(iDynTree::getRandomTwist());
    input.robotBaseVelocity = baseVel;

    manif::SE3Tangentd baseAcc
        = BipedalLocomotion::Conversions::toManifTwist(iDynTree::getRandomTwist());
    input.robotBaseAcceleration = baseAcc;
}

void createStateVector(UKFInput& input,
    VariablesHandler& stateVariableHandler,
    std::shared_ptr<iDynTree::KinDynComputations> kinDyn,
    Eigen::Ref<Eigen::VectorXd> state)
{
    state.setZero();

    Eigen::VectorXd jointVel = Eigen::VectorXd::Random(stateVariableHandler.getVariable("JOINT_VELOCITIES").size);

    int offset = stateVariableHandler.getVariable("JOINT_VELOCITIES").offset;
    int size = stateVariableHandler.getVariable("JOINT_VELOCITIES").size;
    for (int jointIndex = 0; jointIndex < size; jointIndex++)
    {
    state[offset + jointIndex] = jointVel(jointIndex);
    }

    // Compute joint torques from inverse dynamics on the full model
    offset = stateVariableHandler.getVariable("MOTOR_TORQUES").offset;
    size = stateVariableHandler.getVariable("MOTOR_TORQUES").size;
    iDynTree::LinkNetExternalWrenches extWrench(kinDyn->model());
    extWrench.zero();
    iDynTree::FreeFloatingGeneralizedTorques jointTorques(kinDyn->model());
    kinDyn->inverseDynamics(iDynTree::make_span(input.robotBaseAcceleration.data(),
                                manif::SE3d::Tangent::DoF),
            input.robotJointAccelerations,
            extWrench,
            jointTorques);
    for (int jointIndex = 0; jointIndex < size; jointIndex++)
    {
    state[offset + jointIndex] = jointTorques.jointTorques()[jointIndex];
    }
}

void setRandomKinDynState(std::vector<SubModel>& subModelList,
    std::vector<std::shared_ptr<KinDynWrapper>>& kinDynWrapperList,
    std::shared_ptr<iDynTree::KinDynComputations> kinDyn,
    UKFInput& input,
    Eigen::VectorXd& state,
    VariablesHandler& stateVariableHandler)
{
    std::vector<Eigen::VectorXd> subModelJointPos(subModelList.size()); /**< List of sub-model joint
                                                        velocities. */
    std::vector<Eigen::VectorXd> subModelJointVel(subModelList.size()); /**< List of sub-model joint
                                                        velocities. */
    Eigen::Vector3d gravity;
    gravity.setZero();
    gravity(2) = -BipedalLocomotion::Math::StandardAccelerationOfGravitation;

    int offset = stateVariableHandler.getVariable("JOINT_VELOCITIES").offset;
    int size = stateVariableHandler.getVariable("JOINT_VELOCITIES").size;

    Eigen::VectorXd jointVel(size);
    for (int jointIndex = 0; jointIndex < size; jointIndex++)
    {
    jointVel(jointIndex) = state[offset + jointIndex];
    }

    REQUIRE(kinDyn->setRobotState(input.robotBasePose.transform(),
                input.robotJointPositions,
                iDynTree::make_span(input.robotBaseVelocity.data(),
                                    manif::SE3d::Tangent::DoF),
                jointVel,
                gravity));

    // The submodel is only one and it is the full model
    // Indeed the model does not have ft sensors in this test
    // Get sub-model base pose
    manif::SE3d worldTBase = BipedalLocomotion::Conversions::toManifPose(
    kinDyn->getWorldTransform(kinDynWrapperList[0]->getFloatingBase()));

    subModelJointPos[0].resize(subModelList[0].getModel().getNrOfDOFs());

    // Get sub-model joint position
    for (int jointIdx = 0; jointIdx < subModelList[0].getModel().getNrOfDOFs(); jointIdx++)
    {
    subModelJointPos[0](jointIdx)
    = input.robotJointPositions[subModelList[0].getJointMapping()[jointIdx]];
    }

    // Get sub-model joint velocities
    subModelJointVel[0].resize(subModelList[0].getModel().getNrOfDOFs());
    offset = stateVariableHandler.getVariable("JOINT_VELOCITIES").offset;
    size = stateVariableHandler.getVariable("JOINT_VELOCITIES").size;
    for (int jointIdx = 0; jointIdx < subModelList[0].getModel().getNrOfDOFs(); jointIdx++)
    {
    subModelJointVel[0](jointIdx) = state[offset + subModelList[0].getJointMapping()[jointIdx]];
    }

    // Set the sub-model state
    REQUIRE(kinDynWrapperList[0]->setRobotState(worldTBase.transform(),
                    subModelJointPos[0],
                    iDynTree::make_span(input.robotBaseVelocity.data(),
                                        manif::SE3d::Tangent::DoF),
                    subModelJointVel[0],
                    gravity));
}

TEST_CASE("Gyroscope Measurement Dynamics")
{
    // Create gyroscope parameter handler
    auto gyroHandler = std::make_shared<StdImplementation>();

    const std::string variable_name = "r_foot_rear_ft_imu";

    Eigen::VectorXd covariance(3);
    covariance << 2.3e-3, 1.9e-3, 1.8e-3;

    const std::string dynModel = "GyroscopeMeasurementDynamics";
    const std::string sensorType = "gyroscope";
    const bool useBias = false;

    gyroHandler->setParameter("variable_name", variable_name);
    gyroHandler->setParameter("covariance", covariance);
    gyroHandler->setParameter("dynamic_model", dynModel);
    gyroHandler->setParameter("use_bias", useBias);
    gyroHandler->setParameter("sensor_type", sensorType);

    // Create state variable handler
    constexpr size_t sizeVariable = 6;
    VariablesHandler stateVariableHandler;
    REQUIRE(stateVariableHandler.addVariable("JOINT_VELOCITIES", sizeVariable));
    REQUIRE(stateVariableHandler.addVariable("MOTOR_TORQUES", sizeVariable));
    REQUIRE(stateVariableHandler.addVariable("FRICTION_TORQUES", sizeVariable));

    // Create parameter handler to load the model
    auto modelParamHandler = createModelParameterHandler();

    // Load model
    iDynTree::ModelLoader modelLoader;
    createModelLoader(modelParamHandler, modelLoader);

    auto kinDyn = std::make_shared<iDynTree::KinDynComputations>();
    REQUIRE(kinDyn->loadRobotModel(modelLoader.model()));
    REQUIRE(kinDyn->setFrameVelocityRepresentation(iDynTree::BODY_FIXED_REPRESENTATION));

    REQUIRE(setStaticState(kinDyn));

    SubModelCreator subModelCreator;
    createSubModels(modelLoader, kinDyn, modelParamHandler, subModelCreator);

    std::vector<std::shared_ptr<KinDynWrapper>> kinDynWrapperList;
    std::vector<SubModel> subModelList = subModelCreator.getSubModelList();

    for (int idx = 0; idx < subModelCreator.getSubModelList().size(); idx++)
    {
        kinDynWrapperList.emplace_back(std::make_shared<KinDynWrapper>());
        REQUIRE(kinDynWrapperList[idx]->setModel(subModelList[idx]));
    }

    // Create the dynamics
    GyroscopeMeasurementDynamics gyroDynamics;
    REQUIRE(gyroDynamics.setSubModels(subModelList, kinDynWrapperList));
    REQUIRE(gyroDynamics.initialize(gyroHandler, "r_foot_rear_ft_imu"));
    REQUIRE(gyroDynamics.finalize(stateVariableHandler));

    // Create an input for the ukf state
    UKFInput input;
    createUkfInput(stateVariableHandler, input);

    // Create the state vector
    Eigen::VectorXd state;
    state.resize(stateVariableHandler.getNumberOfVariables());
    createStateVector(input, stateVariableHandler, kinDyn, state);

    // Set the kindyn submodel state
    setRandomKinDynState(subModelList,
                         kinDynWrapperList,
                         kinDyn,
                         input,
                         state,
                         stateVariableHandler);

    // Set input and state to the dynamics
    gyroDynamics.setInput(input);
    gyroDynamics.setState(state);

    // Update the dynamics
    REQUIRE(gyroDynamics.update());

    manif::SE3Tangentd gyroFrameVel
        = BipedalLocomotion::Conversions::toManifTwist(kinDyn->getFrameVel("r_foot_rear_ft_sensor"));

    // Check the output
    for (int idx = 0; idx < gyroDynamics.getUpdatedVariable().size(); idx++)
    {
        REQUIRE(std::abs(gyroDynamics.getUpdatedVariable()(idx)
                         - gyroFrameVel.coeffs()(idx + 3))
                < 0.1);
    }
}
