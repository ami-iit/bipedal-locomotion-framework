/**
 * @file KinDynWrapperTest.cpp
 * @authors Ines Sorrentino
 * @copyright 2023 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the BSD-3-Clause license.
 */

#include <catch2/catch_test_macros.hpp>
#include <catch2/generators/catch_generators_all.hpp>

#include <iCubModels/iCubModels.h>
#include <yarp/os/ResourceFinder.h>

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

#include <BipedalLocomotion/RobotDynamicsEstimator/KinDynWrapper.h>

using namespace BipedalLocomotion::Estimators::RobotDynamicsEstimator;
using namespace BipedalLocomotion::ParametersHandler;
using namespace BipedalLocomotion::System;
using namespace BipedalLocomotion;

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
    manif::SE3Tangentd baseVel;
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
                                 qj,
                                 iDynTree::make_span(baseVel.data(), manif::SE3d::Tangent::DoF),
                                 dqj,
                                 gravity);
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
    emptyGroupNamesFrames->setParameter("ukf_names", emptyVectorString);
    emptyGroupNamesFrames->setParameter("associated_joints", emptyVectorString);
    REQUIRE(modelParamHandler->setGroup("FT", emptyGroupNamesFrames));
    auto gyroGroup = std::make_shared<StdImplementation>();
    std::vector<std::string> gyroNameList = {"root_link"};
    std::vector<std::string> gyroFrameList = {"root_link"};
    std::vector<std::string> gyroUkfNameList = {"root_link_gyro"};
    gyroGroup->setParameter("names", gyroNameList);
    gyroGroup->setParameter("frames", gyroFrameList);
    gyroGroup->setParameter("ukf_names", gyroUkfNameList);
    REQUIRE(modelParamHandler->setGroup("GYROSCOPE", gyroGroup));
    auto accGroup = std::make_shared<StdImplementation>();
    std::vector<std::string> accNameList = {"root_link"};
    std::vector<std::string> accFrameList = {"root_link"};
    std::vector<std::string> accUkfNameList = {"root_link_acc"};
    accGroup->setParameter("names", accNameList);
    accGroup->setParameter("frames", accFrameList);
    accGroup->setParameter("ukf_names", accUkfNameList);
    REQUIRE(modelParamHandler->setGroup("ACCELEROMETER", accGroup));
    REQUIRE(modelParamHandler->setGroup("EXTERNAL_CONTACT", emptyGroupNamesFrames));

    modelParamHandler->setParameter("joint_list", jointList);

    return modelParamHandler;
}

TEST_CASE("KinDynWrapper Test")
{
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

    // Test kindynwrapper
    Eigen::Vector3d gravity;
    gravity.setZero();
    gravity(2) = -BipedalLocomotion::Math::StandardAccelerationOfGravitation;

    int numJoints = 6;

    Eigen::VectorXd jointPos = Eigen::VectorXd::Random(numJoints);
    Eigen::VectorXd jointVel = Eigen::VectorXd::Random(numJoints);
    Eigen::VectorXd jointAcc = Eigen::VectorXd::Random(numJoints);

    manif::SE3d basePose
        = BipedalLocomotion::Conversions::toManifPose(iDynTree::getRandomTransform());

    manif::SE3Tangentd baseVel
        = BipedalLocomotion::Conversions::toManifTwist(iDynTree::getRandomTwist());

    manif::SE3Tangentd baseAcc
        = BipedalLocomotion::Conversions::toManifTwist(iDynTree::getRandomTwist());

    REQUIRE(kinDyn->setRobotState(basePose.transform(),
                                  jointPos,
                                  iDynTree::make_span(baseVel.data(), manif::SE3d::Tangent::DoF),
                                  jointVel,
                                  gravity));

    // Compute joint torques from inverse dynamics on the full model
    iDynTree::LinkNetExternalWrenches extWrench(kinDyn->model());
    extWrench.zero();
    iDynTree::FreeFloatingGeneralizedTorques jointTorques(kinDyn->model());
    kinDyn->inverseDynamics(iDynTree::make_span(baseAcc.data(), manif::SE3d::Tangent::DoF),
                            jointAcc,
                            extWrench,
                            jointTorques);

    Eigen::VectorXd jointTrq = iDynTree::toEigen(jointTorques.jointTorques());

    // The submodel is only one and it is the full model
    // Indeed the model does not have ft sensors in this test
    // Get sub-model base pose
    manif::SE3d worldTBase = BipedalLocomotion::Conversions::toManifPose(
        kinDyn->getWorldTransform(kinDynWrapperList[0]->getFloatingBase()));

    // Set the sub-model state
    kinDynWrapperList[0]->setRobotState(worldTBase.transform(),
                                        jointPos,
                                        iDynTree::make_span(baseVel.data(),
                                                            manif::SE3d::Tangent::DoF),
                                        jointVel,
                                        gravity);

    // Forward dynamics
    Eigen::VectorXd jointAccFD(numJoints);
    REQUIRE(kinDynWrapperList[0]->forwardDynamics(jointTrq,
                                                  Eigen::VectorXd::Zero(numJoints),
                                                  Eigen::VectorXd::Zero(numJoints),
                                                  baseAcc,
                                                  jointAccFD));

    constexpr double tolerance = 1e-2;
    REQUIRE(jointAcc.isApprox(jointAccFD, tolerance));

    // Compute wrench at the base
    Eigen::VectorXd trqExt = Eigen::VectorXd::Zero(6 + numJoints);
    Eigen::VectorXd genTrq = Eigen::VectorXd::Zero(6 + numJoints);
    REQUIRE(kinDynWrapperList[0]->generalizedBiasForces(genTrq));
    Eigen::MatrixXd massMatrix = Eigen::MatrixXd::Zero(6 + numJoints, 6 + numJoints);
    REQUIRE(kinDynWrapperList[0]->getFreeFloatingMassMatrix(massMatrix));

    Eigen::VectorXd tempVec = massMatrix.topLeftCorner(6, 6) * baseAcc.coeffs();
    tempVec += massMatrix.topRightCorner(6, numJoints) * jointAcc;
    trqExt.head(6) = tempVec + genTrq.head(6);

    Eigen::VectorXd nuDot(6 + numJoints);
    REQUIRE(kinDynWrapperList[0]->forwardDynamics(jointTrq,
                                                  Eigen::VectorXd::Zero(numJoints),
                                                  trqExt,
                                                  nuDot));

    REQUIRE(nuDot.head(6).isApprox(baseAcc.coeffs(), tolerance));
    REQUIRE(nuDot.tail(numJoints).isApprox(jointAcc, tolerance));

    // Test forward dynamics when written in terms of sensor proper acceleration
    // Set the robot state with gravity equal to zero
    baseAcc.coeffs().head(3) -= worldTBase.rotation().transpose() * gravity;

    gravity.setZero();
    REQUIRE(kinDyn->setRobotState(worldTBase.transform(),
                                  jointPos,
                                  iDynTree::make_span(baseVel.data(), manif::SE3d::Tangent::DoF),
                                  jointVel,
                                  gravity));

    kinDyn->inverseDynamics(iDynTree::make_span(baseAcc.data(), manif::SE3d::Tangent::DoF),
                            jointAcc,
                            extWrench,
                            jointTorques);

    jointTrq = iDynTree::toEigen(jointTorques.jointTorques());

    // Set the sub-model state
    kinDynWrapperList[0]->setRobotState(worldTBase.transform(),
                                        jointPos,
                                        iDynTree::make_span(baseVel.data(),
                                                            manif::SE3d::Tangent::DoF),
                                        jointVel,
                                        gravity);

    // Forward dynamics
    Eigen::VectorXd jointAccFD2(numJoints);
    REQUIRE(kinDynWrapperList[0]->forwardDynamics(jointTrq,
                                                  Eigen::VectorXd::Zero(numJoints),
                                                  Eigen::VectorXd::Zero(numJoints),
                                                  baseAcc,
                                                  jointAccFD2));

    REQUIRE(jointAcc.isApprox(jointAccFD2, tolerance));


    // Test forward dynamics with identity as base pose to verify that nothing depends on the base orientation
    // Set the robot state with gravity equal to zero
    worldTBase.setIdentity();

    REQUIRE(kinDyn->setRobotState(worldTBase.transform(),
                                  jointPos,
                                  iDynTree::make_span(baseVel.data(), manif::SE3d::Tangent::DoF),
                                  jointVel,
                                  gravity));

    kinDyn->inverseDynamics(iDynTree::make_span(baseAcc.data(), manif::SE3d::Tangent::DoF),
                            jointAcc,
                            extWrench,
                            jointTorques);

    jointTrq = iDynTree::toEigen(jointTorques.jointTorques());

    // Set the sub-model state
    kinDynWrapperList[0]->setRobotState(worldTBase.transform(),
                                        jointPos,
                                        iDynTree::make_span(baseVel.data(),
                                                            manif::SE3d::Tangent::DoF),
                                        jointVel,
                                        gravity);

    // Forward dynamics
    REQUIRE(kinDynWrapperList[0]->forwardDynamics(jointTrq,
                                                  Eigen::VectorXd::Zero(numJoints),
                                                  Eigen::VectorXd::Zero(numJoints),
                                                  baseAcc,
                                                  jointAccFD2));

    REQUIRE(jointAcc.isApprox(jointAccFD2, tolerance));
}
