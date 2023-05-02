/**
 * @file KinDynWrapperTest.cpp
 * @authors Ines Sorrentino
 * @copyright 2023 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the BSD-3-Clause license.
 */

#include <catch2/catch_test_macros.hpp>

#include <ConfigFolderPath.h>
#include <iCubModels/iCubModels.h>
#include <yarp/os/ResourceFinder.h>
#include <map>

// BLF
#include <BipedalLocomotion/Conversions/ManifConversions.h>
#include <BipedalLocomotion/ParametersHandler/IParametersHandler.h>
#include <BipedalLocomotion/ParametersHandler/YarpImplementation.h>
#include <BipedalLocomotion/Math/Constants.h>
#include <BipedalLocomotion/Math/Wrench.h>

// iDynTree
#include <iDynTree/KinDynComputations.h>
#include <iDynTree/ModelIO/ModelLoader.h>
#include <iDynTree/Estimation/ExtWrenchesAndJointTorquesEstimator.h>

#include <BipedalLocomotion/RobotDynamicsEstimator/SubModel.h>
#include <BipedalLocomotion/RobotDynamicsEstimator/SubModelKinDynWrapper.h>

using namespace BipedalLocomotion;
using namespace BipedalLocomotion::ParametersHandler;
namespace RDE = BipedalLocomotion::Estimators::RobotDynamicsEstimator;

namespace Eigen
{
    using Vector6d = Eigen::Matrix<double, 6, 1>;
}

bool setStaticStateKinDyn(std::shared_ptr<iDynTree::KinDynComputations> kinDyn)
{
    size_t dofs = kinDyn->getNrOfDegreesOfFreedom();

    Eigen::VectorXd baseVel(6);
    Eigen::Vector3d gravity;

    Eigen::VectorXd qj(dofs), dqj(dofs);

    Eigen::Matrix4d transform;
    transform.setIdentity();

    baseVel.setZero();

    gravity.setZero();
    gravity(2) = - BipedalLocomotion::Math::StandardAccelerationOfGravitation;

    for (size_t dof = 0; dof < dofs; dof++)

    {
        qj(dof) = 0.0;
        dqj(dof) = 0.0;
    }

    return kinDyn->setRobotState(iDynTree::make_span(qj.data(), qj.size()),
                                iDynTree::make_span(dqj.data(), dqj.size()),
                                iDynTree::make_span(gravity.data(), gravity.size()));
}

std::map<std::string, Eigen::VectorXd> getFTMeasurementFromStaticState(std::shared_ptr<iDynTree::ExtWrenchesAndJointTorquesEstimator> estimator)
{
    std::map<std::string, Eigen::VectorXd> ftMeasurement;

    iDynTree::JointPosDoubleArray qj(estimator->model());
    iDynTree::JointDOFsDoubleArray dqj(estimator->model()), ddqj(estimator->model());

    qj.zero();
    dqj.zero();
    ddqj.zero();

    int rootLinkIndex = estimator->model().getFrameIndex("root_link");

    iDynTree::Vector3 gravityOnRootLink;
    gravityOnRootLink.zero();
    gravityOnRootLink(2) = - BipedalLocomotion::Math::StandardAccelerationOfGravitation;

    // The estimated FT sensor measurements
    iDynTree::SensorsMeasurements expFTmeasurements(estimator->sensors());

    // The estimated external wrenches
    iDynTree::LinkContactWrenches estContactWrenches(estimator->model());

    // The estimated joint torques
    iDynTree::JointDOFsDoubleArray estJointTorques(estimator->model());

    iDynTree::UnknownWrenchContact unknownWrench;
    unknownWrench.unknownType = iDynTree::FULL_WRENCH;
    unknownWrench.contactPoint.zero();

    iDynTree::LinkUnknownWrenchContacts fullBodyUnknowns(estimator->model());
    fullBodyUnknowns.clear();

    REQUIRE(fullBodyUnknowns.addNewContactInFrame(estimator->model(), rootLinkIndex, unknownWrench));

    REQUIRE(estimator->updateKinematicsFromFixedBase(qj, dqj, ddqj, rootLinkIndex, gravityOnRootLink));

    REQUIRE(estimator->computeExpectedFTSensorsMeasurements(fullBodyUnknowns, expFTmeasurements, estContactWrenches, estJointTorques));

    iDynTree::Wrench estimatedSensorWrench;
    estimatedSensorWrench.zero();

    for (int index = 0; index < estimator->sensors().getNrOfSensors(iDynTree::SIX_AXIS_FORCE_TORQUE); index++)
    {
        REQUIRE(expFTmeasurements.getMeasurement(iDynTree::SIX_AXIS_FORCE_TORQUE, index, estimatedSensorWrench));
        ftMeasurement[estimator->sensors().getSensor(iDynTree::SIX_AXIS_FORCE_TORQUE, index)->getName()] = iDynTree::toEigen(estimatedSensorWrench);
    }

    return ftMeasurement;
}

std::shared_ptr<iDynTree::ExtWrenchesAndJointTorquesEstimator> createFTAndJointTorquesEstimator(iDynTree::ModelLoader& modelLoader)
{
    std::shared_ptr<iDynTree::ExtWrenchesAndJointTorquesEstimator> estimator = std::make_shared<iDynTree::ExtWrenchesAndJointTorquesEstimator>();

    REQUIRE(estimator->setModelAndSensors(modelLoader.model(), modelLoader.sensors()));

    return estimator;
}

TEST_CASE("SubModelKinDynWrapper")
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

    auto group = parameterHandler->getGroup("MODEL").lock();
    REQUIRE(group != nullptr);

    const std::string modelPath = iCubModels::getModelFile("iCubGenova09");

    std::vector<std::string> jointList;
    REQUIRE(group->getParameter("joint_list", jointList));

    std::vector<std::string> ftFramesList;
    auto ftGroup = group->getGroup("FT").lock();
    REQUIRE(ftGroup->getParameter("associated_joints", ftFramesList));

    std::vector<std::string> jointsAndFTs;
    jointsAndFTs.insert(jointsAndFTs.begin(), jointList.begin(), jointList.end());
    jointsAndFTs.insert(jointsAndFTs.end(), ftFramesList.begin(), ftFramesList.end());

    iDynTree::ModelLoader mdlLdr;
    REQUIRE(mdlLdr.loadReducedModelFromFile(modelPath, jointsAndFTs));

    auto kinDyn = std::make_shared<iDynTree::KinDynComputations>();
    REQUIRE(kinDyn->loadRobotModel(mdlLdr.model()));

    REQUIRE(kinDyn->setFrameVelocityRepresentation(iDynTree::BODY_FIXED_REPRESENTATION));

    // List of joints and fts to load the model
    RDE::SubModelCreator subModelCreator;
    subModelCreator.setModelAndSensors(mdlLdr.model(), mdlLdr.sensors());
    REQUIRE(subModelCreator.setKinDyn(kinDyn));

    REQUIRE(subModelCreator.createSubModels(group));

    auto subModelList = subModelCreator.getSubModelList();

    auto totalMass = 0.0;
    for (int idx = 0; idx < subModelList.size(); idx++)
    {
        totalMass += subModelList[idx].getModel().getTotalMass();
    }
    REQUIRE(totalMass == kinDyn->model().getTotalMass());

    // Full model base acceleration
    manif::SE3d::Tangent robotBaseAcceleration;
    robotBaseAcceleration.setZero();

    // Full model joint acceleration
    Eigen::VectorXd robotJointAcceleration(kinDyn->model().getNrOfDOFs());
    robotJointAcceleration.setZero();

    REQUIRE(setStaticStateKinDyn(kinDyn));

    std::shared_ptr<iDynTree::ExtWrenchesAndJointTorquesEstimator> estimator = createFTAndJointTorquesEstimator(mdlLdr);

    std::map<std::string, Eigen::VectorXd> ftMeasurement = getFTMeasurementFromStaticState(estimator);

    // Compute joint torques in static configuration from inverse dynamics on the full model
    iDynTree::FreeFloatingGeneralizedTorques jointTorques(kinDyn->model());
    iDynTree::LinkNetExternalWrenches extWrench(kinDyn->model());
    extWrench.zero();

    kinDyn->inverseDynamics(iDynTree::make_span(robotBaseAcceleration.data(),
                                                manif::SE3d::Tangent::DoF),
                            robotJointAcceleration,
                            extWrench,
                            jointTorques);

    for (int idx = 0; idx < subModelList.size(); idx++)
    {
        RDE::SubModelKinDynWrapper kinDynSubModel;

        REQUIRE(kinDynSubModel.setKinDyn(kinDyn));

        REQUIRE(kinDynSubModel.initialize(subModelList[idx]));

        REQUIRE(kinDynSubModel.updateState(robotBaseAcceleration,
                                           robotJointAcceleration,
                                           BipedalLocomotion::Estimators::RobotDynamicsEstimator::UpdateMode::Full));

        int numberOfJoints = subModelList[idx].getModel().getNrOfDOFs();

        if (numberOfJoints > 0)
        {
            Eigen::VectorXd motorTorques(numberOfJoints);
            Eigen::VectorXd frictionTorques(numberOfJoints);
            Eigen::VectorXd contactTorques(numberOfJoints);

            motorTorques.setZero();
            frictionTorques.setZero();
            contactTorques.setZero();

            const std::string baseFrame = kinDynSubModel.getBaseFrameName();

            manif::SE3d::Tangent baseAcceleration;
            baseAcceleration = kinDynSubModel.getBaseAcceleration();

            manif::SE3d::Tangent baseAccelerationFromFullModel;
            REQUIRE(kinDyn->getFrameAcc(baseFrame,
                                        iDynTree::make_span(robotBaseAcceleration.data(),
                                                            manif::SE3d::Tangent::DoF),
                                        robotJointAcceleration,
                                        iDynTree::make_span(baseAccelerationFromFullModel.data(),
                                                            manif::SE3d::Tangent::DoF)));
            REQUIRE(baseAccelerationFromFullModel == baseAcceleration);

            manif::SE3d::Tangent baseVelocity;
            baseVelocity = kinDynSubModel.getBaseVelocity();

            manif::SE3d::Tangent baseVelFromFullModel
                = BipedalLocomotion::Conversions::toManifTwist(kinDyn->getFrameVel(baseFrame));
            REQUIRE(baseVelocity == baseVelFromFullModel);

            for (auto const& [key, val] : ftMeasurement)
            {
                if (subModelList[idx].getModel().isFrameNameUsed(key))
                {
                    Eigen::MatrixXd jacobian = kinDynSubModel.getFTJacobian(key);

                    Eigen::MatrixXd jac = jacobian.block(0, 6, 6, subModelList[idx].getModel().getNrOfDOFs());

                    for (auto & [ftName, ftObj] : subModelList[idx].getFTList())
                    {
                        if (ftName == key)
                        {
                            contactTorques.noalias() += static_cast<int>(ftObj.forceDirection) * jac.transpose() * val;
                        }
                    }
                }
            }

            Eigen::VectorXd jointAcceleration(numberOfJoints);

            for (int jointIndex = 0; jointIndex < subModelList[idx].getJointMapping().size(); jointIndex++)
            {
                motorTorques[jointIndex] = jointTorques.jointTorques()[subModelList[idx].getJointMapping()[jointIndex]];
            }

            REQUIRE(kinDynSubModel.forwardDynamics(motorTorques,
                                                   frictionTorques,
                                                   contactTorques,
                                                   baseAcceleration.coeffs(),
                                                   jointAcceleration));

            Eigen::VectorXd zeroVector(numberOfJoints);
            zeroVector.setZero();

            for (int index = 0; index < jointAcceleration.size(); index++)
            {
                REQUIRE(std::abs(jointAcceleration[index]) < 0.1);
            }

            auto massMatrix = kinDynSubModel.getMassMatrix();
            REQUIRE(massMatrix.rows() == (6 + numberOfJoints));

            auto genForces = kinDynSubModel.getGeneralizedForces();
            REQUIRE(genForces.size() == (6 + numberOfJoints));
        }

        if (subModelList[idx].getFTList().size() > 0)
        {
            const std::string ftname = subModelList[idx].getFTList().begin()->first;
            auto jacobianFT = kinDynSubModel.getFTJacobian(ftname);
            REQUIRE(jacobianFT.rows() == 6);
            REQUIRE(jacobianFT.cols() == 6 + numberOfJoints);
        }

        if (subModelList[idx].getAccelerometerList().size())
        {
            const std::string accName = subModelList[idx].getAccelerometerList().begin()->first;
            auto jacobianAcc = kinDynSubModel.getAccelerometerJacobian(accName);
            REQUIRE(jacobianAcc.rows() == 6);
            REQUIRE(jacobianAcc.cols() == 6 + numberOfJoints);
        }
    }
}
