/**
 * @file KinDynWrapperTest.cpp
 * @authors Ines Sorrentino
 * @copyright 2023 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the BSD-3-Clause license.
 */

#include <catch2/catch.hpp>

#include <ConfigFolderPath.h>
#include <iCubModels/iCubModels.h>
#include <yarp/os/ResourceFinder.h>

// BLF
#include <BipedalLocomotion/Conversions/ManifConversions.h>
#include <BipedalLocomotion/ParametersHandler/IParametersHandler.h>
#include <BipedalLocomotion/ParametersHandler/YarpImplementation.h>

// iDynTree
#include <iDynTree/KinDynComputations.h>
#include <iDynTree/ModelIO/ModelLoader.h>

#include <BipedalLocomotion/RobotDynamicsEstimator/SubModel.h>
#include <BipedalLocomotion/RobotDynamicsEstimator/SubModelKinDynWrapper.h>

using namespace BipedalLocomotion::ParametersHandler;
namespace RDE = BipedalLocomotion::Estimators::RobotDynamicsEstimator;

double random_double()
{
    return 1.0 * ((double)rand() - RAND_MAX / 2) / ((double)RAND_MAX);
}

double real_random_double()
{
    return 1.0 * ((double)rand() - RAND_MAX / 2) / ((double)RAND_MAX);
}

int real_random_int(int initialValue, int finalValue)
{
    int length = finalValue - initialValue;
    return initialValue + rand() % length;
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
    gravity(2) = -9.80665;

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

TEST_CASE("SubModelKinDynWrapper")
{
    std::shared_ptr<YarpImplementation> originalHandler = std::make_shared<YarpImplementation>();
    IParametersHandler::shared_ptr parameterHandler = originalHandler;

    yarp::os::ResourceFinder& rf = yarp::os::ResourceFinder::getResourceFinderSingleton();
    rf.setDefaultConfigFile("model.ini");

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

    const std::string modelPath = iCubModels::getModelFile("iCubGenova09");

    std::vector<std::string> jointList;
    REQUIRE(parameterHandler->getParameter("joint_list", jointList));

    std::vector<std::string> ftFramesList;
    auto ftGroup = parameterHandler->getGroup("FT").lock();
    REQUIRE(ftGroup->getParameter("frames", ftFramesList));

    std::vector<std::string> jointsAndFTs;
    jointsAndFTs.insert(jointsAndFTs.begin(), jointList.begin(), jointList.end());
    jointsAndFTs.insert(jointsAndFTs.end(), ftFramesList.begin(), ftFramesList.end());

    iDynTree::ModelLoader mdlLdr;
    REQUIRE(mdlLdr.loadReducedModelFromFile(modelPath, jointsAndFTs));

    auto kinDyn = std::make_shared<iDynTree::KinDynComputations>();
    REQUIRE(kinDyn->loadRobotModel(mdlLdr.model()));

    // List of joints and fts to load the model
    RDE::SubModelCreator subModelCreator;
    subModelCreator.setModelAndSensors(mdlLdr.model(), mdlLdr.sensors());
    REQUIRE(subModelCreator.setKinDyn(kinDyn));

    REQUIRE(subModelCreator.createSubModels(originalHandler));

    auto subModelList = subModelCreator.getSubModelList();

    // Full model base acceleration
    manif::SE3d::Tangent robotBaseAcceleration;
    robotBaseAcceleration.setZero();

    // Full model joint acceleration
    Eigen::VectorXd robotJointAcceleration(kinDyn->model().getNrOfDOFs());
    robotJointAcceleration.setZero();

    REQUIRE(setStaticState(kinDyn));

    for (int idx = 0; idx < subModelList.size(); idx++)
    {
        RDE::SubModelKinDynWrapper kinDynSubModel;
        REQUIRE(kinDynSubModel.initialize(subModelList[idx], kinDyn));

        REQUIRE(kinDynSubModel.updateKinDynState(kinDyn));

        int numberOfJoints = subModelList[idx].getModel().getNrOfDOFs();

        if (numberOfJoints > 0)
        {
            Eigen::VectorXd motorTorques(numberOfJoints);
            Eigen::VectorXd frictionTorques(numberOfJoints);
            Eigen::VectorXd contactTorques(numberOfJoints);

            const std::string baseFrame = kinDynSubModel.getBaseFrameName();

            manif::SE3d::Tangent baseAcceleration;
            REQUIRE(kinDynSubModel.getBaseAcceleration(kinDyn,
                                                       robotBaseAcceleration,
                                                       robotJointAcceleration,
                                                       baseAcceleration));
            manif::SE3d::Tangent subModelBaseAcceleration;
            REQUIRE(kinDyn->getFrameAcc(baseFrame,
                                        iDynTree::make_span(robotBaseAcceleration.data(),
                                                            manif::SE3d::Tangent::DoF),
                                        robotJointAcceleration,
                                        iDynTree::make_span(subModelBaseAcceleration.data(),
                                                            manif::SE3d::Tangent::DoF)));
            REQUIRE(subModelBaseAcceleration == baseAcceleration);

            manif::SE3d::Tangent baseVelocity;
            baseVelocity = kinDynSubModel.getBaseVelocity(kinDyn);

            manif::SE3d::Tangent baseVelFromFullModel
                = BipedalLocomotion::Conversions::toManifTwist(kinDyn->getFrameVel(baseFrame));
            REQUIRE(baseVelocity == baseVelFromFullModel);

            Eigen::VectorXd jointAcceleration(numberOfJoints);

            for (int i = 0; i < numberOfJoints; i++)
            {
                motorTorques(i) = random_double();
                frictionTorques(i) = random_double();
                contactTorques(i) = random_double();
            }

            REQUIRE(kinDynSubModel.inverseDynamics(motorTorques,
                                                   frictionTorques,
                                                   contactTorques,
                                                   baseAcceleration.coeffs(),
                                                   jointAcceleration));

            auto massMatrix = kinDynSubModel.getMassMatrix();
            REQUIRE(massMatrix.rows() == (6 + numberOfJoints));

            auto genForces = kinDynSubModel.getGeneralizedForces();
            REQUIRE(genForces.size() == (6 + numberOfJoints));
        }

        if (subModelList[idx].getFTList().size() > 0)
        {
            const std::string ftname = subModelList[idx].getFTList()[0].name;
            auto jacobianFT = kinDynSubModel.getFTJacobian(ftname);
            REQUIRE(jacobianFT.rows() == 6);
            REQUIRE(jacobianFT.cols() == 6 + numberOfJoints);
        }

        if (subModelList[idx].getAccelerometerList().size())
        {
            const std::string accName = subModelList[idx].getAccelerometerList()[0].name;
            auto jacobianAcc = kinDynSubModel.getAccelerometerJacobian(accName);
            REQUIRE(jacobianAcc.rows() == 6);
            REQUIRE(jacobianAcc.cols() == 6 + numberOfJoints);
        }
    }
}
