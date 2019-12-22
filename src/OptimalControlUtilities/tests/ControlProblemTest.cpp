/**
 * @file ControlProblemTest.cpp
 * @authors Giulio Romualdi
 * @copyright 2019 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the GNU Lesser General Public License v2.1 or any later version.
 */

// Catch2
#include <catch2/catch.hpp>

#include <iDynTree/Core/EigenHelpers.h>

#include <iDynTree/Core/EigenHelpers.h>
#include <iDynTree/KinDynComputations.h>
#include <iDynTree/Model/ModelTestUtils.h>

#include <BipedalLocomotionControllers/OptimalControlUtilities/CartesianElements.h>
#include <BipedalLocomotionControllers/OptimalControlUtilities/CentroidalMomentumElements.h>
#include <BipedalLocomotionControllers/OptimalControlUtilities/ControlProblemElements.h>
#include <BipedalLocomotionControllers/OptimalControlUtilities/FeasibilityElements.h>
#include <BipedalLocomotionControllers/OptimalControlUtilities/FloatingBaseMultiBodyDynamicsElements.h>
#include <BipedalLocomotionControllers/OptimalControlUtilities/PDController.h>
#include <BipedalLocomotionControllers/OptimalControlUtilities/RegularizationElements.h>
#include <BipedalLocomotionControllers/OptimalControlUtilities/VariableHandler.h>
#include <BipedalLocomotionControllers/OptimalControlUtilities/ZMPElements.h>

using namespace BipedalLocomotionControllers::OptimalControlUtilities;

TEST_CASE("Check Cartesian element of the ControlProblemElementsTest",
          "[ControlProblemElementsTest-CartesianElement]")
{
    unsigned int numberOfJoints = 30;

    iDynTree::Model model = iDynTree::getRandomModel(numberOfJoints, 10);
    unsigned int numberDoFs = model.getNrOfDOFs();

    std::shared_ptr<iDynTree::KinDynComputations> kinDyn;
    kinDyn = std::make_shared<iDynTree::KinDynComputations>();

    kinDyn->loadRobotModel(model);
    kinDyn->setFrameVelocityRepresentation(iDynTree::MIXED_REPRESENTATION);
    REQUIRE(kinDyn->setFloatingBase("baseLink"));

    iDynTree::Vector3 gravity;
    gravity.zero();
    gravity(2) = -9.81;
    iDynTree::VectorDynSize s(numberDoFs);
    iDynTree::toEigen(s).setRandom();

    iDynTree::VectorDynSize ds(numberDoFs);
    iDynTree::toEigen(ds).setRandom();

    REQUIRE(kinDyn->setRobotState(iDynTree::Transform::Identity(),
                                  s,
                                  iDynTree::Twist::Zero(),
                                  ds,
                                  gravity));

    VariableHandler handler;

    int baseAccelerationOffset = 0;
    handler.addVariable("base_acceleration", 6);
    handler.addVariable("joint_accelerations", numberDoFs);

    iDynTree::MatrixDynSize jacobian(6, numberDoFs + 6);

    std::string frame = "link2";

    SECTION("Test Position Element")
    {
        CartesianElement element(kinDyn, handler, CartesianElement::Type::POSITION, frame);
        iDynTree::Vector3 gains;
        gains(0) = 1;
        gains(1) = 1;
        gains(2) = 1;
        element.setLinearPDGains(gains, gains);

        iDynTree::Vector3 dummy;
        dummy.zero();
        element.setDesiredTrajectory(dummy, dummy, dummy);

        REQUIRE(kinDyn->getFrameFreeFloatingJacobian(frame, jacobian));
        REQUIRE(
            iDynTree::toEigen(element.getA()).block(0, baseAccelerationOffset, 3, numberDoFs + 6)
            == iDynTree::toEigen(jacobian).block(0, 0, 3, numberDoFs + 6));

        LinearPD<iDynTree::Vector3> pd;
        pd.setGains(gains, gains);
        pd.setDesiredTrajectory(dummy, dummy, dummy);
        pd.setFeedback(kinDyn->getFrameVel(frame).getLinearVec3(),
                        kinDyn->getWorldTransform(frame).getPosition());

        REQUIRE(iDynTree::toEigen(element.getB())
                == -iDynTree::toEigen(kinDyn->getFrameBiasAcc(frame)).head(3)
                       + iDynTree::toEigen(pd.getControllerOutput()));
    }

    SECTION("Test Orientation Element")
    {
        CartesianElement element(kinDyn, handler, CartesianElement::Type::ORIENTATION, frame);
        element.setOrientationPDGains(1, 1, 1);

        iDynTree::Vector3 dummy;
        dummy.zero();
        element.setDesiredTrajectory(dummy, dummy, iDynTree::Rotation::Identity());

        REQUIRE_THROWS(element.setDesiredTrajectory(dummy, dummy, dummy));

        REQUIRE(kinDyn->getFrameFreeFloatingJacobian(frame, jacobian));
        REQUIRE(
            iDynTree::toEigen(element.getA()).block(0, baseAccelerationOffset, 3, numberDoFs + 6)
            == iDynTree::toEigen(jacobian).block(3, 0, 3, numberDoFs + 6));

        OrientationPD pd;
        pd.setGains(1, 1, 1);
        pd.setDesiredTrajectory(dummy, dummy, iDynTree::Rotation::Identity());
        pd.setFeedback(kinDyn->getFrameVel(frame).getAngularVec3(),
                        kinDyn->getWorldTransform(frame).getRotation());

        REQUIRE(iDynTree::toEigen(element.getB())
                == -iDynTree::toEigen(kinDyn->getFrameBiasAcc(frame)).tail(3)
                       + iDynTree::toEigen(pd.getControllerOutput()));
    }

    SECTION("Test Pose Element")
    {
        CartesianElement element(kinDyn, handler, CartesianElement::Type::POSE, frame);
        iDynTree::Vector3 gains;
        gains(0) = 1;
        gains(1) = 1;
        gains(2) = 1;
        element.setLinearPDGains(gains, gains);
        element.setOrientationPDGains(1, 1, 1);

        iDynTree::Vector3 dummy;
        dummy.zero();
        element.setDesiredTrajectory(dummy, dummy, dummy);
        element.setDesiredTrajectory(dummy, dummy, iDynTree::Rotation::Identity());

        REQUIRE(kinDyn->getFrameFreeFloatingJacobian(frame, jacobian));
        REQUIRE(
            iDynTree::toEigen(element.getA()).block(0, baseAccelerationOffset, 6, numberDoFs + 6)
            == iDynTree::toEigen(jacobian));

        // Linear part
        LinearPD<iDynTree::Vector3> positionPd;
        positionPd.setGains(gains, gains);
        positionPd.setDesiredTrajectory(dummy, dummy, dummy);
        positionPd.setFeedback(kinDyn->getFrameVel(frame).getLinearVec3(),
                                kinDyn->getWorldTransform(frame).getPosition());
        REQUIRE(iDynTree::toEigen(element.getB()).head(3)
                == -iDynTree::toEigen(kinDyn->getFrameBiasAcc(frame)).head(3)
                       + iDynTree::toEigen(positionPd.getControllerOutput()));

        OrientationPD orientationPd;
        orientationPd.setGains(1, 1, 1);
        orientationPd.setDesiredTrajectory(dummy, dummy, iDynTree::Rotation::Identity());
        orientationPd.setFeedback(kinDyn->getFrameVel(frame).getAngularVec3(),
                                   kinDyn->getWorldTransform(frame).getRotation());
        REQUIRE(iDynTree::toEigen(element.getB()).tail(3)
                == -iDynTree::toEigen(kinDyn->getFrameBiasAcc(frame)).tail(3)
                       + iDynTree::toEigen(orientationPd.getControllerOutput()));
    }

    SECTION("One Degree of Freedom - X")
    {
        CartesianElement element(kinDyn,
                                 handler,
                                 CartesianElement::Type::ONE_DIMENSION,
                                 frame,
                                 CartesianElement::AxisName::X);
        double gain = 1;
        element.setOneDegreePDGains(gain, gain);

        double dummy = 0;
        element.setDesiredTrajectory(dummy, dummy, dummy);

        REQUIRE(kinDyn->getFrameFreeFloatingJacobian(frame, jacobian));
        REQUIRE(
            iDynTree::toEigen(element.getA()).block(0, baseAccelerationOffset, 1, numberDoFs + 6)
            == iDynTree::toEigen(jacobian).block(0, 0, 1, numberDoFs + 6));

        LinearPD<double> pd;
        pd.setGains(gain, gain);
        pd.setDesiredTrajectory(dummy, dummy, dummy);
        pd.setFeedback(kinDyn->getFrameVel(frame).getLinearVec3()(0),
                        kinDyn->getWorldTransform(frame).getPosition()(0));
        REQUIRE(element.getB()(0)
                == -kinDyn->getFrameBiasAcc(frame)(0) + pd.getControllerOutput());
    }

    SECTION("One Degree of Freedom - Y")
    {
        CartesianElement element(kinDyn,
                                 handler,
                                 CartesianElement::Type::ONE_DIMENSION,
                                 frame,
                                 CartesianElement::AxisName::Y);
        double gain = 1;
        element.setOneDegreePDGains(gain, gain);

        double dummy = 0;
        element.setDesiredTrajectory(dummy, dummy, dummy);

        REQUIRE(kinDyn->getFrameFreeFloatingJacobian(frame, jacobian));
        REQUIRE(
            iDynTree::toEigen(element.getA()).block(0, baseAccelerationOffset, 1, numberDoFs + 6)
            == iDynTree::toEigen(jacobian).block(1, 0, 1, numberDoFs + 6));

        LinearPD<double> pd;
        pd.setGains(gain, gain);
        pd.setDesiredTrajectory(dummy, dummy, dummy);
        pd.setFeedback(kinDyn->getFrameVel(frame).getLinearVec3()(1),
                        kinDyn->getWorldTransform(frame).getPosition()(1));
        REQUIRE(element.getB()(0)
                == -kinDyn->getFrameBiasAcc(frame)(1) + pd.getControllerOutput());
    }

    SECTION("One Degree of Freedom - Z")
    {
        CartesianElement element(kinDyn,
                                 handler,
                                 CartesianElement::Type::ONE_DIMENSION,
                                 frame,
                                 CartesianElement::AxisName::Z);
        double gain = 1;
        element.setOneDegreePDGains(gain, gain);

        double dummy = 0;
        element.setDesiredTrajectory(dummy, dummy, dummy);

        REQUIRE(kinDyn->getFrameFreeFloatingJacobian(frame, jacobian));
        REQUIRE(
            iDynTree::toEigen(element.getA()).block(0, baseAccelerationOffset, 1, numberDoFs + 6)
            == iDynTree::toEigen(jacobian).block(2, 0, 1, numberDoFs + 6));

        LinearPD<double> pd;
        pd.setGains(gain, gain);
        pd.setDesiredTrajectory(dummy, dummy, dummy);
        pd.setFeedback(kinDyn->getFrameVel(frame).getLinearVec3()(2),
                        kinDyn->getWorldTransform(frame).getPosition()(2));
        REQUIRE(element.getB()(0)
                == -kinDyn->getFrameBiasAcc(frame)(2) + pd.getControllerOutput());
    }
}

TEST_CASE("Check System Dynamics element of the ControlProblemElementsTest",
          "[ControlProblemElementsTest-SystemDynamicsElement]")
{

    unsigned int numberOfJoints = 30;

    iDynTree::Model model = iDynTree::getRandomModel(numberOfJoints, 10);
    unsigned int numberDoFs = model.getNrOfDOFs();

    std::shared_ptr<iDynTree::KinDynComputations> kinDyn;
    kinDyn = std::make_shared<iDynTree::KinDynComputations>();

    kinDyn->loadRobotModel(model);
    kinDyn->setFrameVelocityRepresentation(iDynTree::MIXED_REPRESENTATION);
    REQUIRE(kinDyn->setFloatingBase("baseLink"));

    iDynTree::Vector3 gravity;
    gravity.zero();
    gravity(2) = -9.81;
    iDynTree::VectorDynSize s(numberDoFs);
    iDynTree::toEigen(s).setRandom();

    iDynTree::VectorDynSize ds(numberDoFs);
    iDynTree::toEigen(ds).setRandom();

    REQUIRE(kinDyn->setRobotState(iDynTree::Transform::Identity(),
                                  s,
                                  iDynTree::Twist::Zero(),
                                  ds,
                                  gravity));


    std::string linkInContact1 = "link5";
    std::string linkInContact2 = "link9";


    VariableHandler handler;

    handler.addVariable("base_acceleration", 6);
    handler.addVariable("joint_accelerations", numberDoFs);
    handler.addVariable("joint_torques", numberDoFs);
    handler.addVariable("link_in_contact_1", 6);
    handler.addVariable("link_in_contact_2", 6);

    // Instantiate the element
    FloatingBaseMultiBodyDynamicsElement element(kinDyn,
                                                 handler,
                                                 {{"link_in_contact_1", linkInContact1},
                                                  {"link_in_contact_2", linkInContact2}});

    // check the matrix A
    iDynTree::MatrixDynSize massMatrix(numberDoFs + 6, numberDoFs + 6);
    REQUIRE(kinDyn->getFreeFloatingMassMatrix(massMatrix));
    REQUIRE(iDynTree::toEigen(element.getA()).block(0, 0, numberDoFs + 6, numberDoFs + 6)
            == -iDynTree::toEigen(massMatrix));

    iDynTree::MatrixDynSize identity(numberDoFs, numberDoFs);
    iDynTree::toEigen(identity).setIdentity();
    REQUIRE(iDynTree::toEigen(element.getA()).block(6, numberDoFs + 6, numberDoFs, numberDoFs)
            == iDynTree::toEigen(identity));
    iDynTree::MatrixDynSize jacobian(6, numberDoFs + 6);
    REQUIRE(kinDyn->getFrameFreeFloatingJacobian(linkInContact1, jacobian));
    REQUIRE(iDynTree::toEigen(element.getA()).block(0, 2 * numberDoFs + 6, 6 + numberDoFs, 6)
            == iDynTree::toEigen(jacobian).transpose());

    REQUIRE(kinDyn->getFrameFreeFloatingJacobian(linkInContact2, jacobian));
    REQUIRE(iDynTree::toEigen(element.getA()).block(0, 2 * numberDoFs + 6 + 6,
                                                    6 + numberDoFs, 6) == iDynTree::toEigen(jacobian).transpose());

    // check the vector b
    iDynTree::FreeFloatingGeneralizedTorques generalizedBiasForces;
    generalizedBiasForces.resize(kinDyn->model());
    kinDyn->generalizedBiasForces(generalizedBiasForces);

    REQUIRE(iDynTree::toEigen(element.getB()).head(6)
            == iDynTree::toEigen(generalizedBiasForces.baseWrench()));
    REQUIRE(iDynTree::toEigen(element.getB()).tail(numberDoFs)
            == iDynTree::toEigen(generalizedBiasForces.jointTorques()));
}

TEST_CASE("Check CentroidalMomentum element of the ControlProblemElementsTest",
          "[ControlProblemElementsTest-CentroidalMomentumElement]")
{
    unsigned int numberOfJoints = 30;

    iDynTree::Model model = iDynTree::getRandomModel(numberOfJoints, 10);
    unsigned int numberDoFs = model.getNrOfDOFs();

    std::shared_ptr<iDynTree::KinDynComputations> kinDyn;
    kinDyn = std::make_shared<iDynTree::KinDynComputations>();

    kinDyn->loadRobotModel(model);
    kinDyn->setFrameVelocityRepresentation(iDynTree::MIXED_REPRESENTATION);
    REQUIRE(kinDyn->setFloatingBase("baseLink"));

    iDynTree::Vector3 gravity;
    gravity.zero();
    gravity(2) = -9.81;
    iDynTree::VectorDynSize s(numberDoFs);
    iDynTree::toEigen(s).setRandom();

    iDynTree::VectorDynSize ds(numberDoFs);
    iDynTree::toEigen(ds).setRandom();

    REQUIRE(kinDyn->setRobotState(iDynTree::Transform::Identity(),
                                  s,
                                  iDynTree::Twist::Zero(),
                                  ds,
                                  gravity));


    std::string linkInContact1 = "link5";
    std::string linkInContact2 = "link9";

    VariableHandler handler;

    handler.addVariable("base_acceleration", 6);
    handler.addVariable("joint_accelerations", numberDoFs);
    handler.addVariable("joint_torques", numberDoFs);
    handler.addVariable("link_in_contact_1", 6);
    handler.addVariable("link_in_contact_2", 6);

    iDynTree::MatrixDynSize identity(3, 3);
    iDynTree::toEigen(identity).setIdentity();

    iDynTree::Vector3 gain;
    for(unsigned int i = 0; i < 3; i++)
        gain(i) = 1;
    iDynTree::Vector3 dummy;
    iDynTree::toEigen(dummy).setRandom();

    SECTION("Linear Momentum")
    {
        CentroidalLinearMomentumElement element(kinDyn,
                                                handler,
                                                {"link_in_contact_1", "link_in_contact_2"});

        element.setGain(gain);
        element.setDesiredCentroidalLinearMomentum(dummy, dummy);

        // Check A
        REQUIRE(iDynTree::toEigen(element.getA()).block(0, 2 * numberDoFs + 6, 3, 3)
                == iDynTree::toEigen(identity));
        REQUIRE(iDynTree::toEigen(element.getA()).block(0, 2 * numberDoFs + 6 + 6, 3, 3)
                == iDynTree::toEigen(identity));

        iDynTree::Vector3 computedB;
        iDynTree::toEigen(computedB) = iDynTree::toEigen(dummy) + iDynTree::toEigen(gain).asDiagonal()
            * (iDynTree::toEigen(dummy)
               - iDynTree::toEigen(kinDyn->getCentroidalTotalMomentum().getLinearVec3()));
        computedB(2) += 9.81 * model.getTotalMass();

        REQUIRE(iDynTree::toEigen(element.getB()) == iDynTree::toEigen(computedB));
    }

    SECTION("Angular Momentum")
    {
        CentroidalAngularMomentumElement element(kinDyn,
                                                 handler,
                                                 {{"link_in_contact_1", linkInContact1},
                                                     {"link_in_contact_2", linkInContact2}});
        element.setGain(gain);
        element.setDesiredCentroidalAngularMomentum(dummy, dummy);

        REQUIRE(iDynTree::toEigen(element.getA()).block(0, 2 * numberDoFs + 6 + 3, 3, 3)
                == iDynTree::toEigen(identity));
        REQUIRE(iDynTree::toEigen(element.getA()).block(0, 2 * numberDoFs + 6 + 6 + 3, 3, 3)
                == iDynTree::toEigen(identity));

        iDynTree::Vector3 com = kinDyn->getCenterOfMassPosition();
        iDynTree::Vector3 linkInContact1Pose;
        linkInContact1Pose = kinDyn->getWorldTransform(linkInContact1).getPosition();
        iDynTree::Vector3 linkInContact2Pose;
        linkInContact2Pose = kinDyn->getWorldTransform(linkInContact2).getPosition();

        REQUIRE(iDynTree::toEigen(element.getA()).block(0, 2 * numberDoFs + 6, 3, 3)
                == iDynTree::skew(iDynTree::toEigen(linkInContact1Pose) - iDynTree::toEigen(com)));
        REQUIRE(iDynTree::toEigen(element.getA()).block(0, 2 * numberDoFs + 6 + 6, 3, 3)
                == iDynTree::skew(iDynTree::toEigen(linkInContact2Pose) - iDynTree::toEigen(com)));

        // check the vector b
        REQUIRE(iDynTree::toEigen(element.getB())
                == iDynTree::toEigen(dummy) + iDynTree::toEigen(gain).asDiagonal()
                * (iDynTree::toEigen(dummy)
                   - iDynTree::toEigen(kinDyn->getCentroidalTotalMomentum().getAngularVec3())));
    }
}

TEST_CASE("Check Regularization element of the ControlProblemElementsTest",
          "[ControlProblemElementsTest-RegularizationElement]")
{
    unsigned int numberOfJoints = 30;

    iDynTree::Model model = iDynTree::getRandomModel(numberOfJoints, 10);
    unsigned int numberDoFs = model.getNrOfDOFs();

    std::shared_ptr<iDynTree::KinDynComputations> kinDyn;
    kinDyn = std::make_shared<iDynTree::KinDynComputations>();

    kinDyn->loadRobotModel(model);
    kinDyn->setFrameVelocityRepresentation(iDynTree::MIXED_REPRESENTATION);
    REQUIRE(kinDyn->setFloatingBase("baseLink"));

    iDynTree::Vector3 gravity;
    gravity.zero();
    gravity(2) = -9.81;
    iDynTree::VectorDynSize s(numberDoFs);
    iDynTree::toEigen(s).setRandom();

    iDynTree::VectorDynSize ds(numberDoFs);
    iDynTree::toEigen(ds).setRandom();

    REQUIRE(kinDyn->setRobotState(iDynTree::Transform::Identity(),
                                  s,
                                  iDynTree::Twist::Zero(),
                                  ds,
                                  gravity));


    std::string linkInContact1 = "link5";
    std::string linkInContact2 = "link9";

    VariableHandler handler;

    handler.addVariable("base_acceleration", 6);
    handler.addVariable("joint_accelerations", numberDoFs);
    handler.addVariable("joint_torques", numberDoFs);
    handler.addVariable("link_in_contact_1", 6);
    handler.addVariable("link_in_contact_2", 6);

    iDynTree::MatrixDynSize identity(numberDoFs, numberDoFs);
    iDynTree::toEigen(identity).setIdentity();

    SECTION("Regularization Element")
    {
        RegularizationElement element(kinDyn, handler, "joint_torques");
        iDynTree::VectorDynSize zero(numberDoFs);
        zero.zero();

        REQUIRE(iDynTree::toEigen(element.getA()).block(0, numberDoFs + 6, numberDoFs, numberDoFs)
                == iDynTree::toEigen(identity));

        REQUIRE(iDynTree::toEigen(element.getB()) == iDynTree::toEigen(zero));
    }

    SECTION("Regularization With Control Element")
    {
        RegularizationWithControlElement element(kinDyn, handler, "joint_accelerations");

        iDynTree::VectorDynSize dummyRef(numberDoFs);
        dummyRef.zero();

        iDynTree::VectorDynSize gains(numberDoFs);
        iDynTree::toEigen(gains).setOnes();

        element.setState(ds, s);
        element.setDesiredTrajectory(dummyRef, dummyRef, dummyRef);
        element.setPDGains(gains, gains);

        REQUIRE(iDynTree::toEigen(element.getA()).block(0, 6, numberDoFs, numberDoFs)
                == iDynTree::toEigen(identity));

        REQUIRE(iDynTree::toEigen(element.getB())
                == iDynTree::toEigen(dummyRef) + iDynTree::toEigen(gains).asDiagonal()
                * (iDynTree::toEigen(dummyRef) - iDynTree::toEigen(ds))
                + iDynTree::toEigen(gains).asDiagonal()
                * (iDynTree::toEigen(dummyRef) - iDynTree::toEigen(s)));
    }
}

TEST_CASE("Check ZMP element of the ControlProblemElementsTest",
          "[ControlProblemElementsTest-ZMPElement]")
{
    unsigned int numberOfJoints = 30;

    iDynTree::Model model = iDynTree::getRandomModel(numberOfJoints, 10);
    unsigned int numberDoFs = model.getNrOfDOFs();

    std::shared_ptr<iDynTree::KinDynComputations> kinDyn;
    kinDyn = std::make_shared<iDynTree::KinDynComputations>();

    kinDyn->loadRobotModel(model);
    kinDyn->setFrameVelocityRepresentation(iDynTree::MIXED_REPRESENTATION);
    REQUIRE(kinDyn->setFloatingBase("baseLink"));

    iDynTree::Vector3 gravity;
    gravity.zero();
    gravity(2) = -9.81;
    iDynTree::VectorDynSize s(numberDoFs);
    iDynTree::toEigen(s).setRandom();

    iDynTree::VectorDynSize ds(numberDoFs);
    iDynTree::toEigen(ds).setRandom();

    REQUIRE(kinDyn->setRobotState(iDynTree::Transform::Identity(),
                                  s,
                                  iDynTree::Twist::Zero(),
                                  ds,
                                  gravity));


    std::string linkInContact1 = "link5";
    std::string linkInContact2 = "link9";

    VariableHandler handler;

    handler.addVariable("base_acceleration", 6);
    handler.addVariable("joint_accelerations", numberDoFs);
    handler.addVariable("joint_torques", numberDoFs);
    handler.addVariable("link_in_contact_1", 6);
    handler.addVariable("link_in_contact_2", 6);

    ZMPElement element(kinDyn,
                       handler,
                       {{"link_in_contact_1", linkInContact1},
                        {"link_in_contact_2", linkInContact2}});

    iDynTree::Vector2 zero;
    zero.zero();

    iDynTree::Vector2 ZMP;
    ZMP(0) = ZMP(1) = 1;
    element.setDesiredZMP(ZMP);

    REQUIRE(iDynTree::toEigen(element.getB()) == iDynTree::toEigen(zero));
}

TEST_CASE("Check Joint Value Feasibility element of the ControlProblemElementsTest",
          "[ControlProblemElementsTest-JointValueFeasibility]")
{
    unsigned int numberOfJoints = 30;

    iDynTree::Model model = iDynTree::getRandomModel(numberOfJoints, 10);
    unsigned int numberDoFs = model.getNrOfDOFs();

    std::shared_ptr<iDynTree::KinDynComputations> kinDyn;
    kinDyn = std::make_shared<iDynTree::KinDynComputations>();

    kinDyn->loadRobotModel(model);
    kinDyn->setFrameVelocityRepresentation(iDynTree::MIXED_REPRESENTATION);
    REQUIRE(kinDyn->setFloatingBase("baseLink"));

    iDynTree::Vector3 gravity;
    gravity.zero();
    gravity(2) = -9.81;
    iDynTree::VectorDynSize s(numberDoFs);
    iDynTree::toEigen(s).setRandom();

    iDynTree::VectorDynSize ds(numberDoFs);
    iDynTree::toEigen(ds).setRandom();

    REQUIRE(kinDyn->setRobotState(iDynTree::Transform::Identity(),
                                  s,
                                  iDynTree::Twist::Zero(),
                                  ds,
                                  gravity));


    std::string linkInContact1 = "link5";
    std::string linkInContact2 = "link9";

    VariableHandler handler;

    handler.addVariable("base_acceleration", 6);
    handler.addVariable("joint_accelerations", numberDoFs);
    handler.addVariable("joint_torques", numberDoFs);
    handler.addVariable("link_in_contact_1", 6);
    handler.addVariable("link_in_contact_2", 6);

    iDynTree::VectorDynSize jointMaxLimits(numberDoFs);
    for (int i = 0; i < numberDoFs; i++)
        jointMaxLimits(i) = iDynTree::deg2rad(100);

    iDynTree::VectorDynSize jointMinLimits(numberDoFs);
    iDynTree::toEigen(jointMinLimits) = -iDynTree::toEigen(jointMaxLimits);

    JointValuesFeasibilityElement element(kinDyn,
                                          handler,
                                          "joint_accelerations",
                                          jointMaxLimits,
                                          jointMinLimits,
                                          0.01);

    std::cerr << element.getA().toString() << std::endl;
    std::cerr << element.getUpperBound().toString() << std::endl;
    std::cerr << element.getLowerBound().toString() << std::endl;
}
