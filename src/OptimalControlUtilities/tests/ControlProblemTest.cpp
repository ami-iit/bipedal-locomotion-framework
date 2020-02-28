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
#include <BipedalLocomotionControllers/OptimalControlUtilities/CentroidalMomentumElementsWithCompliantContacts.h>
#include <BipedalLocomotionControllers/OptimalControlUtilities/CentroidalMomentumRateOfChangeElements.h>
#include <BipedalLocomotionControllers/OptimalControlUtilities/ControlProblemElements.h>
#include <BipedalLocomotionControllers/OptimalControlUtilities/FeasibilityElements.h>
#include <BipedalLocomotionControllers/OptimalControlUtilities/FloatingBaseMultiBodyDynamicsElements.h>
#include <BipedalLocomotionControllers/OptimalControlUtilities/PDController.h>
#include <BipedalLocomotionControllers/OptimalControlUtilities/RegularizationElements.h>
#include <BipedalLocomotionControllers/OptimalControlUtilities/VariableHandler.h>
#include <BipedalLocomotionControllers/OptimalControlUtilities/ZMPElements.h>
#include <BipedalLocomotionControllers/OptimalControlUtilities/ContactModelElement.h>
#include <BipedalLocomotionControllers/OptimalControlUtilities/PIDController.h>

#include <BipedalLocomotionControllers/ContactModels/ContinuousContactModel.h>


using namespace BipedalLocomotionControllers::OptimalControlUtilities;

template <typename T, typename U>
void checkVectorsAreEqual(const T& vector1, const U& vector2, double tol = 0)
{
    // the tolerance has to be positive
    REQUIRE(tol >= 0);

    // the vectors must have the same size
    REQUIRE(vector1.size() == vector2.size());

    for (std::size_t i = 0; i < vector1.size(); i++)
        REQUIRE(std::abs(vector1[i] - vector2[i]) <= tol);
}

template <typename T, typename U>
void checkMatricesAreEqual(const T& matrix1, const U& matrix2, double tol = 0)
{
    // the tolerance has to be positive
    REQUIRE(tol >= 0);

    // the matrices must have the same size
    REQUIRE(matrix1.cols() == matrix2.cols());
    REQUIRE(matrix1.rows() == matrix2.rows());

    for (std::size_t i = 0; i < matrix1.rows(); i++)
        for (std::size_t j = 0; i < matrix1.cols(); j++)
            REQUIRE(std::abs(matrix1(i, j) - matrix2(i, j)) <= tol);
}

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
        iDynTree::Vector3 gains;
        gains(0) = 1;
        gains(1) = 1;
        gains(2) = 1;
        LinearPD<iDynTree::Vector3>pd(gains, gains);
        CartesianElement<CartesianElementType::POSITION> element(kinDyn,
                                                                 pd,
                                                                 handler,
                                                                 frame);
        iDynTree::Vector3 dummy;
        dummy.zero();
        element.setReference(dummy, dummy, dummy);
        element.isInContact(false);

        REQUIRE(kinDyn->getFrameFreeFloatingJacobian(frame, jacobian));
        REQUIRE(
            iDynTree::toEigen(element.getA()).block(0, baseAccelerationOffset, 3, numberDoFs + 6)
            == iDynTree::toEigen(jacobian).block(0, 0, 3, numberDoFs + 6));


        auto pdTest = std::make_unique<LinearPD<iDynTree::Vector3>>(gains, gains);
        pdTest->setDesiredTrajectory(dummy, dummy, dummy);
        pdTest->setFeedback(kinDyn->getFrameVel(frame).getLinearVec3(),
                            kinDyn->getWorldTransform(frame).getPosition());

        REQUIRE(iDynTree::toEigen(element.getB())
                == -iDynTree::toEigen(kinDyn->getFrameBiasAcc(frame)).head(3)
                       + iDynTree::toEigen(pdTest->getControllerOutput()));
    }

    SECTION("Test Orientation Element")
    {
        OrientationPD pd;
        pd.setGains(1, 1, 1);
        CartesianElement<CartesianElementType::ORIENTATION> element(kinDyn, pd, handler, frame);

        iDynTree::Vector3 dummy;
        dummy.zero();
        element.setReference(dummy, dummy, iDynTree::Rotation::Identity());

        REQUIRE(kinDyn->getFrameFreeFloatingJacobian(frame, jacobian));
        REQUIRE(
            iDynTree::toEigen(element.getA()).block(0, baseAccelerationOffset, 3, numberDoFs + 6)
            == iDynTree::toEigen(jacobian).block(3, 0, 3, numberDoFs + 6));

        OrientationPD pdTest;
        pdTest.setGains(1, 1, 1);
        pdTest.setDesiredTrajectory(dummy, dummy, iDynTree::Rotation::Identity());
        pdTest.setFeedback(kinDyn->getFrameVel(frame).getAngularVec3(),
                        kinDyn->getWorldTransform(frame).getRotation());

        REQUIRE(iDynTree::toEigen(element.getB())
                == -iDynTree::toEigen(kinDyn->getFrameBiasAcc(frame)).tail(3)
                       + iDynTree::toEigen(pdTest.getControllerOutput()));
    }

    SECTION("Test Pose Element")
    {
        iDynTree::Vector3 gains;
        gains(0) = 1;
        gains(1) = 1;
        gains(2) = 1;
        double scalarGain = 1;

        PosePD pd;
        pd.setGains(gains, gains, scalarGain, scalarGain, scalarGain);
        CartesianElement<CartesianElementType::POSE> element(kinDyn, pd, handler, frame);

        iDynTree::Twist dummyTwist;
        iDynTree::SpatialAcc dummySpatialAcc;
        dummyTwist.zero();
        dummySpatialAcc.zero();
        element.setReference(dummySpatialAcc, dummyTwist, iDynTree::Transform::Identity());

        REQUIRE(kinDyn->getFrameFreeFloatingJacobian(frame, jacobian));
        REQUIRE(
            iDynTree::toEigen(element.getA()).block(0, baseAccelerationOffset, 6, numberDoFs + 6)
            == iDynTree::toEigen(jacobian));

        // Linear part
        PosePD pdTest;
        pdTest.setGains(gains, gains, scalarGain, scalarGain, scalarGain);
        pdTest.setDesiredTrajectory(dummySpatialAcc, dummyTwist, iDynTree::Transform::Identity());

        pdTest.setFeedback(kinDyn->getFrameVel(frame), kinDyn->getWorldTransform(frame));
        REQUIRE(iDynTree::toEigen(element.getB())
                == -iDynTree::toEigen(kinDyn->getFrameBiasAcc(frame))
                       + iDynTree::toEigen(pdTest.getControllerOutput()));
    }

    SECTION("One Degree of Freedom - X")
    {
        double gain = 1;
        LinearPD<double> pd;
        pd.setGains(gain, gain);
        CartesianElement<CartesianElementType::POSITION, CartesianElementAxisName::X>
            element(kinDyn, pd, handler, frame);

        double dummy = 0;
        element.setReference(dummy, dummy, dummy);

        REQUIRE(kinDyn->getFrameFreeFloatingJacobian(frame, jacobian));
        REQUIRE(
            iDynTree::toEigen(element.getA()).block(0, baseAccelerationOffset, 1, numberDoFs + 6)
            == iDynTree::toEigen(jacobian).block(0, 0, 1, numberDoFs + 6));

        LinearPD<double> pdTest;
        pdTest.setGains(gain, gain);
        pdTest.setDesiredTrajectory(dummy, dummy, dummy);
        pdTest.setFeedback(kinDyn->getFrameVel(frame).getLinearVec3()(0),
                        kinDyn->getWorldTransform(frame).getPosition()(0));
        REQUIRE(element.getB()(0)
                == -kinDyn->getFrameBiasAcc(frame)(0) + pdTest.getControllerOutput());
    }

    SECTION("One Degree of Freedom - Y")
    {
        double gain = 1;
        LinearPD<double> pd;
        pd.setGains(gain, gain);
        CartesianElement<CartesianElementType::POSITION, CartesianElementAxisName::Y>
            element(kinDyn, pd, handler, frame);

        double dummy = 0;
        element.setReference(dummy, dummy, dummy);

        REQUIRE(kinDyn->getFrameFreeFloatingJacobian(frame, jacobian));
        REQUIRE(
            iDynTree::toEigen(element.getA()).block(0, baseAccelerationOffset, 1, numberDoFs + 6)
            == iDynTree::toEigen(jacobian).block(1, 0, 1, numberDoFs + 6));

        LinearPD<double> pdTest;
        pdTest.setGains(gain, gain);
        pdTest.setDesiredTrajectory(dummy, dummy, dummy);
        pdTest.setFeedback(kinDyn->getFrameVel(frame).getLinearVec3()(1),
                        kinDyn->getWorldTransform(frame).getPosition()(1));
        REQUIRE(element.getB()(0)
                == -kinDyn->getFrameBiasAcc(frame)(1) + pdTest.getControllerOutput());
    }

    SECTION("One Degree of Freedom - Z")
    {
        double gain = 1;
        LinearPD<double> pd;
        pd.setGains(gain, gain);
        CartesianElement<CartesianElementType::POSITION, CartesianElementAxisName::Z>
            element(kinDyn, pd, handler, frame);

        double dummy = 0;
        element.setReference(dummy, dummy, dummy);

        REQUIRE(kinDyn->getFrameFreeFloatingJacobian(frame, jacobian));
        REQUIRE(
            iDynTree::toEigen(element.getA()).block(0, baseAccelerationOffset, 1, numberDoFs + 6)
            == iDynTree::toEigen(jacobian).block(2, 0, 1, numberDoFs + 6));

        LinearPD<double> pdTest;
        pdTest.setGains(gain, gain);
        pdTest.setDesiredTrajectory(dummy, dummy, dummy);
        pdTest.setFeedback(kinDyn->getFrameVel(frame).getLinearVec3()(2),
                        kinDyn->getWorldTransform(frame).getPosition()(2));
        REQUIRE(element.getB()(0)
                == -kinDyn->getFrameBiasAcc(frame)(2) + pdTest.getControllerOutput());
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

    iDynTree::MatrixDynSize massMatrix(numberDoFs + 6, numberDoFs + 6);
    REQUIRE(kinDyn->getFreeFloatingMassMatrix(massMatrix));

    iDynTree::MatrixDynSize identity(numberDoFs, numberDoFs);
    iDynTree::toEigen(identity).setIdentity();

    iDynTree::MatrixDynSize jacobianLink1(6, numberDoFs + 6);
    iDynTree::MatrixDynSize jacobianLink2(6, numberDoFs + 6);
    REQUIRE(kinDyn->getFrameFreeFloatingJacobian(linkInContact1, jacobianLink1));
    REQUIRE(kinDyn->getFrameFreeFloatingJacobian(linkInContact2, jacobianLink2));

    iDynTree::FreeFloatingGeneralizedTorques generalizedBiasForces;
    generalizedBiasForces.resize(kinDyn->model());
    kinDyn->generalizedBiasForces(generalizedBiasForces);

    SECTION("Whole-Body Floating Base Dynamics Element")
    {
        // Instantiate the element
        WholeBodyFloatingBaseDynamicsElement element(kinDyn,
                                                 handler,
                                                 {{"link_in_contact_1", linkInContact1},
                                                  {"link_in_contact_2", linkInContact2}});

        // check the matrix A
        REQUIRE(iDynTree::toEigen(element.getA()).block(0, 0, numberDoFs + 6, numberDoFs + 6)
                == -iDynTree::toEigen(massMatrix));

        REQUIRE(iDynTree::toEigen(element.getA()).block(6, numberDoFs + 6, numberDoFs, numberDoFs)
                == iDynTree::toEigen(identity));

        REQUIRE(iDynTree::toEigen(element.getA()).block(0, 2 * numberDoFs + 6, 6 + numberDoFs, 6)
                == iDynTree::toEigen(jacobianLink1).transpose());

        REQUIRE(iDynTree::toEigen(element.getA()).block(0, 2 * numberDoFs + 6 + 6, 6 + numberDoFs, 6)
                == iDynTree::toEigen(jacobianLink2).transpose());

        REQUIRE(iDynTree::toEigen(element.getB()).head(6)
                == iDynTree::toEigen(generalizedBiasForces.baseWrench()));
        REQUIRE(iDynTree::toEigen(element.getB()).tail(numberDoFs)
                == iDynTree::toEigen(generalizedBiasForces.jointTorques()));
    }

    SECTION("Floating Base Dynamics Element")
    {
        // Instantiate the element
        FloatingBaseDynamicsElement element(kinDyn,
                                            handler,
                                            {{"link_in_contact_1", linkInContact1, false},
                                             {"link_in_contact_2", linkInContact2, false}});

        // check the matrix A
        REQUIRE(iDynTree::toEigen(element.getA()).block(0, 0, 6, numberDoFs + 6)
                == -iDynTree::toEigen(massMatrix).topRows(6));

        REQUIRE(iDynTree::toEigen(element.getA()).block(0, 2 * numberDoFs + 6, 6, 6)
                == iDynTree::toEigen(jacobianLink1).transpose().topRows(6));

        REQUIRE(iDynTree::toEigen(element.getA()).block(0, 2 * numberDoFs + 6 + 6, 6, 6)
                == iDynTree::toEigen(jacobianLink2).transpose().topRows(6));

        REQUIRE(iDynTree::toEigen(element.getB())
                == iDynTree::toEigen(generalizedBiasForces.baseWrench()));

    }

    SECTION("Floating Base Dynamics Element with compliant contacts")
    {
        // Instantiate the element
        FloatingBaseDynamicsElement element(kinDyn,
                                            handler,
                                            {{"link_in_contact_1", linkInContact1},
                                             {"link_in_contact_2", linkInContact2}});

        element.setCompliantContact("link_in_contact_1", true);
        element.setCompliantContact("link_in_contact_2", true);


        iDynTree::Wrench wrench1, wrench2;
        iDynTree::toEigen(wrench1.getLinearVec3()).setRandom();
        iDynTree::toEigen(wrench1.getAngularVec3()).setRandom();

        iDynTree::toEigen(wrench2.getLinearVec3()).setRandom();
        iDynTree::toEigen(wrench2.getAngularVec3()).setRandom();

        element.setMeasuredContactWrenches(
            {{"link_in_contact_1", wrench1}, {"link_in_contact_2", wrench2}});

        // check the matrix A
        REQUIRE(iDynTree::toEigen(element.getA()).block(0, 0, 6, numberDoFs + 6)
                == -iDynTree::toEigen(massMatrix).topRows(6));

        // check the vector B
        iDynTree::Vector6 computedB;
        iDynTree::toEigen(computedB) = iDynTree::toEigen(generalizedBiasForces.baseWrench());
        iDynTree::toEigen(computedB) -= iDynTree::toEigen(jacobianLink1).transpose().topRows(6) * iDynTree::toEigen(wrench1);
        iDynTree::toEigen(computedB) -= iDynTree::toEigen(jacobianLink2).transpose().topRows(6) * iDynTree::toEigen(wrench2);

        checkVectorsAreEqual(element.getB(), computedB, 1e-4);
    }

    SECTION("Joint space Dynamics Element")
    {
        // Instantiate the element
        JointSpaceDynamicsElement element(kinDyn,
                                          handler,
                                          {{"link_in_contact_1", linkInContact1},
                                           {"link_in_contact_2", linkInContact2}});

        // check the matrix A
        REQUIRE(iDynTree::toEigen(element.getA()).block(0, 0, numberDoFs, numberDoFs + 6)
                == -iDynTree::toEigen(massMatrix).bottomRows(numberDoFs));

        REQUIRE(iDynTree::toEigen(element.getA()).block(0, numberDoFs + 6, numberDoFs, numberDoFs)
                == iDynTree::toEigen(identity));

        REQUIRE(iDynTree::toEigen(element.getA()).block(0, 2 * numberDoFs + 6,  numberDoFs, 6)
                == iDynTree::toEigen(jacobianLink1).transpose().bottomRows(numberDoFs));

        REQUIRE(iDynTree::toEigen(element.getA()).block(0, 2 * numberDoFs + 6 + 6, numberDoFs, 6)
                == iDynTree::toEigen(jacobianLink2).transpose().bottomRows(numberDoFs));

        REQUIRE(iDynTree::toEigen(element.getB())
                == iDynTree::toEigen(generalizedBiasForces.jointTorques()));
    }
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

    SECTION("Linear Momentum Rate of change")
    {
        PIDController<iDynTree::Vector3> pid(gain, gain, gain);
        CentroidalLinearMomentumRateOfChangeElement element(kinDyn,
                                                            pid,
                                                            handler,
                                                            {{"link_in_contact_1", linkInContact1},
                                                             {"link_in_contact_2",linkInContact2}});

        iDynTree::Wrench wrench1, wrench2;
        wrench1.zero();
        wrench2.zero();

        element.setMeasuredContactWrenches(
            {{"link_in_contact_1", wrench1}, {"link_in_contact_2", wrench2}});
        element.setReference(dummy, dummy, dummy, dummy);

        // Check A
        REQUIRE(iDynTree::toEigen(element.getA()).block(0, 2 * numberDoFs + 6, 3, 3)
                == iDynTree::toEigen(identity));
        REQUIRE(iDynTree::toEigen(element.getA()).block(0, 2 * numberDoFs + 6 + 6, 3, 3)
                == iDynTree::toEigen(identity));

        // Compute b
        // instantiate the PD controller
        PIDController<iDynTree::Vector3> pidTest(gain, gain, gain);
        pidTest.setReference(dummy, dummy, dummy, dummy);

        // compute the rate of change of the linear centroidal momentum (given by the sum of the
        // external forces)
        double weight = -9.81 * kinDyn->model().getTotalMass();
        iDynTree::LinearForceVector3 linearCentroidalMomentumRateOfChange(0, 0, weight);
        linearCentroidalMomentumRateOfChange
            = linearCentroidalMomentumRateOfChange + wrench1.getLinearVec3();
        linearCentroidalMomentumRateOfChange
            = linearCentroidalMomentumRateOfChange + wrench2.getLinearVec3();

        pidTest.setFeedback(linearCentroidalMomentumRateOfChange,
                            kinDyn->getCentroidalTotalMomentum().getLinearVec3(),
                            kinDyn->getCenterOfMassPosition());

        iDynTree::Vector3 computedB = pidTest.getControllerOutput();

        // Test b
        checkVectorsAreEqual(computedB, element.getB());
    }

    SECTION("Linear Momentum With Compliant contacts")
    {
        double springCoeff = 2000.0;
        double damperCoeff = 100.0;

        double length = 0.12;
        double width = 0.09;

        std::unordered_map<std::string, std::any> parameters({{"length", length},
                                                              {"width", width},
                                                              {"spring_coeff", springCoeff},
                                                              {"damper_coeff", damperCoeff}});

        using namespace BipedalLocomotionControllers::ContactModels;
        std::shared_ptr<ContactModel> contactModel1
            = std::make_shared<ContinuousContactModel>(parameters);

        std::shared_ptr<ContactModel> contactModel2(contactModel1);

        auto pd = std::make_unique<LinearPD<iDynTree::Vector3>>(gain, gain);
        CentroidalLinearMomentumElementWithCompliantContact
            element(kinDyn,
                    std::move(pd),
                    handler,
                    {{"link_in_contact_1", linkInContact1, contactModel1},
                     {"link_in_contact_2", linkInContact2, contactModel2}});

        element.setContactState("link_in_contact_1", true, iDynTree::Transform::Identity());
        element.setContactState("link_in_contact_2", true, iDynTree::Transform::Identity());

        iDynTree::LinearForceVector3 force1, force2;
        force1.zero();
        force2.zero();

        element.setMeasuredContactForces({force1, force2});
        element.setDesiredCentroidalLinearMomentum(dummy, dummy, dummy);

        // compute Jacobian matrices
        iDynTree::MatrixDynSize jacobianLink1(6, numberDoFs + 6);
        iDynTree::MatrixDynSize jacobianLink2(6, numberDoFs + 6);
        REQUIRE(kinDyn->getFrameFreeFloatingJacobian(linkInContact1, jacobianLink1));
        REQUIRE(kinDyn->getFrameFreeFloatingJacobian(linkInContact2, jacobianLink2));

        // compute bias accelerations
        auto biasAcceleration1 = kinDyn->getFrameBiasAcc(linkInContact1);
        auto biasAcceleration2 = kinDyn->getFrameBiasAcc(linkInContact2);


        // TEST  element A and b
        // Compute A
        iDynTree::MatrixDynSize computedA(3, numberDoFs + 6);
        iDynTree::toEigen(computedA)
            = iDynTree::toEigen(contactModel1->getControlMatrix()).topRows(3)
              * iDynTree::toEigen(jacobianLink1);

        iDynTree::toEigen(computedA)
            += iDynTree::toEigen(contactModel2->getControlMatrix()).topRows(3)
              * iDynTree::toEigen(jacobianLink2);

        // Test A
        REQUIRE(iDynTree::toEigen(computedA) == iDynTree::toEigen(element.getA()).leftCols(numberDoFs + 6));

        // Compute b
        // instantiate the PD controller
        LinearPD<iDynTree::Vector3> pdTest;
        pdTest.setGains(gain, gain);
        pdTest.setDesiredTrajectory(dummy, dummy, dummy);

        // compute the rate of change of the linear centroidal momentum (given by the sum of the external forces)
        double weight = -9.81 * kinDyn->model().getTotalMass();
        iDynTree::LinearForceVector3 linearCentroidalMomentumRateOfChange(0, 0, weight);
        linearCentroidalMomentumRateOfChange = linearCentroidalMomentumRateOfChange + force1;
        linearCentroidalMomentumRateOfChange = linearCentroidalMomentumRateOfChange + force2;

        pdTest.setFeedback(linearCentroidalMomentumRateOfChange,
                           kinDyn->getCentroidalTotalMomentum().getLinearVec3());

        iDynTree::Vector3 computedB = pdTest.getControllerOutput();

        iDynTree::toEigen(computedB)
            -= iDynTree::toEigen(contactModel1->getAutonomousDynamics()).head(3)
               + iDynTree::toEigen(contactModel1->getControlMatrix()).topRows(3)
                     * iDynTree::toEigen(biasAcceleration1);

        iDynTree::toEigen(computedB)
            -= iDynTree::toEigen(contactModel2->getAutonomousDynamics()).head(3)
               + iDynTree::toEigen(contactModel2->getControlMatrix()).topRows(3)
                     * iDynTree::toEigen(biasAcceleration2);

        // Test b
        checkVectorsAreEqual(computedB, element.getB());
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
        iDynTree::VectorDynSize gains(numberDoFs);
        iDynTree::toEigen(gains).setOnes();


        auto controller = std::make_unique<LinearPD<iDynTree::VectorDynSize>>(gains, gains);
        RegularizationWithControlElement element(kinDyn, std::move(controller), handler, "joint_accelerations");

        iDynTree::VectorDynSize dummyRef(numberDoFs);
        dummyRef.zero();

        element.setState(ds, s);
        element.setDesiredTrajectory(dummyRef, dummyRef, dummyRef);

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

TEST_CASE("Contact Model")
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
    handler.addVariable("link_in_contact_1", 6);
    handler.addVariable("link_in_contact_2", 6);

    iDynTree::VectorDynSize identityVector(6);
    for (auto& element : identityVector)
        element = -1;
    iDynTree::MatrixDynSize identity(6,6);
    iDynTree::toEigen(identity) = iDynTree::toEigen(identityVector).asDiagonal();

    double springCoeff = 2000.0;
    double damperCoeff = 100.0;

    double length = 0.12;
    double width = 0.09;

    std::unordered_map<std::string, std::any> parameters({{"length", length},
                                                          {"width", width},
                                                          {"spring_coeff", springCoeff},
                                                          {"damper_coeff", damperCoeff}});

    using namespace BipedalLocomotionControllers::ContactModels;
    std::shared_ptr<ContactModel> contactModel
        = std::make_shared<ContinuousContactModel>(parameters);

    ContactModelElement element(kinDyn, handler, {"link_in_contact_1", linkInContact1, contactModel});

    element.setContactState(true, iDynTree::Transform::Identity());

    // compute A
    iDynTree::MatrixDynSize jacobianLink(6, numberDoFs + 6);
    REQUIRE(kinDyn->getFrameFreeFloatingJacobian(linkInContact1, jacobianLink));


    // Check A
    REQUIRE(iDynTree::toEigen(identity) == iDynTree::toEigen(element.getA()).block(0, numberDoFs + 6, 6, 6));
    REQUIRE(iDynTree::toEigen(contactModel->getControlMatrix()) * iDynTree::toEigen(jacobianLink)
            == iDynTree::toEigen(element.getA()).block(0, 0, 6, numberDoFs + 6));

    // Compute b
    iDynTree::Vector6 computedB;
    auto biasAcceleration = kinDyn->getFrameBiasAcc(linkInContact1);
    iDynTree::toEigen(computedB) = - iDynTree::toEigen(contactModel->getAutonomousDynamics())
                                   - iDynTree::toEigen(contactModel->getControlMatrix())
                                         * iDynTree::toEigen(biasAcceleration);

    checkVectorsAreEqual(element.getB(), computedB);
}
