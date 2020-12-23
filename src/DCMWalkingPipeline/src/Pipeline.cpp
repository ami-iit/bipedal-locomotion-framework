/**
 * @file Pipeline.cpp
 * @authors Giulio Romualdi
 * @copyright 2020 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the GNU Lesser General Public License v2.1 or any later version.
 */

#include <BipedalLocomotion/DCMWalkingPipeline/Pipeline.h>

// TODO remove me please
#include <iDynTree/Core/EigenHelpers.h>
#include <iDynTree/Model/Model.h>
#include <yarp/os/Bottle.h>
#include <yarp/os/Searchable.h>

using namespace BipedalLocomotion::DCMWalkingPipeline;
using namespace BipedalLocomotion;

Pipeline::Pipeline()
{
    m_kinDynComputation = std::make_shared<iDynTree::KinDynComputations>();
    m_phaseList = std::make_shared<Planners::ContactPhaseList>();
}

bool Pipeline::setRobotModel( const iDynTree::Model &model )
{
    m_kinDynComputation->loadRobotModel(model);
    m_kinDynComputation->setFrameVelocityRepresentation(iDynTree::MIXED_REPRESENTATION);

    // todo please remove me
    m_kinDynComputation->setFloatingBase("root_link");

    return true;
}

bool Pipeline::generateTrajectory(const manif::SE3d& baseTransform)
{
    // TODO remove me!
    const manif::SE3d::Translation& basePosition = baseTransform.translation();

    const double scaling = 0.85;
    const double stepLength = 0.13;

    Planners::ContactListMap contactListMap;
    Eigen::Vector3d leftPos;
    leftPos << 0 + basePosition(0), 0.08 + basePosition(1), 0;
    manif::SE3d leftTransform(leftPos, manif::SO3d::Identity());
    contactListMap["left"].addContact(leftTransform, 0.0, 1.0 * scaling);

    // second footstep
    leftPos(0) = stepLength + basePosition(0);
    leftTransform = manif::SE3d(leftPos, manif::SO3d::Identity());
    contactListMap["left"].addContact(leftTransform, 2.0 * scaling, 5.0 * scaling);

    leftPos(0) = 3 * stepLength + basePosition(0);
    leftPos(2) = 0.00;
    leftTransform = manif::SE3d(leftPos, manif::SO3d::Identity());
    contactListMap["left"].addContact(leftTransform, 6.0* scaling, 9.0 * scaling);

    leftPos(0) = 5 * stepLength + basePosition(0);
    leftPos(2) = 0.0;
    leftTransform = manif::SE3d(leftPos, manif::SO3d::Identity());
    contactListMap["left"].addContact(leftTransform, 10.0* scaling, 13.0* scaling);


    // right foot
    // first footstep
    Eigen::Vector3d rightPos;
    rightPos << 0 + basePosition(0), -0.08 + basePosition(1), 0;
    manif::SE3d rightTransform(rightPos, manif::SO3d::Identity());
    contactListMap["right"].addContact(rightTransform, 0.0, 3.0* scaling);

    // second footstep
    rightPos(0) = 2 * stepLength + basePosition(0);
    // rightPos(1) = basePosition(1);
    rightPos(2) = 0.0;
    rightTransform = manif::SE3d(rightPos, manif::SO3d::Identity());
    contactListMap["right"].addContact(rightTransform, 4.0* scaling, 7.0* scaling);

    // second footstep
    rightPos(0) = 4 * stepLength + basePosition(0);
    // rightPos(1) = basePosition(1);
    rightPos(2) = 0.0;
    rightTransform = manif::SE3d(rightPos, manif::SO3d::Identity());
    contactListMap["right"].addContact(rightTransform, 8.0* scaling, 11.0* scaling);

    rightPos(0) = 5 * stepLength + basePosition(0);
    // rightPos(1) = basePosition(1);
    rightPos(2) = 0.0;
    rightTransform = manif::SE3d(rightPos, manif::SO3d::Identity());
    contactListMap["right"].addContact(rightTransform, 12.0* scaling, 13.0* scaling);



    m_phaseList->setLists(contactListMap);


    // initialize DCM planner
    // TODO here we are initializing the DCM position as
    Planners::DCMPlannerState initialState;
    initialState.dcmPosition.setZero();
    initialState.dcmPosition[0] = 0.03 + basePosition(0);
    initialState.dcmPosition[1] = basePosition(1);
    initialState.dcmPosition[2] = 0.52;
    initialState.dcmVelocity.setZero();
    initialState.vrpPosition = initialState.dcmPosition;
    initialState.omega = std::sqrt(9.81 / initialState.dcmPosition[2]);

    m_dcmPlanner.setContactPhaseList(m_phaseList);
    m_dcmPlanner.setInitialState(initialState);
    m_dcmPlanner.computeTrajectory();

    // initialize left and right and left foot trajectory planner
    m_leftFootPlanner.setContactList(contactListMap["left"]);
    m_rightFootPlanner.setContactList(contactListMap["right"]);

    return true;
}

bool Pipeline::computeSimplifedModelControl()
{
    // compute the DCM position
    m_dcmPosition = iDynTree::toEigen(m_kinDynComputation->getCenterOfMassPosition())
                    + iDynTree::toEigen(m_kinDynComputation->getCenterOfMassVelocity())
                          / m_dcmPlanner.get().omega;

    // std::cerr << "real dcm position " << m_dcmPosition.transpose() << std::endl;
    // std::cerr << "desired dcm position " << m_dcmPlanner.get().dcmPosition.transpose() << std::endl;
    // std::cerr << "desired vrp position " << m_dcmPlanner.get().vrpPosition.transpose() << std::endl;

    // Eigen::Vector3d dcmTemp;
    // dcmTemp << 0.139924, -0.114984,  0.529931;

    // m_dcmController.setFeedForward(dcmTemp);
    // m_dcmController.setDesiredState(dcmTemp);


    m_dcmController.setFeedForward(m_dcmPlanner.get().vrpPosition);
    m_dcmController.setDesiredState(m_dcmPlanner.get().dcmPosition);
    m_dcmController.setState(m_dcmPosition);
    m_dcmController.computeControlLaw();

    return true;
}

bool Pipeline::computeWholeBodyControl()
{
    iDynTree::Vector3 dummy;
    dummy.zero();

    // set feet status

    // if(m_leftFootPlanner.get().isInContact != m_rightFootPlanner.get().isInContact)
    //     return false;

    m_torqueControl->setContactStateCartesianElement(m_leftFootPlanner.get().isInContact,
                                                     "left_foot");
    m_torqueControl->setContactStateCartesianElement(m_rightFootPlanner.get().isInContact,
                                                     "right_foot");

    // m_torqueControl->setContactStateCartesianElement(false,
    //                                                  "left_foot");
    // m_torqueControl->setContactStateCartesianElement(false,
    //                                                  "right_foot");


    m_torqueControl->setContactStateWrenchFeasibilityElement(m_leftFootPlanner.get().isInContact,
                                                             "left_foot");
    m_torqueControl->setContactStateWrenchFeasibilityElement(m_rightFootPlanner.get().isInContact,
                                                             "right_foot");

    // TODO
    // m_torqueControl->setWeight(0.5, "left_foot");
    // m_torqueControl->setWeight(0.5, "right_foot");

    // set desired desired centroidal momentum trajectory (the name of the function is misleading
    // please change the code. Actually this part of the code should be completely rewritten)
    iDynTree::Vector3 desiredRateOfChangeLinearMomentum;
    iDynTree::toEigen(desiredRateOfChangeLinearMomentum)
        = m_kinDynComputation->model().getTotalMass()
          * (m_dcmPlanner.get().omega * m_dcmPlanner.get().omega - m_dcmPlanner.get().omegaDot)
          * (iDynTree::toEigen(m_kinDynComputation->getCenterOfMassPosition())
             - m_dcmController.getControl().coeffs());

    m_torqueControl->setDesiredVRP(desiredRateOfChangeLinearMomentum);
    // iDynTree::Vector3 zero;
    // zero.zero();
    // iDynTree::Vector3 comTemp;
    // iDynTree::toEigen(comTemp) = m_dcmPlanner.get().dcmPosition;

    // m_torqueControl->setDesiredCartesianTrajectory(zero, zero, comTemp, "CoM");



    // set feet trajectories
    iDynTree::Transform transform = iDynTree::Transform(iDynTree::MatrixView<const double>(m_leftFootPlanner.get().transform.isometry().matrix()));
    iDynTree::SpatialMotionVector twist, acceleration;

    for (int i = 0; i < 6; i++)
        twist(i) = m_leftFootPlanner.get().mixedVelocity.coeffs()(i);

    for (int i = 0; i < 6; i++)
        acceleration(i) = m_leftFootPlanner.get().mixedAcceleration.coeffs()(i);

    m_torqueControl->setDesiredCartesianTrajectory(acceleration, twist, transform, "left_foot");

    transform = iDynTree::Transform(iDynTree::MatrixView<const double>(m_rightFootPlanner.get().transform.isometry().matrix()));

    for (int i = 0; i < 6; i++)
        twist(i) = m_rightFootPlanner.get().mixedVelocity.coeffs()(i);

    for (int i = 0; i < 6; i++)
        acceleration(i) = m_rightFootPlanner.get().mixedAcceleration.coeffs()(i);

    m_torqueControl->setDesiredCartesianTrajectory(acceleration, twist, transform, "right_foot");


    // // regularization term
    iDynTree::VectorDynSize dummyJoint(m_kinDynComputation->getNrOfDegreesOfFreedom());
    dummyJoint.zero();
    // std::cerr << m_qDesired.toString() << std::endl;
    m_torqueControl->setDesiredRegularizationTrajectory(dummyJoint, dummyJoint, m_qRegularization, "joint_accelerations");

    iDynTree::VectorDynSize jointPos(m_kinDynComputation->getNrOfDegreesOfFreedom());
    iDynTree::VectorDynSize jointVel(m_kinDynComputation->getNrOfDegreesOfFreedom());
    m_kinDynComputation->getJointPos(jointPos);
    m_kinDynComputation->getJointVel(jointVel);
    m_torqueControl->setJointState(jointVel, jointPos);



    // m_torqueControl->setJointState(m_simulator->jointVelocities(), m_simulator->jointPositions());

    // torso orientation
    // TODO remove me
    Eigen::Matrix3d r;
    r << 0.0, 0.0, 1.0,
        1.0, 0.0, 0.0,
        0.0, 1.0, 0.0;

    iDynTree::Rotation additionalRotation(r);
    double yawLeft = m_leftFootPlanner.get().transform.asSO3().quat().toRotationMatrix().eulerAngles(2, 1, 0)(0);
    double yawRight = m_rightFootPlanner.get().transform.asSO3().quat().toRotationMatrix().eulerAngles(2, 1, 0)(0);

    double meanYaw = std::atan2(std::sin(yawLeft) + std::sin(yawRight),
                                std::cos(yawLeft) + std::cos(yawRight));
    iDynTree::Rotation yawRotation = iDynTree::Rotation::RotZ(meanYaw);


    m_torqueControl->setDesiredCartesianTrajectory(dummy, dummy, yawRotation * additionalRotation, "torso");


    try{
        m_torqueControl->solve();
    }
    catch(const std::exception& e)
    {
        std::cerr << e.what() << std::endl;
        return false;
    }
    return true;
}



bool Pipeline::initialize(std::shared_ptr<ParametersHandler::IParametersHandler> handler)
{
    bool ok = m_dcmPlanner.initialize(handler->getGroup("DCM_PLANNER").lock());

    ok &= m_leftFootPlanner.initialize(handler->getGroup("SWING_FOOT_PLANNER").lock());
    ok &= m_rightFootPlanner.initialize(handler->getGroup("SWING_FOOT_PLANNER").lock());

    auto simplifiedModelControlOptions = handler->getGroup("SIMPLIFIED_MODEL_CONTROL").lock();

    double dcmGain;
    if (!simplifiedModelControlOptions->getParameter("dcm_gain", dcmGain))
    {
        return false;
    }

    m_dcmController.setGains(dcmGain);

    m_gravity.setZero();
    m_gravity(2) = -9.81;

    // TODO please remove me
    m_stream.open("Dataset.txt");
    m_stream << "dcm_x_des dcm_y_des dcm_z_des dcm_x dcm_y dcm_z "
                "lf_x_des lf_y_des lf_z_des "
                "lf_x lf_y lf_z "
                "rf_x_des rf_y_des rf_z_des "
                "rf_x rf_y rf_z "
                "vrp_x_des vrp_y_des vrp_z_des "
                "vrp_x vrp_y vrp_z "
                "omega d_omega "
             << std::endl;

    return ok;
}

bool Pipeline::initializeTorqueController(yarp::os::Searchable& options,
                                          Eigen::Ref<const Eigen::VectorXd> jointPositionUpperLimits,
                                          Eigen::Ref<const Eigen::VectorXd> jointPositionLowerLimits)
{
    iDynTree::VectorDynSize jointPositionUpperLimits_iDynTree(jointPositionUpperLimits.size());
    iDynTree::VectorDynSize jointPositionLowerLimits_iDynTree(jointPositionLowerLimits.size());

    iDynTree::toEigen(jointPositionUpperLimits_iDynTree) = jointPositionUpperLimits;
    iDynTree::toEigen(jointPositionLowerLimits_iDynTree) = jointPositionLowerLimits;

    m_torqueControl = std::make_unique<WalkingControllers::TaskBasedTorqueControlYARPInterface>(*m_kinDynComputation);
    try
    {
        m_torqueControl->initialize(options,
                                    jointPositionUpperLimits_iDynTree,
                                    jointPositionLowerLimits_iDynTree);
    }
    catch(const std::exception& e)
    {
        std::cerr << e.what() << std::endl;
        return false;
    }

    return true;
}

bool Pipeline::setRobotState(const manif::SE3d& world_T_base,
                             Eigen::Ref<const Eigen::VectorXd> jointPos,
                             const manif::SE3d::Tangent& baseVelocity,
                             Eigen::Ref<const Eigen::VectorXd> jointVel)
{
    bool ok = m_kinDynComputation->setRobotState(world_T_base.transform(),
                                              iDynTree::make_span(jointPos.data(), jointPos.size()),
                                              iDynTree::make_span(baseVelocity.data(),
                                                                  manif::SE3d::Tangent::DoF),
                                              iDynTree::make_span(jointVel.data(), jointVel.size()),
                                              iDynTree::make_span(m_gravity.data(),
                                                                  m_gravity.size()));
    return ok;
}

bool Pipeline::advance()
{
    bool ok = m_dcmPlanner.advance();

    ok &= m_leftFootPlanner.advance();
    ok &= m_rightFootPlanner.advance();
    return ok;
}

const manif::SE3d& Pipeline::getLeftFootTransform() const
{
    return m_leftFootPlanner.get().transform;
}

const manif::SE3d& Pipeline::getRightFootTransform() const
{
    return m_rightFootPlanner.get().transform;
}

Eigen::Ref<const Eigen::Vector3d> Pipeline::getDCMPosition() const
{
    return m_dcmPlanner.get().dcmPosition;
}

Eigen::Ref<const Eigen::VectorXd> Pipeline::getJointTorques() const
{
    return iDynTree::toEigen(m_torqueControl->getDesiredTorques());
}

bool Pipeline::computeControl()
{
    bool ret = this->computeSimplifedModelControl();

    ret = ret && this->computeWholeBodyControl();

    m_stream << m_dcmPlanner.get().dcmPosition.transpose() << " " << m_dcmPosition.transpose()
             << " " << m_leftFootPlanner.get().transform.translation().transpose() << " "
             << m_kinDynComputation->getWorldTransform("l_sole").getPosition().toString() << " "
             << m_rightFootPlanner.get().transform.translation().transpose() << " "
             << m_kinDynComputation->getWorldTransform("r_sole").getPosition().toString() << " "
             << m_dcmPlanner.get().vrpPosition.transpose() << " " << m_dcmController.getControl().coeffs().transpose() << " " << m_dcmPlanner.get().omega << " " << m_dcmPlanner.get().omegaDot
             << std::endl;
    return ret;
}

void Pipeline::setRegularization(Eigen::Ref<const Eigen::VectorXd> reg)
{
    m_qRegularization.resize(reg.size());

    iDynTree::toEigen(m_qRegularization) = reg;
}


void Pipeline::close()
{
    m_stream.close();
}
