/**
 * @file Simulator.h
 * @authors Giulio Romualdi
 * @copyright 2020 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the GNU Lesser General Public License v2.1 or any later version.
 */

#ifndef BIPEDAL_LOCOMOTION_CONTROLLERS_SIMULATOR_SIMULATOR_H
#define BIPEDAL_LOCOMOTION_CONTROLLERS_SIMULATOR_SIMULATOR_H

#include <memory>

#include <iDynTree/Core/VectorDynSize.h>
#include <iDynTree/KinDynComputations.h>
#include <iDynTree/Model/Model.h>
#include <iDynTree/Model/FreeFloatingState.h>

#include <BipedalLocomotionControllers/ContactModels/ContinuousContactModel.h>
#include <BipedalLocomotionControllers/Simulator/Integrator.h>
#include <BipedalLocomotionControllers/ParametersHandler/IParametersHandler.h>

namespace BipedalLocomotionControllers
{
namespace Simulator
{
class Simulator
{
    struct Contact
    {
        std::unique_ptr<ContactModels::ContinuousContactModel> model;
        iDynTree::FrameIndex indexInTheModel;
        iDynTree::Transform frameNullForce;
        iDynTree::MatrixDynSize jacobian;
        bool isInContact{true};
    };

    std::unordered_map<std::string, std::pair<const std::string, const iDynTree::Transform>> m_baseFrames;

    //TODO move from here
    double m_length, m_width, m_springCoeff, m_damperCoeff;

    std::size_t m_numberOfDoF;
    double m_totalMass;
    double m_dT;

    Contact m_leftContact;
    Contact m_rightContact;

    iDynTree::KinDynComputations m_kinDyn;

    iDynTree::VectorDynSize m_jointAcceleration;
    iDynTree::VectorDynSize m_jointVelocity;
    iDynTree::VectorDynSize m_jointPosition;

    iDynTree::SpatialAcc m_baseAcceleration;
    iDynTree::Twist m_baseTwist;
    iDynTree::Transform m_baseTransform;

    iDynTree::VectorDynSize m_generalizedJointTorques;

    iDynTree::MatrixDynSize m_massMatrix;
    iDynTree::MatrixDynSize m_massMatrixInverse;
    iDynTree::FreeFloatingGeneralizedTorques m_generalizedBiasForces;

    std::unique_ptr<Integrator<iDynTree::VectorDynSize>> m_jointVelocityIntegrator;
    std::unique_ptr<Integrator<iDynTree::VectorDynSize>> m_jointPositionIntegrator;

    std::unique_ptr<Integrator<iDynTree::LinearMotionVector3>> m_baseLinearVelocityIntegrator;
    std::unique_ptr<Integrator<iDynTree::AngularMotionVector3>> m_baseAngularVelocityIntegrator;

    std::unique_ptr<Integrator<iDynTree::Vector3>> m_basePositionIntegrator;
    std::unique_ptr<Integrator<iDynTree::Matrix3x3>> m_baseRotationIntegrator;

    iDynTree::Vector3 m_gravity;
    iDynTree::FrameIndex m_baseFrame;

    bool setBaseFrame(const iDynTree::FrameIndex& frameBaseIndex, const std::string& name);

    enum class ControlMode
    {
        Acceleration,
        Torque
    };

    enum class State
    {
        NotInitialized,
        Initialized,
        Ready,
        Running
    };

    State m_state{State::NotInitialized};
    ControlMode m_controlMode;

public:


    Simulator(const iDynTree::Model& model);

    /* template <class Derived> */
    /* Simulator(const iDynTree::Model& model, */
    /*           std::unique_ptr<ParametersHandler::IParametersHandler<Derived>> handler) */
    /* { */
    /*     m_numberOfDoF = model.getNrOfDOFs(); */
    /*     m_totalMass = model.getTotalMass(); */

    /*     m_kinDyn.loadRobotModel(model); */
    /*     m_kinDyn.setFrameVelocityRepresentation(iDynTree::MIXED_REPRESENTATION); */

    /*     // get contact models parameters */

    /*     if (!handler->getParameter("length", m_length) || !handler->getParameter("width", m_width) */
    /*         || !handler->getParameter("spring_coeff", m_springCoeff) */
    /*         || !handler->getParameter("damper_coeff", m_damperCoeff)) */
    /*     { */
    /*         std::cerr << "[MomentumBasedTorqueControl::addContactModelElement] Unable to get the " */
    /*                      "contact parameters." */
    /*                   << std::endl; */
    /*     } */

    /*     const std::unordered_map<std::string, std::any> parameters({{"length", m_length}, */
    /*                                                                 {"width", m_width}, */
    /*                                                                 {"spring_coeff", m_springCoeff}, */
    /*                                                                 {"damper_coeff", m_damperCoeff}}); */

    /*     m_leftContact.model = std::make_unique<ContactModels::ContinuousContactModel>(parameters); */
    /*     m_rightContact.model = std::make_unique<ContactModels::ContinuousContactModel>(parameters); */

    /*     // get the contact frames */
    /*     std::string footFrame; */
    /*     if (!handler->getParameter("left_foot_frame", footFrame)) */
    /*     { */
    /*         std::cerr << "[MomentumBasedTorqueControl::addContactModelElement] Unable to get the " */
    /*                      "frame name." */
    /*                   << std::endl; */
    /*     } */
    /*     m_leftContact.indexInTheModel = m_kinDyn.model().getFrameIndex(footFrame); */
    /*     if(m_leftContact.indexInTheModel == iDynTree::FRAME_INVALID_INDEX) */
    /*     { */
    /*         std::cerr << "[WalkingFK::initialize] Unable to find the frame named: " << footFrame */
    /*                   << std::endl; */
    /*     } */

    /*     if (!handler->getParameter("right_foot_frame", footFrame)) */
    /*     { */
    /*         std::cerr << "[MomentumBasedTorqueControl::addContactModelElement] Unable to get the " */
    /*                      "frame name." */
    /*                   << std::endl; */
    /*     } */
    /*     m_rightContact.indexInTheModel = m_kinDyn.model().getFrameIndex(footFrame); */
    /*     if (m_rightContact.indexInTheModel == iDynTree::FRAME_INVALID_INDEX) */
    /*     { */
    /*         std::cerr << "[WalkingFK::initialize] Unable to find the frame named: " << footFrame */
    /*                   << std::endl; */
    /*     } */

    /*     if(!setBaseFrame(m_leftContact.indexInTheModel, "left_foot")) */
    /*     { */
    /*         std::cerr << "[initialize] Unable to set the leftFootFrame." << std::endl; */
    /*     } */

    /*     // set the right foot frame */
    /*     if(!setBaseFrame(m_rightContact.indexInTheModel, "right_foot")) */
    /*     { */
    /*         std::cerr << "[initialize] Unable to set the rightFootFrame." << std::endl; */
    /*     } */


    /*     // get the base frame */
    /*     std::string baseFrame; */
    /*     if (!handler->getParameter("base_frame", baseFrame)) */
    /*     { */
    /*         std::cerr << "[MomentumBasedTorqueControl::addContactModelElement] Unable to get the " */
    /*                      "frame name." */
    /*                   << std::endl; */
    /*     } */
    /*     m_baseFrame = m_kinDyn.model().getFrameIndex(baseFrame); */

    /*     if(!setBaseFrame(m_baseFrame, "root")) */
    /*     { */
    /*         std::cerr << "[initialize] Unable to set the rightFootFrame." << std::endl; */
    /*     } */

    /*     if (!handler->getParameter("sampling_time", m_dT)) */
    /*     { */
    /*         std::cerr << "[MomentumBasedTorqueControl::addContactModelElement] Unable to get the " */
    /*                      "frame name." */
    /*                   << std::endl; */
    /*     } */

    /*     std::string controlMode; */
    /*     if (!handler->getParameter("control_mode", controlMode)) */
    /*     { */
    /*         std::cerr << "[MomentumBasedTorqueControl::addContactModelElement] Unable to get the " */
    /*                      "control mode." */
    /*                   << std::endl; */
    /*     } */

    /*     if(controlMode == "torque") */
    /*         m_controlMode = ControlMode::Torque; */
    /*     else if(controlMode == "acceleration") */
    /*         m_controlMode = ControlMode::Acceleration; */
    /*     else */
    /*         std::cerr << "[MomentumBasedTorqueControl::addContactModelElement]  The desired " */
    /*                      "control mode is not yet supported" */
    /*                   << std::endl; */

    /*     // instantiate the integrates */
    /*     m_jointVelocityIntegrator = std::make_unique<Integrator<iDynTree::VectorDynSize>>(m_dT); */
    /*     m_jointPositionIntegrator = std::make_unique<Integrator<iDynTree::VectorDynSize>>(m_dT); */

    /*     m_baseLinearVelocityIntegrator = std::make_unique<Integrator<iDynTree::LinearMotionVector3>>(m_dT); */
    /*     m_baseAngularVelocityIntegrator = std::make_unique<Integrator<iDynTree::AngularMotionVector3>>(m_dT); */

    /*     m_basePositionIntegrator = std::make_unique<Integrator<iDynTree::Vector3>>(m_dT); */
    /*     m_baseRotationIntegrator = std::make_unique<Integrator<iDynTree::Matrix3x3>>(m_dT); */


    /*     // resize quantities */
    /*     m_jointPosition.resize(m_numberOfDoF); */
    /*     m_jointVelocity.resize(m_numberOfDoF); */
    /*     m_jointAcceleration.resize(m_numberOfDoF); */
    /*     m_generalizedJointTorques.resize(m_numberOfDoF + 6); */

    /*     m_massMatrix.resize(m_numberOfDoF + 6, m_numberOfDoF + 6); */
    /*     m_massMatrixInverse.resize(m_numberOfDoF + 6, m_numberOfDoF + 6); */

    /*     m_generalizedBiasForces.resize(model); */

    /*     m_leftContact.jacobian.resize(6, m_numberOfDoF + 6); */
    /*     m_rightContact.jacobian.resize(6, m_numberOfDoF + 6); */

    /*     m_gravity.zero(); */
    /*     m_gravity(2) = -9.81; */

    /* } */

    template <class Derived>
    bool initialize(std::weak_ptr<ParametersHandler::IParametersHandler<Derived>> handler);

    bool reset(const iDynTree::VectorDynSize& initialJointValues,
               const iDynTree::Transform& leftFootTransform,
               const iDynTree::Transform& rightFootTransform);

    bool advance(const double& seconds = 0);

    iDynTree::Wrench leftWrench();
    iDynTree::Wrench rightWrench();

    const iDynTree::VectorDynSize& jointPositions() const;
    const iDynTree::VectorDynSize& jointVelocities() const;

    const iDynTree::Transform& baseTransform() const;
    const iDynTree::Twist& baseVelocity() const;

    void setLeftFootNullForceTransform(const iDynTree::Transform& transform);

    void setRightFootNullForceTransform(const iDynTree::Transform& transform);

    void setLeftFootState(bool isInContact);
    void setRightFootState(bool isInContact);

    bool setTorqueReferences(const iDynTree::VectorDynSize& torques);
    bool setAccelerationReferences(const iDynTree::VectorDynSize& acceleration);
};
} // namespace Simulator
} // namespace BipedalLocmotionControllers

#include "Simulator.tpp"

#endif // BIPEDAL_LOCOMOTION_CONTROLLERS_SIMULATOR_SIMULATOR_H
