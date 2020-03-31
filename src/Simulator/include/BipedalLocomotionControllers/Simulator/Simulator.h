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
        Velocity,
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

    bool initialize(std::weak_ptr<ParametersHandler::IParametersHandler> handlerWeak);

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
    bool setVelocityReferences(const iDynTree::VectorDynSize& velocity);
};
} // namespace Simulator
} // namespace BipedalLocmotionControllers

#endif // BIPEDAL_LOCOMOTION_CONTROLLERS_SIMULATOR_SIMULATOR_H
