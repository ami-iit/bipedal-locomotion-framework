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
#include <iDynTree/Model/FreeFloatingState.h>
#include <iDynTree/Model/Model.h>

#include <BipedalLocomotionControllers/ContactModels/ContinuousContactModel.h>
#include <BipedalLocomotionControllers/ParametersHandler/IParametersHandler.h>
#include <BipedalLocomotionControllers/Simulator/Integrator.h>

namespace BipedalLocomotionControllers
{
namespace Simulator
{
/**
 * Simple implementation of a simulator for walking on compliant contacts
 */
class Simulator
{
    /**
     * Contact model
     */
    struct Contact
    {
        /** Pointer to a continuous contact model */
        std::unique_ptr<ContactModels::ContinuousContactModel> model;
        /** Index of the frame in contact with the environment in the model */
        iDynTree::FrameIndex indexInTheModel;
        /** Homogeneous transformation I_T_B (it converts a vector expressed in B in a vector
         * expressed in the inertial frame) such that the resulting contact wrench is zero */
        iDynTree::Transform frameNullForce;
        /** Jacobian matrix related to the contact */
        iDynTree::MatrixDynSize jacobian;
        /** Flag used to consider if a frame is in contact */
        bool isInContact{true};
    };

    /**
     * Dictionary used to consider the frames that can be used as base frames
     */
    std::unordered_map<std::string, std::pair<const std::string, const iDynTree::Transform>>
        m_baseFrames;

    // TODO move from here
    double m_length, m_width, m_springCoeff, m_damperCoeff;


    std::size_t m_numberOfDoF; /**< Number of actuated DoFs */
    double m_totalMass; /**< Total mass of the robot expressed in Kg */
    double m_dT; /**< Sampling time of the simulator expressed in seconds */

    Contact m_leftContact; /**< Model of the left foot contact */
    Contact m_rightContact; /**< Model of the right foot contact */

    iDynTree::KinDynComputations m_kinDyn;

    iDynTree::VectorDynSize m_jointAcceleration; /**< Joints acceleration expressed in rad/s^2 */
    iDynTree::VectorDynSize m_jointVelocity; /**< Joints velocity expressed in rad/s */
    iDynTree::VectorDynSize m_jointPosition; /**< Joints position expressed in rad */

    iDynTree::SpatialAcc m_baseAcceleration; /**< Base acceleration expressed in mixed
                                                representation */
    iDynTree::Twist m_baseTwist; /**< Base velocity expressed in mixed representation */
    iDynTree::Transform m_baseTransform; /**< Homogeneous transformation of the base frame w.r.t.
                                            the inertial frame */

    iDynTree::VectorDynSize m_generalizedJointTorques; /**< Vector containing the generalized hoint
                                                          torques. The length of the vector is ^ +
                                                          actuated degrees of freedom. The first six
                                                          components are always zero. The others are
                                                          the joint torques  */

    iDynTree::MatrixDynSize m_massMatrix; /**< Mass Matrix */
    iDynTree::FreeFloatingGeneralizedTorques m_generalizedBiasForces; /**< Generalized bias forces
                                                                         containing the Coriolis and
                                                                         the gravitational terms */

    /** Integrator used to compute the joints velocity given the joints acceleration */
    std::unique_ptr<Integrator<iDynTree::VectorDynSize>> m_jointVelocityIntegrator;
    /** Integrator used to compute the joints position given the joints velocity */
    std::unique_ptr<Integrator<iDynTree::VectorDynSize>> m_jointPositionIntegrator;

    /** Integrator used to compute the base linear velocity given the base linear acceleration */
    std::unique_ptr<Integrator<iDynTree::LinearMotionVector3>> m_baseLinearVelocityIntegrator;
    /** Integrator used to compute the base angular velocity given the base angular acceleration */
    std::unique_ptr<Integrator<iDynTree::AngularMotionVector3>> m_baseAngularVelocityIntegrator;

    /** Integrator used to compute the base position  given the base linear velocity */
    std::unique_ptr<Integrator<iDynTree::Vector3>> m_basePositionIntegrator;
    /** Integrator used to compute the base rotation matrix  given the base rotation matrix rate of
     * change */
    std::unique_ptr<Integrator<iDynTree::Matrix3x3>> m_baseRotationIntegrator;

    /** Vector containing the gravity g = [0 0 -9.81] */
    iDynTree::Vector3 m_gravity;

    /**
     * Collection of the control mode used to control the system
     */
    enum class ControlMode
    {
        Velocity, /**< The user can only set the desired generalized velocity (joint + base) in
                     mixed representation */
        Acceleration, /**< The user can only set the desired generalized acceleration (joint + base)
                         in mixed representation */
        Torque /**< The user can only set the joint torque in Nm */
    };

    /**
     * Simple state machine for handling the status of the simulator
     */
    enum class State
    {
        NotInitialized, /**< The simulator is not initialized. To initialize it please call
                           Simulator::initialize */
        Initialized, /**< The simulator is now initialized. Before run the simulation please call
                        the Simulator::reset method */
        Ready, /**< The simulator is ready to be used */
        Running /**< The simulator is running without any problem */
    };

    State m_state{State::NotInitialized}; /**< Current state of the simulator */
    ControlMode m_controlMode; /**< Desired control mode used */

    /**
     * Add the base frame in m_baseFrames dictionary.
     * @param frameBaseIndex index of the frame in the model
     * @param name label used to identify the frame
     * @return true/false in case of success/failure
     */
    bool setBaseFrame(const iDynTree::FrameIndex& frameBaseIndex, const std::string& name);

public:
    /**
     * Create a new simulator from a given model
     * @param model model of the robot
     */
    Simulator(const iDynTree::Model& model);

    /**
     * Initialize the simulator with the parameters
     * @note To initialize the simulator the following parameters are mandatory:
     * - length: length (in meters) of the foot
     * - width: width (in meters) of the foot
     * - spring_coeff: coefficient of the spring used to model the contact
     * - damper_coeff: coefficient of the damper used to model the contact
     * - left_foot_frame: name in the model of the left foot frame (i.e. l_sole) The frame should be
    placed in the center of the foot
    * - right_foot_frame: name in the model of the left foot frame (i.e. r_sole) The frame should be
    placed in the center of the foot
    * - base_frame: name of the base frame in the model (i.e. root_link)
    * - sampling_time: integration step of the simulator
    * - control_mode: control mode used to control the system. The supported control mode are:
    "torque", "acceleration" and "velocity"
    * @param handler parameter container
    * @return true/false in case of success/failure
    */
    template <class Derived>
    bool initialize(std::weak_ptr<ParametersHandler::IParametersHandler<Derived>> handler);

    /**
     * Reset the simulator.
     * @note When the simulator is reset, we assume that the robot is in equilibrium with zero
     * velocity and acceleration
     * @param initialJointValues initial position of the joint expressed in rad
     * @param leftFootTransform homogeneous transformation of the left foot w.r.t. the inertial
     * frame
     * @param rightFootTransform homogeneous transformation of the left foot w.r.t. the inertial
     * frame
     * @return true/false in case of success/failure
     */
    bool reset(const iDynTree::VectorDynSize& initialJointValues,
               const iDynTree::Transform& leftFootTransform,
               const iDynTree::Transform& rightFootTransform);

    /**
     * Advance the simulator for a certain amount of seconds
     * @param seconds number of seconds that the controller should advance. If seconds is equal to
     * zero it will advance of only one step.
     * @return true/false in case of success/failure
     */
    bool advance(const double& seconds = 0);

    /**
     * Contact wrench exerted by the environment on the left foot
     * @return left wrench
     */
    iDynTree::Wrench leftWrench();

    /**
     * Contact wrench exerted by the environment on the right foot
     * @return right wrench
     */
    iDynTree::Wrench rightWrench();

    /**
     * Get the joints position expressed in radians
     * @return the joint position
     */
    const iDynTree::VectorDynSize& jointPositions() const;

    /**
     * Get the joints velocity expressed in radians per seconds
     * @return the joint velocity
     */
    const iDynTree::VectorDynSize& jointVelocities() const;

    /**
     * Get the base transformation
     * @return base transform w.r.t. the inertial frame
     */
    const iDynTree::Transform& baseTransform() const;

    /**
     * Get the base velocity expressed in mixed representation
     * @return base velocity
     */
    const iDynTree::Twist& baseVelocity() const;

    /**
     * Set the left foot transform such that the contact wrench will be null
     * @param transform transform of the left foot w.r.t. the inertial frame
     */
    void setLeftFootNullForceTransform(const iDynTree::Transform& transform);

    /**
     * Set the right foot transform such that the contact wrench will be null
     * @param transform transform of the right foot w.r.t. the inertial frame
     */
    void setRightFootNullForceTransform(const iDynTree::Transform& transform);

    /**
     * Set the state of the left foot
     * @param isInContact true if the left foot is in contact
     */
    void setLeftFootState(bool isInContact);

    /**
     * Set the state of the right foot
     * @param isInContact true if the right foot is in contact
     */
    void setRightFootState(bool isInContact);

    /**
     * Set the reference for the joint torques
     * @param torques joint torques expressed in Nm
     * @return true/false in case of success/failure
     */
    bool setTorqueReferences(const iDynTree::VectorDynSize& torques);

    /**
     * Set the reference for the generalized acceleration
     * @param acceleration vector containing the base acceleration and the joint acceleration
     * @return true/false in case of success/failure
     */
    bool setAccelerationReferences(const iDynTree::VectorDynSize& acceleration);

    /**
     * Set the reference for the generalized velocity
     * @param velocity vector containing the base velocity and the joint velocity
     * @return true/false in case of success/failure
     */
    bool setVelocityReferences(const iDynTree::VectorDynSize& velocity);
};
} // namespace Simulator
} // namespace BipedalLocomotionControllers

#include "Simulator.tpp"

#endif // BIPEDAL_LOCOMOTION_CONTROLLERS_SIMULATOR_SIMULATOR_H
