/**
 * @file ContinuousContactModel.h
 * @authors Giulio Romualdi
 * @copyright 2020 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the GNU Lesser General Public License v2.1 or any later version.
 */

#ifndef BIPEDAL_LOCOMOTION_CONTROLLERS_CONTACT_MODELS_CONTINUOUS_CONTACT_MODEL_H
#define BIPEDAL_LOCOMOTION_CONTROLLERS_CONTACT_MODELS_CONTINUOUS_CONTACT_MODEL_H

#include <iDynTree/Core/Transform.h>
#include <iDynTree/Core/Twist.h>
#include <iDynTree/Core/Wrench.h>

#include <BipedalLocomotionControllers/ContactModels/ContactModel.h>

namespace BipedalLocomotionControllers
{
namespace ContactModels
{
/**
 * ContinuousContactModel is a model that describe the contact using a continuous representation. It
 * is an extension of the brush model used for describing the contact between the tire and the
 * ground. Each point in the contact surface is subjected to an infinitesimal force \f$ f \f$ given
 * by \f$ f=k(x_0 - x)- b x\f$, where \f$k\f$ is the spring coefficient and \f$b\f$ is the damper
 * coefficient. \f$x\f$ and \f$x_0\f$ are, respectively, the position of the point placed in the
 * contact surface and the point con corresponding to a null contact force written in the inertial
 * frame. Furthermore, during the contact scenario, we assume the link acts as a rigid body. While
 * the environment will deform. The ground characteristics are isotropic, and it can be approximated
 * as a continuum of springs and dampers.
 * The contact surface is supposed to be <b>rectangular</b> and the frame placed on the contact
 * surfaced is centered in the middle of the surface, with the \a z-axis pointing upwards and the \a
 * x-axis pointing forward (parallel to one edge of the rectangle).
 */
class ContinuousContactModel final : public ContactModel
{
    iDynTree::Transform m_frameTransform; /**< Homogeneous transformation of the frame placed in the
                                             center of the contact surface */
    iDynTree::Transform m_nullForceTransform; /**< Homogeneous transformation corresponding to a
                                                 null contact wrench */
    iDynTree::Twist m_twist; /**< Twist expressed in mixed representation of the frame placed in the
                                center of the contact surface */

    double m_springCoeff; /**< Spring coefficient associated to the model */
    double m_damperCoeff; /**< Damper coefficient associated to the model */

    double m_length; /**< Length of the rectangular contact surface */
    double m_width; /**< Width of the rectangular contact surface */

    /**
     * Evaluate the contact wrench given a specific contact model
     */
    void computeContactWrench() final;

    /**
     * Evaluate the Autonomous System Dynamics of the derivative of a specific contact model
     */
    void computeAutonomousDynamics() final;

    /**
     * Evaluate the Control Matrix of the derivative of a specific contact model
     */
    void computeControlMatrix() final;

    /**
     * Set the parameters cannot change
     * @param state std::unordered_map containing the mutable parameters of the system. To compute the contact
     * wrench between two system (i.e. a link of a robot and the environment) the following data has
     * to be available. If not an std::runtime_error is thrown.
     *   - \a length (double): length in meters associated to the model
     *   - \a width (double): width in meters associated to the model
     */
    void setImmutableParameters(const std::unordered_map<std::string, std::any>& parameters) final;

public:

    /**
     * Constructor. It instantiate the value of the mutable parameters and the immutable parameters.
     * @param immutableParameters std::unordered_map containing the immutable parameters of the
     * system. The list of the required parameters could be found in @ref
     * BipedalLocomotionController::ContactModels::setImmutableparameters
     * @param mutableParameters std::unordered_map containing the mutable parameters of the
     * system. The list of the required parameters could be found in @ref
     * BipedalLocomotionController::ContactModels::setMutableparameters
     */
    ContinuousContactModel(const std::unordered_map<std::string, std::any>& immutableParameters,
                           const std::unordered_map<std::string, std::any>& mutableParameters);

    /**
     * Constructor. It instantiate the value of the mutable parameters and the immutable parameters.
     * @param parameters std::unordered_map containing the immutable and mutable parameters of the
     * system. The list of the required parameters could be found in @ref
     * BipedalLocomotionController::ContactModels::setMutableparameters
     */
    ContinuousContactModel(const std::unordered_map<std::string, std::any>& parameters);


    /**
     * Set the internal state of the model.
     * @param state std::unordered_map containing the state of the system. To compute the contact
     * wrench between two system (i.e. a link of a robot and the environment) the following data has
     * to be available. If not an std::runtime_error is thrown.
     *   - \a frame_transform (iDynTree::Transform): transformation between the link and the inertial frame;
     *   - \a null_force_transform (iDynTree::Transform): transformation corresponding to a null force;
     *   - \a twist (iDynTree::Twist): twist (expressed in mixed representation) of the link
     */
    void setState(const std::unordered_map<std::string, std::any>& state) final;

    /**
     * Set the parameters may depends on time/state (i.e. they are not considered constant)
     * @param state std::unordered_map containing the mutable parameters of the system. To compute the contact
     * wrench between two system (i.e. a link of a robot and the environment) the following data has
     * to be available. If not an std::runtime_error is thrown.
     *   - \a spring_coeff (double): spring coefficient associated to the model
     *   - \a damper_coeff (double): damper coefficient associated to the model
     */
    void setMutableParameters(const std::unordered_map<std::string, std::any>& parameters) final;

    /**
     * Compute the force in a particular point in the contact surface.
     * @param x x-coordinate of the point expressed in a frame placed in the contact surface whose
     * origin is the center of the surface [in meters]
     * @param y y-coordinate of the point expressed in a frame placed in the contact surface whose
     * origin is the center of the surface [in meters]
     */
    iDynTree::Force getForceAtPoint(const double& x, const double& y);

    /**
     * Compute the torque about the origin of the frame place in the contact surface whose
     * origin is the center of the surface generated by the force acting on the point (x, y)
     * @param x x-coordinate of the point expressed in a frame placed in the contact surface whose
     * origin is the center of the surface [in meters]
     * @param y y-coordinate of the point expressed in a frame placed in the contact surface whose
     * origin is the center of the surface [in meters]
     */
    iDynTree::Torque getTorqueGeneratedAtPoint(const double& x, const double& y);
};
} // namespace ContactModels
} // namespace BipedalLocomotionControllers

#endif // BIPEDAL_LOCOMOTION_CONTROLLERS_CONTACT_MODELS_CONTINUOUS_CONTACT_MODEL_H
