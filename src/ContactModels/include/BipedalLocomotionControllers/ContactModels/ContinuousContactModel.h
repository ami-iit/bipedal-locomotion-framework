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
#include <BipedalLocomotionControllers/ParametersHandler/IParametersHandler.h>

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
    /** Homogeneous transformation of the frame placed in the center of the contact surface */
    iDynTree::Transform m_frameTransform{iDynTree::Transform::Identity()};

    /** Homogeneous transformation corresponding to a null contact wrench */
    iDynTree::Transform m_nullForceTransform{iDynTree::Transform::Identity()};

    /** Twist expressed in mixed representation of the frame placed in the center of the contact
     * surface */
    iDynTree::Twist m_twist{iDynTree::Twist::Zero()};

    double m_springCoeff{0.0}; /**< Spring coefficient associated to the model */
    double m_damperCoeff{0.0}; /**< Damper coefficient associated to the model */

    double m_length{0.0}; /**< Length of the rectangular contact surface */
    double m_width{0.0}; /**< Width of the rectangular contact surface */

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
     * Initialization of the class. Please call this method before evaluating any other function
     * @param handler std::weak_ptr to a parameter container. This class does not have the ownership
     * of the container.
     * @note The following parameters are required.
     *   - \a length (double): length in meters associated to the model
     *   - \a width (double): width in meters associated to the model
     *   - \a spring_coeff (double): spring coefficient associated to the model
     *   - \a damper_coeff (double): damper coefficient associated to the model
     * @return true/false in case of success/failure
     */
    bool initializePrivate(std::weak_ptr<ParametersHandler::IParametersHandler> handler) final;

    /**
     * Set the internal state of the model.
     * @param twist spatial velocity (expressed in mixed representation) of the link
     * @param transform transformation between the link and the inertial frame
     * @param nullForceTransform transformation corresponding to a null force expressed w.r.t. the
     * inertial frame
     */
    void setStatePrivate(const iDynTree::Twist& twist,
                         const iDynTree::Transform& transform,
                         const iDynTree::Transform& nullForceTransform) final;

public:

    /**
     * Constructor
     */
    ContinuousContactModel();

    /**
     * Compute the contact force applied by the environment on the system in a particular point of
     * the contact surface.
     * @param x x-coordinate of the point expressed in a frame placed in the contact surface whose
     * origin is the center of the surface [in meters]
     * @param y y-coordinate of the point expressed in a frame placed in the contact surface whose
     * origin is the center of the surface [in meters]
     */
    iDynTree::Force getForceAtPoint(const double& x, const double& y);

    /**
     * Compute the torque applied by the environment on the system about the origin of the frame
     * place in the contact surface whose origin is the center of the surface generated by the force
     * acting on the point (x, y)
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
