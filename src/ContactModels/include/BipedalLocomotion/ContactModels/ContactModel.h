/**
 * @file ContactModel.h
 * @authors Giulio Romualdi
 * @copyright 2020 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the BSD-3-Clause license.
 */

#ifndef BIPEDAL_LOCOMOTION_CONTACT_MODELS_CONTACT_MODEL_H
#define BIPEDAL_LOCOMOTION_CONTACT_MODELS_CONTACT_MODEL_H

#include <any>
#include <memory>
#include <string>
#include <unordered_map>

#include <iDynTree/MatrixDynSize.h>
#include <iDynTree/MatrixFixSize.h>
#include <iDynTree/Transform.h>
#include <iDynTree/Twist.h>
#include <iDynTree/VectorFixSize.h>
#include <iDynTree/Wrench.h>

#include <BipedalLocomotion/ParametersHandler/IParametersHandler.h>

namespace BipedalLocomotion
{
namespace ContactModels
{
/**
 * ContactModel is a generic implementation of a contact model. It computes the contact wrench
 * between the robot and the environments.
 */
class ContactModel
{
    bool m_isContactWrenchComputed; /**< If true the contact wrench has been already computed */
    bool m_isAutonomousDynamicsComputed; /**< If true the autonomous dynamics has been already
                                            computed */
    bool m_isControlMatrixComputed; /**< If true the controllers matrix has been already computed */

    bool m_isRegressorComputed; /**< If true the regressor matrix has been already computed */

protected:
    iDynTree::Wrench m_contactWrench; /**< Contact wrench between the robot and the environment
                                         expressed in mixed representation */

    /** Autonomous dynamics of the contact model rate of change (i.e. given a non linear system
     * \f$\dot{x} = f + g u\f$ the autonomous dynamics is \a f */
    iDynTree::Vector6 m_autonomousDynamics;

    /** Control matrix of the contact model rate of change (i.e. given a non linear system
     * \f$\dot{x} = f + g u\f$ the control matrix is \a g */
    iDynTree::Matrix6x6 m_controlMatrix;

    /** Contains the regressor of the contact model. \f$f = A \theta\f$, where \f$f\f$ is the
     * contact wrench, \f$A\f$ the regressor and \f$\theta\f$ the parameters */
    iDynTree::MatrixDynSize m_regressor;

    /**
     * Evaluate the contact wrench given a specific contact model
     */
    virtual void computeContactWrench() = 0;

    /**
     * Evaluate the Autonomous System Dynamics of the derivative of a specific contact model
     */
    virtual void computeAutonomousDynamics() = 0;

    /**
     * Evaluate the Control Matrix of the derivative of a specific contact model
     */
    virtual void computeControlMatrix() = 0;

    /**
     * Evaluate the regressor matrix
     */
    virtual void computeRegressor() = 0;

    /**
     * Initialization of the class.
     * @param handler std::weak_ptr to a parameter container. This class does not have the ownership
     * of the container.
     * @note the required parameters may depends on the particular implementation. An example
     * can be found in BipedalLocomotionControllers::ContactModel::ContinuousContactmodel::initializePrivate
     * @return true/false in case of success/failure
     */
    virtual bool initializePrivate(std::weak_ptr<ParametersHandler::IParametersHandler> handler) = 0;

    /**
     * Set the internal state of the model.
     */
    virtual void setStatePrivate(const iDynTree::Twist& twist,
                                 const iDynTree::Transform& transform) = 0;

    /**
     * Set the null force transform.
     */
    virtual void setNullForceTransformPrivate(const iDynTree::Transform& transform) = 0;

public:
    /**
     * Initialization of the class. Please call this method before evaluating any other function
     * @param handler std::weak_ptr to a parameter container. This class does not have the ownership
     * of the container.
     * @warning std::weak_ptr models temporary ownership: when the handler is accessed only if it
     * exists, the std::weak_ptr is converted in a std::shared_ptr.
     * @note the required parameters may depends on the particular implementation. An example
     * can be found in BipedalLocomotionControllers::ContactModel::ContinuousContactmodel::initializePrivate
     * @return true/false in case of success/failure
     */
    bool initialize(std::weak_ptr<ParametersHandler::IParametersHandler> handler);

    /**
     * Get and compute (only if it is necessary) the contact wrench
     * @return the contact wrench expressed in mixed representation
     */
    const iDynTree::Wrench& getContactWrench();

    /**
     * Get and compute (only if it is necessary) the autonomous system dynamics
     * @return the autonomous system dynamics at a given state
     */
    const iDynTree::Vector6& getAutonomousDynamics();

    /**
     * Get and compute (only if it is necessary) the control matrix
     * @return the control matrix at a given state
     */
    const iDynTree::Matrix6x6& getControlMatrix();

    /**
     * Get and compute (only if it is necessary) the regressor
     * @return the regressor at a given state
     */
    const iDynTree::MatrixDynSize& getRegressor();

    /**
     * Set the internal state of the model.
     * @note the meaning of the parameters may depend on the particular implementation. An example
     * can be found in BipedalLocomotionControllers::ContactModel::ContinuousContactmodel::setState
     */
    void setState(const iDynTree::Twist& twist,
                  const iDynTree::Transform& transform);

    void setNullForceTransform(const iDynTree::Transform& transform);

};
} // namespace ContactModels
} // namespace BipedalLocomotion

#endif // BIPEDAL_LOCOMOTION_CONTACT_MODELS_CONTACT_MODEL_H
