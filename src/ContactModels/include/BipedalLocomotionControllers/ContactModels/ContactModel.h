/**
 * @file ContactModel.h
 * @authors Giulio Romualdi
 * @copyright 2020 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the GNU Lesser General Public License v2.1 or any later version.
 */

#ifndef BIPEDAL_LOCOMOTION_CONTROLLERS_CONTACT_MODELS_CONTACT_MODEL_H
#define BIPEDAL_LOCOMOTION_CONTROLLERS_CONTACT_MODELS_CONTACT_MODEL_H

#include <any>
#include <unordered_map>
#include <string>

#include <iDynTree/Core/MatrixFixSize.h>
#include <iDynTree/Core/VectorFixSize.h>
#include <iDynTree/Core/Wrench.h>

namespace BipedalLocomotionControllers
{
namespace ContactModels
{
/**
 * ContactModel is a generic implementation of a contact model. It computes the contact wrench
 * between the robot and the environments
 */
class ContactModel
{
protected:
    iDynTree::Wrench m_contactWrench; /**< Contact wrench between the robot and the environment
                                         expressed in mixed representation */

    /** Autonomous dynamics of the contact model rate of change (i.e. given a non linear system\f$
     * \dot{x} = f + g u\f$ the autonomous is \a f */
    iDynTree::Vector6 m_autonomousDynamics;

    /** Autonomous dynamics of the contact model rate of change (i.e. given a non linear system\f$
     * \dot{x} = f + g u\f$ the autonomous is \a g */
    iDynTree::Matrix6x6 m_controlMatrix;

    bool m_isContactWrenchComputed; /**< If true the contact wrench has been already computed */
    bool m_isAutonomusDynamicsComputed; /**< If true the autonomous dynamics has been already computed */
    bool m_isControlMatrixComputed; /**< If true the controllers matrix has been already computed */

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
     * Get variable from a set of variables
     * @param variables map containing variables
     * @param variableName name of the variable
     * @param variable variable
     * @return true/false in case of success/failure
     */
    template <class T>
    bool getVariable(const std::unordered_map<std::string, std::any>& variables,
                     const std::string& variableName,
                     T& variable);

    /**
     * Set the immutable parameters
     */
    virtual void setImmutableParameters(const std::unordered_map<std::string, std::any>& parameters) = 0;

public:
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
     * Set the internal state of the model
     */
    virtual void setState(const std::unordered_map<std::string, std::any>& state) = 0;

    /**
     * Set the mutable parameters
     */
    virtual void setMutableParameters(const std::unordered_map<std::string, std::any>& parameters) = 0;
};
} // namespace ContactModels
} // namespace BipedalLocomotionControllers


#include "ContactModel.tpp"

#endif // BIPEDAL_LOCOMOTION_CONTROLLERS_CONTACT_MODELS_CONTACT_MODEL_H
