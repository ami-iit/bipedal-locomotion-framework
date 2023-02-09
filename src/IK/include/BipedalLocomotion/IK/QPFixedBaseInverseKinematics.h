/**
 * @file QPFixedBaseInverseKinematics.h
 * @authors Giulio Romualdi
 * @copyright 2023 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the BSD-3-Clause license.
 */

#ifndef BIPEDAL_LOCOMOTION_IK_QP_FIXED_BASE_INVERSE_KINEMATICS_H
#define BIPEDAL_LOCOMOTION_IK_QP_FIXED_BASE_INVERSE_KINEMATICS_H

#include <memory>
#include <optional>
#include <functional>

#include <BipedalLocomotion/IK/IntegrationBasedIK.h>
#include <BipedalLocomotion/IK/QPInverseKinematics.h>
#include <BipedalLocomotion/ParametersHandler/IParametersHandler.h>
#include <BipedalLocomotion/System/VariablesHandler.h>

#include <iDynTree/KinDynComputations.h>

namespace BipedalLocomotion
{

namespace IK
{

/**
 * QPFixedBaseInverseKinematics is specialization of QPInverseKinematics class in the case of fixed
 * base system. The IK is here implemented as Quadratic Programming (QP) problem. The user should
 * set the desired task with the method QPFixedBaseInverseKinematics::addTask. Each task has a given
 * priority. Currently we support only priority equal to 0 or 1. If the task priority is set to 0
 * the task will be considered as a hard task, thus treated as a constraint. If the priority
 * is equal to 1 the task will be embedded in the cost function. The class is also able to treat
 * inequality constraints. Note that this class considers just one contact wrench as we assume the
 * external wrench acting on only the base link.
 * Here you can find an example of the QPFixedBaseInverseKinematics class used as velocity
 * controller or IK
 * @subsection vc Velocity Control
 * Here you can find an example of the QPFixedBaseInverseKinematics interface used as
 * a velocity controller.
 * <br/>
 * <img src="https://user-images.githubusercontent.com/16744101/142453785-9e6f2b5e-dc82-417a-a5e3-bc8c61865d0b.png" alt="VelocityControl" width="1500">
 * @subsection ik Inverse Kinematics
 * If you want to use IntegrationBasedInverseKinematics as IK you need to integrate the output
 * velocity. System::FloatingBaseSystemKinematics and System::Integrator classes can be used
 * to integrate the output of the IK taking into account the geometrical structure of the
 * configuration space (\f$ \mathbb{R}^3 \times SO(3) \times \mathbb{R}^n\f$)
 * <br/>
 * <img src="https://user-images.githubusercontent.com/16744101/142453860-6bba2a7a-26af-48da-b04e-114314c6f67c.png" alt="InverseKinematics" width="1500">
 * @note If you want to solve the Inverse Dynamics for a floating base system please use
 * QPInverseKinematics.
 */
class QPFixedBaseInverseKinematics : public QPInverseKinematics
{
    /**
     * Private implementation
     */
    struct Impl;
    std::unique_ptr<Impl> m_pimpl;

public:

    /**
     * Constructor.
     */
    QPFixedBaseInverseKinematics();

    /**
     * Destructor.
     */
    ~QPFixedBaseInverseKinematics();

    /**
     * Initialize the IK algorithm.
     * @param handler pointer to the IParametersHandler interface.h
     * @note the following parameters are required by the class
     * |            Parameter Name            |   Type   |                                             Description                                        | Mandatory |
     * |:------------------------------------:|:--------:|:----------------------------------------------------------------------------------------------:|:---------:|
     * |     `robot_velocity_variable_name`   | `string` | Name of the variable contained in `VariablesHandler` describing the generalized robot velocity |    Yes    |
     * |             `verbosity`              |  `bool`  |                        Verbosity of the solver. Default value `false`                          |     No    |
     * Where the generalized robot velocity is a vector containing the base spatialvelocity
     * (expressed in mixed representation) and the joint velocities.
     * @return True in case of success, false otherwise.
     */
    bool initialize(std::weak_ptr<const ParametersHandler::IParametersHandler> handler) override;

    /**
     * Set the kinDynComputations object.
     * @param kinDyn pointer to a kinDynComputations object.
     * @return True in case of success, false otherwise.
     */
    bool setKinDyn(std::shared_ptr<iDynTree::KinDynComputations> kinDyn);
};

} // namespace IK
} // namespace BipedalLocomotion

#endif // BIPEDAL_LOCOMOTION_IK_QP_FIXED_BASE_INVERSE_KINEMATICS_H
