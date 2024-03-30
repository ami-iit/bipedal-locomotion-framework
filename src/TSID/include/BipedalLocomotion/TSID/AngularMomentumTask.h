/**
 * @file AngularMomentumTask.h
 * @authors Giulio Romualdi
 * @copyright 2021 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the BSD-3-Clause license.
 */

#ifndef BIPEDAL_LOCOMOTION_TSID_ANGULAR_MOMENTUM_TASK_H
#define BIPEDAL_LOCOMOTION_TSID_ANGULAR_MOMENTUM_TASK_H

#include <vector>
#include <memory>

#include <Eigen/Dense>

#include <BipedalLocomotion/System/VariablesHandler.h>
#include <BipedalLocomotion/TSID/TSIDLinearTask.h>

#include <LieGroupControllers/ProportionalController.h>

#include <iDynTree/KinDynComputations.h>


namespace BipedalLocomotion
{
namespace TSID
{

/**
 * AngularMomentumTask is a concrete implementation of the TSIDLinearTask. Please use this element
 * if you want to control the Centroidal angular momentum of the robot.
 * The task represents the following equation
 * \f[
 * \sum \begin{bmatrix} S(p_{C_k} - x & I_{3 \times 3} \end{bmatrix} f_k  = \dot{h} ^ * _\omega
 * \f]
 * where the suffix \f$S\f$ is the skew operator. \f$p_{C_k}\f$ is the position of the contact
 * \f$k\f$, \f$x\f$ is the CoM position, \f$f_k\f$ is the contact wrench expressed in mixed
 * representation. Finally \f$\dot{h} ^ * _\omega\f$ is the desired rate of change of the angular
 * momentum. The angular momentum rate of change is computed by a Proportional controller in
 * \f$\mathbb{R}^3\f$.
 */
class AngularMomentumTask : public TSIDLinearTask
{
    struct ContactWrench
    {
        iDynTree::FrameIndex frameIndex; /**< Frame used to express the contact wrench */
        System::VariablesHandler::VariableDescription variable; /**< Variable describing the contact
                                                                   wrench */
    };

    std::vector<ContactWrench> m_contactWrenches; /**< List of the information related to the
                                                     contact wrenches */

    static constexpr std::size_t m_angularMomentumSize{3}; /**< Size of the angular momentum */

    bool m_isInitialized{false}; /**< True if the task has been initialized. */
    bool m_isValid{false}; /**< True if the task is valid. */

    std::shared_ptr<iDynTree::KinDynComputations> m_kinDyn; /**< Pointer to a KinDynComputations
                                                               object */

    LieGroupControllers::ProportionalControllerR3d m_R3Controller; /**< Controller in R3 */

public:
    /**
     * Initialize the AngularMomentumTask.
     * @param paramHandler pointer to the parameters handler.
     * @note the following parameters are required by the class
     * |           Parameter Name           |   Type   |                                                    Description                                                   | Mandatory |
     * |:----------------------------------:|:--------:|:----------------------------------------------------------------------------------------------------------------:|:---------:|
     * |               `kp`                 | `double` |                              Proportional gain of the angular momentum controller.                               |    Yes    |
     * |      `max_number_of_contacts`      |   `int`  |                               Maximum number of contacts. The default value is 0.                                |     No    |
     * |            `CONTACT_<i>`           |  `group` | `i` is an `int` between `0` and `max_number_of_contacts` The group must contain `variable_name` and `frame_name` |    Yes    |
     * |           `variable_name`          | `string` |                    Name of the variable contained in `VariablesHandler` describing the contact                   |    Yes    |
     * |            `frame_name`            | `string` |                                    Name of the frame associated to the contact                                   |    Yes    |
     *
     * @return True in case of success, false otherwise.
     */
    bool initialize(std::weak_ptr<const ParametersHandler::IParametersHandler> paramHandler) override;

    /**
     * Set the kinDynComputations object.
     * @param kinDyn pointer to a kinDynComputations object.
     * @return True in case of success, false otherwise.
     */
    bool setKinDyn(std::shared_ptr<iDynTree::KinDynComputations> kinDyn) override;

    /**
     * Set the set of variables required by the task. The variables are stored in the
     * System::VariablesHandler.
     * @param variablesHandler reference to a variables handler.
     * @note The handler must contain variables named as the parameter `variable_name`
     * (in `CONTACT_<i>` group) and stored in the parameter handler.
     * @return True in case of success, false otherwise.
     */
    bool setVariablesHandler(const System::VariablesHandler& variablesHandler) override;

    /**
     * Update the content of the element.
     * @return True in case of success, false otherwise.
     */
    bool update() override;

    /**
     * Set the desired reference point.
     * @param angularMomentum desired centroidal angular momentum.
     * @param angularMomentumDerivative desired rate of change of the centroidal angular momentum.
     * @return True in case of success, false otherwise.
     */
    bool setSetPoint(Eigen::Ref<const Eigen::Vector3d> angularMomentum,
                     Eigen::Ref<const Eigen::Vector3d> angularMomentumDerivative);

    /**
     * Get the size of the task. (I.e the number of rows of the vector b)
     * @return the size of the task.
     */
    std::size_t size() const override;

    /**
     * The AngularMomentumTask is an equality task.
     * @return the type of the task.
     */
    Type type() const override;

    /**
     * Determines the validity of the objects retrieved with getA() and getB()
     * @return True if the objects are valid, false otherwise.
     */
    bool isValid() const override;
};

BLF_REGISTER_TSID_TASK(AngularMomentumTask);

} // namespace TSID
} // namespace BipedalLocomotion

#endif // BIPEDAL_LOCOMOTION_TSID_ANGULAR_MOMENTUM_TASK_H
