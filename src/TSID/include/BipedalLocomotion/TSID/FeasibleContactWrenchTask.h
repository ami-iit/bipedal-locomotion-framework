/**
 * @file FeasibleContactWrenchTask.h
 * @authors Giulio Romualdi
 * @copyright 2021 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the BSD-3-Clause license.
 */

#ifndef BIPEDAL_LOCOMOTION_TSID_FEASIBLE_CONTACT_WRENCH_TASK_H
#define BIPEDAL_LOCOMOTION_TSID_FEASIBLE_CONTACT_WRENCH_TASK_H

#include <memory>

#include <manif/manif.h>

#include <BipedalLocomotion/TSID/TSIDLinearTask.h>
#include <BipedalLocomotion/Math/ContactWrenchCone.h>


#include <iDynTree/KinDynComputations.h>


namespace BipedalLocomotion
{
namespace TSID
{

/**
 * FeasibleContactWrenchTask is a concrete implementation of the TSIDLinearTask. Please use this
 * element if you want to ensure that the contact wrench satisfies the constraints implemented in
 * BipedalLocomotion::Math::ContactWrenchCone. Differently from the
 * BipedalLocomotion::Math::ContactWrenchCone class, FeasibleContactWrenchTask requires also the
 * positivstellensatz of the normal force written in local coordinate.
 */
class FeasibleContactWrenchTask : public TSIDLinearTask
{
    struct ContactWrench
    {
        iDynTree::FrameIndex frameIndex; /**< Frame used to express the contact wrench */
        System::VariablesHandler::VariableDescription variable; /**< Variable describing the contact
                                                                   wrench */
        Eigen::Matrix3d frame_R_inertial;
    };

    ContactWrench m_contactWrench;

    BipedalLocomotion::Math::ContactWrenchCone m_cone;

    bool m_isInitialized{false}; /**< True if the task has been initialized. */
    bool m_isValid{false}; /**< True if the task is valid. */

    Eigen::MatrixXd m_AinBodyCoordinate; /**< Matrix A written in body frame */

    std::shared_ptr<iDynTree::KinDynComputations> m_kinDyn; /**< Pointer to a KinDynComputations
                                                               object */
public:

    /**
     * Initialize the task.
     * @param handler pointer to the parameter handler.
     * @note The following parameters are required:
     * |         Parameter Name        |       Type       |                                  Description                                  | Mandatory |
     * |:-----------------------------:|:----------------:|:-----------------------------------------------------------------------------:|:---------:|
     * |        `frame_name`           |      `string`    |                  Name of the frame associated to the contact                  |    Yes    |
     * |       `variable_name`         |      `string`    | Name of the variable contained in `VariablesHandler` describing the contact   |    Yes    |
     * |       `number_of_slices`      |       `int`      |                     Number of slices used to split 90 deg.                    |    Yes    |
     * | `static_friction_coefficient` |      `double`    |                          Static friction coefficient.                         |    Yes    |
     * |        `foot_limits_x`        | `vector<double>` |     x coordinate of the foot limits w.r.t the frame attached to the surface   |    Yes    |
     * |        `foot_limits_y`        | `vector<double>` |     y coordinate of the foot limits w.r.t the frame attached to the surface   |    Yes    |
     * @return true in case of success/false otherwise.
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
     * @note The handler must contain a variable named as the parameter
     * `variable_name` stored in the parameter handler. The variable represents the contact wrench
     * acting of the robot.
     * @return True in case of success, false otherwise.
     */
    bool setVariablesHandler(const System::VariablesHandler& variablesHandler) override;

    /**
     * Update the content of the element.
     * @return True in case of success, false otherwise.
     */
    bool update() override;

    /**
     * Set if the contact is active or not.
     * @param isActive true if the contact is active, false otherwise.
     */
    void setContactActive(bool isActive);

    /**
     * Get the size of the task. (I.e the number of rows of the vector b)
     * @return the size of the task.
     */
    std::size_t size() const override;

    /**
     * The CoMTask is an equality task.
     * @return the size of the task.
     */
    Type type() const override;

    /**
     * Determines the validity of the objects retrieved with getA() and getB()
     * @return True if the objects are valid, false otherwise.
     */
    bool isValid() const override;
};

BLF_REGISTER_TSID_TASK(FeasibleContactWrenchTask);

} // namespace TSID
} // namespace BipedalLocomotion

#endif // BIPEDAL_LOCOMOTION_TSID_FEASIBLE_CONTACT_WRENCH_TASK_H
