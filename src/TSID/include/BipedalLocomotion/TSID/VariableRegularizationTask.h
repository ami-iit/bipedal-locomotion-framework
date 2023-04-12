/**
 * @file VariableRegularizationTask.h
 * @authors Giulio Romualdi
 * @copyright 2021 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the BSD-3-Clause license.
 */

#ifndef BIPEDAL_LOCOMOTION_TSID_VARIABLE_REGULARIZATION_TASK_H
#define BIPEDAL_LOCOMOTION_TSID_VARIABLE_REGULARIZATION_TASK_H

#include <BipedalLocomotion/TSID/TSIDLinearTask.h>

namespace BipedalLocomotion
{
namespace TSID
{

/**
 * VariableRegularizationTask is a concrete implementation of the Task. Please use this element if
 * you want to regularize a variable to a given value. The task represents the following equation
 * \f[
 * x = x_{des}
 * \f]
 * where \f$x\f$ are the elements of the variable you want to regularize to a given value.
 */
class VariableRegularizationTask : public TSIDLinearTask
{
    std::string m_variableName; /**< Name of the variable considered that will be regularized */
    std::vector<std::string> m_controlledElements; /**< Name of the variable elements considered in
                                                     the task */
    bool m_isInitialized{false}; /**< True if the task has been initialized. */
    bool m_isValid{false}; /**< True if the task is valid. */
    std::size_t m_variableSize{0}; /**< Size of the regularized variable. */

public:
    /**
     * Initialize the planner.
     * @param paramHandler pointer to the parameters handler.
     * @param variablesHandler reference to a variables handler.
     * @note the following parameters are required by the class
     * |  Parameter Name  |   Type   |                                   Description                                          | Mandatory |
     * |:----------------:|:--------:|:--------------------------------------------------------------------------------------:|:---------:|
     * | `variable_name`  | `string` |               Name of the variable that you want to regularize                         |    Yes    |
     * | `variable_size`  |   `int`  | Number of the elements that will be regularized. |    Yes     |
     * | `elements_name`  | `vector` | Name of the elements to consider. If not specified all the elements are regularize to the desired vector |    No     |
     * @return True in case of success, false otherwise.
     */
    bool initialize(std::weak_ptr<const ParametersHandler::IParametersHandler> paramHandler) override;

    /**
     * Set the set of variables required by the task. The variables are stored in the
     * System::VariablesHandler.
     * @param variablesHandler reference to a variables handler.
     * @note The variablesHandler must contain a variable named as the parameter `variable_name`.
     * @return True in case of success, false otherwise.
     */
    bool setVariablesHandler(const System::VariablesHandler& variablesHandler) override;

    /**
     * Set the desired setpoint.
     * @param setPoint vector containing the desired variable set point.
     * @return True in case of success, false otherwise.
     */
    bool setSetPoint(Eigen::Ref<const Eigen::VectorXd> setPoint);

    /**
     * Get the size of the task. (I.e the number of rows of the vector b)
     * @return the size of the task.
     */
    std::size_t size() const override;

    /**
     * The VariableRegularizationTask is an equality task.
     * @return the type of the task.
     */
    Type type() const override;

    /**
     * Determines the validity of the objects retrieved with getA() and getB()
     * @return True if the objects are valid, false otherwise.
     */
    bool isValid() const override;
};

BLF_REGISTER_TSID_TASK(VariableRegularizationTask);

} // namespace TSID
} // namespace BipedalLocomotion

#endif // BIPEDAL_LOCOMOTION_TSID_VARIABLE_REGULARIZATION_TASK_H
