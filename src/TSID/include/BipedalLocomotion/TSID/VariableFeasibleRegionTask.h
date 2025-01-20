/**
 * @file VariableFeasibleRegionTask.h
 * @authors Roberto Mauceri
 * @copyright 2025 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the BSD-3-Clause license.
 */

#ifndef BIPEDAL_LOCOMOTION_TSID_VARIABLE_FEASIBLE_REGION_TASK_H
#define BIPEDAL_LOCOMOTION_TSID_VARIABLE_FEASIBLE_REGION_TASK_H

#include <BipedalLocomotion/TSID/TSIDLinearTask.h>

namespace BipedalLocomotion
{
namespace TSID
{
/**
 * VariableFeasibleRegionTask is a concrete implementation of the Task. Please use this element if
 * you want to create a inequality linear constraint on a subset of the optimization variable. The
 * task represents the following equation:
 * \f[
 * l \ne Cx \ne u
 * \f]
 * where \f$C\f$ is a generic transformation matrix (m x n), \f$l\f$ is a vector of lower bounds (m
 * x 1), \f$u\f$ is a vector of upper bounds (m x 1), and \f$x\f$ are the elements of the variable
 * you want to consider (n x 1). \f$m\f$ is the number of constraints and \f$n\f$ is the number of
 * variables.
 */
class VariableFeasibleRegionTask : public TSIDLinearTask
{
    std::string m_variableName; /**< Name of the variable considered in the task. */
    std::vector<std::string> m_controlledElements; /**< Name of the variable elements considered in
                                                      the task. */
    bool m_isInitialized{false}; /**< True if the task has been initialized. */
    bool m_isValid{false}; /**< True if the task is valid. */
    std::size_t m_NumberOfVariables{0}; /**< Number of variables. */
    std::size_t m_variableSize{0}; /**< Size of the variable considered in the task. */

    Eigen::MatrixXd m_S; /**< Selection Matrix. */
    Eigen::MatrixXd m_T; /**< Transformation Matrix. */

public:
    // clang-format off
    /**
     * Initialize the planner.
     * @param paramHandler pointer to the parameters handler.
     * @param variablesHandler reference to a variables handler.
     * @note the following parameters are required by the class
     * |  Parameter Name  |   Type   |                                   Description                                          | Mandatory |
     * |:----------------:|:--------:|:--------------------------------------------------------------------------------------:|:---------:|
     * | `variable_name`  | `string` | Name of the variable that you want to regularize.                                      |    Yes    |
     * | `variable_size`  |   `int`  | Number of the elements that will be regularized.                                       |    Yes    |
     * | `elements_name`  | `vector` | Name of the elements to consider. If not specified all the elements are constrained    |    No     |
     * @return True in case of success, false otherwise.
     */
    bool initialize(std::weak_ptr<const ParametersHandler::IParametersHandler> paramHandler) override;
    // clang-format on

    /**
     * Set the set of variables required by the TSIDLinearTask. The variables are stored in the
     * System::VariablesHandler.
     * @param variablesHandler reference to a variables handler.
     * @note The variablesHandler must contain a variable named as the parameter `variable_name`.
     * @return True in case of success, false otherwise.
     */
    bool setVariablesHandler(const System::VariablesHandler& variablesHandler) override;

    /**
     * Set the region of feasibility for the desired elements of the variable.
     * @param C generic linear transformation matrix (m x n) where m is the number of constraints
     * and n is the number of variables.
     * @param l lower_bounds (m x 1)
     * @param u upper_bounds (m x 1)
     * @return True in case of success, false otherwise.
     */
    bool setFeasibleRegion(Eigen::Ref<const Eigen::MatrixXd> C,
                           Eigen::Ref<const Eigen::VectorXd> l,
                           Eigen::Ref<const Eigen::VectorXd> u);

    /**
     * Get the size of the task. (I.e the number of rows of the vector b)
     * @return the size of the task.
     */
    std::size_t size() const override;

    /**
     * The VariableFeasibleRegionTask is an inequality task.
     * @return the type of the task.
     */
    Type type() const override;

    /**
     * Determines the validity of the objects retrieved with getA() and getB()
     * @return True if the objects are valid, false otherwise.
     */
    bool isValid() const override;
};

} // namespace TSID
} // namespace BipedalLocomotion

#endif // BIPEDAL_LOCOMOTION_TSID_VARIABLE_FEASIBLE_REGION_TASK_H
