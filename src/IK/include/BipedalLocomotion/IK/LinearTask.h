/**
 * @file LinearTask.h
 * @authors Giulio Romualdi
 * @copyright 2021 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the GNU Lesser General Public License v2.1 or any later version.
 */

#ifndef BIPEDAL_LOCOMOTION_IK_LINEAR_TASK_H
#define BIPEDAL_LOCOMOTION_IK_LINEAR_TASK_H

#include <memory>

#include <Eigen/Dense>

#include <iDynTree/Core/MatrixView.h>
#include <iDynTree/KinDynComputations.h>

#include <BipedalLocomotion/ParametersHandler/IParametersHandler.h>
#include <BipedalLocomotion/System/VariablesHandler.h>

namespace BipedalLocomotion
{
namespace IK
{

/**
 * LinearTask describes a IK LinearTask element. The LinearTask is described by a matrix
 * \f$A\f$ and a vector \f$b\f$. This class describes both a linear equality constraint and a linear
 * inequality constraint. In case of equality constraint \f$ A \f$ and \f$ b \f$ represents: \f$ Ax
 * = b\f$ In case of inequality constraint \f$ A \f$ and \f$ b \f$ represents: \f$ Ax \le b \f$
 * @note Please inherit this class if you want to build your own optimal control problem.
 */
class LinearTask
{
protected:
    Eigen::MatrixXd m_A; /**< Task Matrix */
    Eigen::VectorXd m_b; /**< Task Vector */

    std::string m_description{"Generic IK Task Element"}; /**< String describing the content of the
                                                             task */

    std::shared_ptr<iDynTree::KinDynComputations> m_kinDyn; /**< Pointer to a KinDynComputations
                                                               object */

    /**
     * Extract the submatrix A associated to a given variable.
     */
    iDynTree::MatrixView<double>
    subA(const System::VariablesHandler::VariableDescription& description);

    /**
     * Extract the submatrix A associated to a given variable.
     */
    iDynTree::MatrixView<const double>
    subA(const System::VariablesHandler::VariableDescription& description) const;

public:
    /**
     * Type of the task. Namely an equality or an inequality task.
     */
    enum class Type
    {
        equality,
        inequality
    };

    /**
     * Set the kinDynComputations object.
     * @param kinDyn pointer to a kinDynComputations object.
     * @return True in case of success, false otherwise.
     */
    bool setKinDyn(std::shared_ptr<iDynTree::KinDynComputations> kinDyn);

    /**
     * Set the set of variables required by the task. The variables are stored in the
     * System::VariablesHandler
     * @param variablesHandler reference to a variables handler.
     * @return True in case of success, false otherwise.
     */
    virtual bool setVariablesHandler(const System::VariablesHandler& variablesHandler);

    /**
     * Initialize the task
     * @param paramHandler a pointer to the parameter handler containing all the information
     * required by the specific task. Please refer to the documentation of the implemented
     * version of the model for further details.
     * the optimization problem. Please use the same variables handler to initialize all the
     * Tasks.
     * @return True in case of success, false otherwise.
     */
    virtual bool initialize(std::weak_ptr<ParametersHandler::IParametersHandler> paramHandler);

    /**
     * Update the content of the task.
     * @return True in case of success, false otherwise.
     */
    virtual bool update();

    /**
     * Get the matrix A.
     * @return a const reference to the matrix A.
     */
    Eigen::Ref<const Eigen::MatrixXd> getA() const;

    /**
     * Get the vector b.
     * @return a const reference to the vector b.
     */
    Eigen::Ref<const Eigen::VectorXd> getB() const;

    /**
     * Get the description of the task.
     * @return a string containing the description of the task.
     */
    const std::string& getDescription() const;

    /**
     * Get the size of the task. (I.e the number of rows of the vector b)
     * @return the size of the task.
     */
    virtual std::size_t size() const = 0;

    /**
     * Get the type of the task. Namely equality or inequality.
     * @return the size of the task.
     */
    virtual Type type() const = 0;
};

} // namespace IK
} // namespace BipedalLocomotion

#endif // BIPEDAL_LOCOMOTION_IK_LINEAR_TASK_H
