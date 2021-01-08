/**
 * @file OptimalControlElement.h
 * @authors Giulio Romualdi
 * @copyright 2020 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the GNU Lesser General Public License v2.1 or any later version.
 */

#ifndef BIPEDAL_LOCOMOTION_TSID_OPTIMAL_CONTROL_ELEMENT_H
#define BIPEDAL_LOCOMOTION_TSID_OPTIMAL_CONTROL_ELEMENT_H

#include <memory>

#include <Eigen/Dense>

#include <iDynTree/KinDynComputations.h>

#include <BipedalLocomotion/ParametersHandler/IParametersHandler.h>
#include <BipedalLocomotion/System/VariablesHandler.h>

namespace BipedalLocomotion
{
namespace TSID
{

/**
 * OptimalControlElement describes a control problem element. The element is described by a matrix
 * \f$A\f$ and a vector \f$b\f$. This class describes both a linear equality constraint and a linear
 * inequality constraint. In case of equality constraint \f$ A \f$ and \f$ b \f$ represents: \f$ Ax
 * = b\f$ In case of inequality constraint \f$ A \f$ and \f$ b \f$ represents: \f$ Ax \le b \f$
 * @note Please inherit this class if you want to build your own optimal control problem.
 */
class OptimalControlElement
{
protected:
    Eigen::MatrixXd m_A; /**< Element Matrix */
    Eigen::VectorXd m_b; /**< Element Vector */

    std::string m_description{"Generic Optimal Control Element"}; /**<String describing the content
                                                                     of the element*/

    std::shared_ptr<iDynTree::KinDynComputations> m_kinDyn; /**< Pointer to a KinDynComputations
                                                               object */

    /**
     * Extract the submatrix A associated to a given variable.
     */
    Eigen::Ref<Eigen::MatrixXd>
    subA(const System::VariablesHandler::VariableDescription& description);

public:
    /**
     * Set the kinDynComputations object.
     * @param kinDyn pointer to a kinDynComputations object.
     * @return True in case of success, false otherwise.
     */
    bool setKinDyn(std::shared_ptr<iDynTree::KinDynComputations> kinDyn);

    /**
     * Initialize the optimal control element.
     * @param paramHandler a pointer to the parameter handler containing all the information
     * required by the specific element. Please refer to the documentation of the implemented
     * version of the model for further details.
     * @param variablesHandler class containing the list of all the optimization parameter used in
     * the optimization problem. Please use the same variables handler to initialize all the
     * OptimalControlElements
     * @return True in case of success, false otherwise.
     */
    virtual bool initialize(std::weak_ptr<ParametersHandler::IParametersHandler> paramHandler,
                            const System::VariablesHandler& variablesHandler);

    /**
     * Update the content of the element.
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
     * Get the description of the element.
     * @return a string containing the description of the OptimalControlElement.
     */
    const std::string& getDescription() const;
};

} // namespace TSID
} // namespace BipedalLocomotion

#endif // BIPEDAL_LOCOMOTION_TSID_OPTIMAL_CONTROL_ELEMENT_H
