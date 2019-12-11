/**
 * @file OptimizationProblemElements.h
 * @authors Giulio Romualdi
 * @copyright 2019 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the GNU Lesser General Public License v2.1 or any later version.
 */

#ifndef BIPEDAL_LOCOMOTION_CONTROLLERS_OPTIMAL_CONTROL_UTILITIES_OPTIMIZATION_PROBLEM_ELEMENTS_H
#define BIPEDAL_LOCOMOTION_CONTROLLERS_OPTIMAL_CONTROL_UTILITIES_OPTIMIZATION_PROBLEM_ELEMENTS_H

#include <iDynTree/Core/MatrixDynSize.h>
#include <iDynTree/Core/Utils.h>
#include <iDynTree/Core/VectorDynSize.h>

#include <BipedalLocomotionControllers/OptimalControlUtilities/ControlProblemElements.h>
#include <BipedalLocomotionControllers/OptimalControlUtilities/VariableHandler.h>

namespace BipedalLocomotionControllers
{
/**
 * Constraints class handles the equality and inequality constraint for a QP
 * optimization problem. i.e. \f$ l \le A x \le u \f$ where
 * \f$ A \f$ is the constraint matrix
 * \f$ l \f$ and \f$ u \f$ are the lower and upper bound respectively
 */
class Constraints
{
    /**
     * EqualityConstraint contains the equality constraints
     */
    typedef struct EqualityConstraint
    {
        CostFunctionOrEqualityConstraintElement* element; /**< Pointer to the constraint */
        iDynTree::IndexRange indexRange; /**< Size and offset of the constraint */

        /**
         * Constructor
         */
        EqualityConstraint(CostFunctionOrEqualityConstraintElement* const element,
                           const iDynTree::IndexRange& indexRange);
    } EqualityConstraint;

    typedef struct InequalityConstraint
    {
        InequalityConstraintElement* element; /**< Pointer to the constraint */
        iDynTree::IndexRange indexRange; /**< Size and offset of the constraint */

        /**
         * Constructor
         */
        InequalityConstraint(InequalityConstraintElement* const element,
                             const iDynTree::IndexRange& indexRange);

    } InequalityConstraint;

    std::vector<EqualityConstraint> m_equalityConstrains; /**< Vector containing all the equality
                                                             constraints */
    std::vector<InequalityConstraint> m_inequalityConstrains; /**< Vector containing all the
                                                                 inequality constraints */

    iDynTree::VectorDynSize m_lowerBound; /**< Lower bound */
    iDynTree::VectorDynSize m_upperBound; /**< Upper bound */
    iDynTree::MatrixDynSize m_constraintMatrix; /**<  Constraint matrix */

    int m_numberOfVariables; /**< Number of variables */
    int m_numberOfConstraints{0}; /**< Number of constraint */

    /**
     * Get the index of the next constraint
     * @return the index of the next constraint
     */
    int getNextConstraintIndex() const;

public:
    /**
     * Constructor
     * @param handler variable handler object
     */
    Constraints(const VariableHandler& handler);

    /**
     * Add a new constraint. The constraint matrix and the bound vector are resized
     * @param element pointer to the constraint
     */
    void addConstraint(CostFunctionOrEqualityConstraintElement* element);

    /**
     * Add a new constraint. The constraint matrix and the bound vector are resized
     * @param element pointer to the constraint
     */
    void addConstraint(InequalityConstraintElement* element);

    /**
     * Get (and compute) the lower and the upper bounds
     * @return pair containing the bounds. The first element is the lower
     * bound, the second one is the upper bound
     */
    std::pair<iDynTree::VectorDynSize&, iDynTree::VectorDynSize&> getBounds();

    /**
     * Get (and compute) the constraint matrix
     * @return the constraint matrix
     */
    const iDynTree::MatrixDynSize& getConstraintMatrix();

    /**
     * Get the number of constraints
     * @return the number of constraints
     */
    int getNumberOfConstraints() const;

    /**
     * Get the equality constraints
     * @return a vector containing the reference to the equality constraints
     */
    const std::vector<EqualityConstraint>& getEqualityConstraints() const;

    /**
     * Get the inequality constraints
     * @return a vector containing the reference to the inequality constraints
     */
    const std::vector<InequalityConstraint>& getInequalityConstraints() const;
};

/**
 * CostFunction class handles the cost function for a QP
 * optimization problem. i.e. \f$ \frac{1}{2} (x ^ \top H x + g^\top x \f$ where
 * \f$ H \f$ is the hessian matrix
 * \f$ g \f$ is the gradient vector
 */
class CostFunction
{
    /**
     * CostFunctionElement contains an element of the cost function
     */
    typedef struct CostFunctionElement
    {
        CostFunctionOrEqualityConstraintElement* element; /**< Pointer to the element */
        iDynTree::VectorDynSize weight; /**< Weight */
        double weightOffset{0}; /**< Offset of the weight i.e. Weight = weightScaling * customWeight
                                   + weightOffset */
        double weightScaling{1}; /**< Scaling factor of the weight i.e. Weight = weightScaling *
                                    customWeight + weightOffset */

        /**
         * Constructor
         */
        CostFunctionElement(CostFunctionOrEqualityConstraintElement* const element,
                            const iDynTree::VectorDynSize& weight,
                            const double& weightScaling = 1,
                            const double& weightOffset = 0);

    } CostFunctionElement;

    /** Unordered map containing all the cost function element */
    std::unordered_map<std::string, CostFunctionElement> m_costFunctionElements;

    iDynTree::VectorDynSize m_gradient; /**< Gradient vector */
    iDynTree::MatrixDynSize m_hessianMatrix; /**< Hessian matrix */

public:
    /**
     * Constructor
     * @param handler variable handler object
     */
    CostFunction(const VariableHandler& handler);

    /**
     * Add a new cost function element.
     * @param element pointer to the cost function element
     * @param weight cost weight
     * @param weightScaling Scaling factor of the weight
     * @param weightOffset offset of the weight
     * @param name name associated to the element
     */
    void addCostFunction(CostFunctionOrEqualityConstraintElement* element,
                         const iDynTree::VectorDynSize& weight,
                         const double& weightScaling,
                         const double& weightOffset,
                         const std::string& name);

    /**
     * Set the weight. The weight will be computed using
     * weight = weightScaling * customWeight + weightOffset
     * @param customWeight custom weight
     * @param name name associated to the element
     * @return true if the given name correspond to one of the element
     */
    bool setWeight(const iDynTree::VectorDynSize& customWeight, const std::string& name);

    /**
     * Set the weight. The weight will be computed using
     * weight = weightScaling * customWeight + weightOffset
     * @param customWeight custom weight
     * @param name name associated to the element
     * @return true if the given name correspond to one of the element
     */
    bool setWeight(const double& weight, const std::string& name);

    /**
     * Get (and compute) the hessian and the gradient
     * @return pair containing the hessian and the gradient. The first element is hessian
     * the second one is the gradient
     */
    std::pair<const iDynTree::MatrixDynSize&, iDynTree::VectorDynSize&> getElements();

    /**
     * Get the cost functions
     * @return a map containing the reference to the cost function elements
     */
    const std::unordered_map<std::string, CostFunction::CostFunctionElement>&
    getCostFunctions() const;
};
} // namespace BipedalLocomotionControllers

#endif // BIPEDAL_LOCOMOTION_CONTROLLERS_OPTIMAL_CONTROL_UTILITIES_OPTIMIZATION_PROBLEM_ELEMENTS_H
