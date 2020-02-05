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
#include <BipedalLocomotionControllers/OptimalControlUtilities/Weight.h>

namespace BipedalLocomotionControllers
{
namespace OptimalControlUtilities
{
/**
 * Constraints class handles the equality and inequality constraint for a QP
 * optimization problem. i.e. \f$l \le A x \le u\f$ where
 * \f$A\f$ is the constraint matrix
 * \f$l\f$ and \f$u\f$ are the lower and upper bound respectively
 */
class Constraints
{
    /**
     * EqualityConstraint contains the equality constraints
     */
    struct EqualityConstraint
    {
        ControlTask* element; /**< Pointer to the constraint */
        iDynTree::IndexRange indexRange; /**< Size and offset of the constraint */

        /**
         * Constructor
         */
        EqualityConstraint(ControlTask* const element, const iDynTree::IndexRange& indexRange);
    };

    struct InequalityConstraint
    {
        InequalityConstraintElement* element; /**< Pointer to the constraint */
        iDynTree::IndexRange indexRange; /**< Size and offset of the constraint */

        /**
         * Constructor
         */
        InequalityConstraint(InequalityConstraintElement* const element,
                             const iDynTree::IndexRange& indexRange);
    };

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
     * Bounds wraps an std::pair<iDynTree::VectorDynSize&, iDynTree::VectorDynSize&>.
     * It contains the lower and the upper bounds
     */
    class Bounds : public std::pair<iDynTree::VectorDynSize&, iDynTree::VectorDynSize&>
    {
        // obscure first and second attribute
        using std::pair<iDynTree::VectorDynSize&, iDynTree::VectorDynSize&>::first;
        using std::pair<iDynTree::VectorDynSize&, iDynTree::VectorDynSize&>::second;

    public:
        // use the std::pair constructors
        using std::pair<iDynTree::VectorDynSize&, iDynTree::VectorDynSize&>::pair;

        iDynTree::VectorDynSize& lowerBound();
        const iDynTree::VectorDynSize& lowerBound() const;
        iDynTree::VectorDynSize& upperBound();
        const iDynTree::VectorDynSize& upperBound() const;
    };

    /**
     * Constructor
     * @param handler variable handler object
     */
    Constraints(const VariableHandler& handler);

    /**
     * Add a new constraint. The constraint matrix and the bound vector are resized
     * @param element pointer to the constraint
     */
    void addConstraint(ControlTask* element);

    /**
     * Add a new constraint. The constraint matrix and the bound vector are resized
     * @param element pointer to the constraint
     */
    void addConstraint(InequalityConstraintElement* element);

    /**
     * Get (and compute) the lower and the upper bounds
     * @return the lower and upper bounds
     */
    Bounds getBounds();

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
 * optimization problem. i.e. \f$\frac{1}{2} (x ^ \top H x + g^\top x)\f$ where
 * \f$H\f$ is the hessian matrix
 * \f$g\f$ is the gradient vector
 */
class CostFunction
{
    /**
     * CostFunctionElement contains an element of the cost function
     */
    struct CostFunctionElement
    {
        ControlTask* element; /**< Pointer to the element */

        Weight<iDynTree::VectorDynSize> weight;

        /**
         * Constructor
         */
        CostFunctionElement(ControlTask* const element,
                            const Weight<iDynTree::VectorDynSize>& weight);
    };

    /** Unordered map containing all the cost function element */
    std::unordered_map<std::string, CostFunctionElement> m_costFunctionElements;

    iDynTree::VectorDynSize m_gradient; /**< Gradient vector */
    iDynTree::MatrixDynSize m_hessianMatrix; /**< Hessian matrix */

public:
    /**
     * Elements wraps an std::pair<iDynTree::MatrixDynSize&, iDynTree::VectorDynSize&>. It contains
     * the hessian matrix and the gradient vector
     */
    class Elements : public std::pair<iDynTree::MatrixDynSize&, iDynTree::VectorDynSize&>
    {
        // obscure first and second attribute
        using std::pair<iDynTree::MatrixDynSize&, iDynTree::VectorDynSize&>::first;
        using std::pair<iDynTree::MatrixDynSize&, iDynTree::VectorDynSize&>::second;

    public:
        // use the std::pair constructors
        using std::pair<iDynTree::MatrixDynSize&, iDynTree::VectorDynSize&>::pair;

        iDynTree::MatrixDynSize& hessian();
        const iDynTree::MatrixDynSize& hessian() const;
        iDynTree::VectorDynSize& gradient();
        const iDynTree::VectorDynSize& gradient() const;
    };

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
    void addCostFunction(ControlTask* element,
                         const Weight<iDynTree::VectorDynSize>& weight,
                         const std::string& name);

    /**
     * Set the weight. The weight will be computed using
     * weight = weightScaling * customWeight + weightOffset
     * @param customWeight custom weight
     * @param name name associated to the element
     * @return true if the given name correspond to one of the element
     */
    bool setWeight(const Weight<iDynTree::VectorDynSize>& weight, const std::string& name);

    /**
     * Get (and compute) the hessian and the gradient
     * @return the hessian matrix and the gradient vector
     */
    Elements getElements();

    /**
     * Get the cost functions
     * @return a map containing the reference to the cost function elements
     */
    const std::unordered_map<std::string, CostFunction::CostFunctionElement>&
    getCostFunctions() const;
};
} // namespace OptimalControlUtilities
} // namespace BipedalLocomotionControllers

#endif // BIPEDAL_LOCOMOTION_CONTROLLERS_OPTIMAL_CONTROL_UTILITIES_OPTIMIZATION_PROBLEM_ELEMENTS_H
