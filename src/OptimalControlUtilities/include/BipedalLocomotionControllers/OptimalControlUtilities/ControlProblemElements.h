/**
 * @file ControlProblemElements.h
 * @authors Giulio Romualdi
 * @copyright 2019 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the GNU Lesser General Public License v2.1 or any later version.
 */

#ifndef BIPEDAL_LOCOMOTION_CONTROLLERS_OPTIMAL_CONTROL_UTILITIES_CONTROL_PROBLEM_ELEMENTS_H
#define BIPEDAL_LOCOMOTION_CONTROLLERS_OPTIMAL_CONTROL_UTILITIES_CONTROL_PROBLEM_ELEMENTS_H

#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include <iDynTree/Model/Indices.h>
#include <iDynTree/Core/Utils.h>

#include <iDynTree/Core/MatrixDynSize.h>

#include <iDynTree/Core/Rotation.h>
#include <iDynTree/Core/VectorDynSize.h>
#include <iDynTree/KinDynComputations.h>

namespace BipedalLocomotionControllers
{

namespace OptimalControlUtilities
{

/**
 * The struct frame defines a Frame in a ControlProblemElement
 */
struct Frame
{
    iDynTree::FrameIndex indexInModel; /**< Index of the frame in the Model */

    /** Index and size of the element in the ControlProblemElement */
    iDynTree::IndexRange indexRangeInElement{iDynTree::IndexRange::InvalidRange()};
};

/**
 * FrameNames wraps a std::pair<std::string, std::string>. It contains the label
 * associated to the frame in the VariableHandler end the name of the frame in the model
 */
class FrameNames : public std::pair<std::string, std::string>
{
    // obscure first and second attribute
    using std::pair<std::string, std::string>::first;
    using std::pair<std::string, std::string>::second;

public:

    // use the std::pair constructors
    using std::pair<std::string, std::string>::pair;

    std::string& label();
    const std::string& label() const;
    std::string& nameInModel();
    const std::string& nameInModel() const;
};

/**
 * ControlProblemElement describes a general control problem element. The
 * element is considered linear w.r.t the unknown variable.
 * i.e. \f$ A x \f$ where \f$ A \f$ is the element matrix and \f$ x \f$ the
 * unknown variable
 */
class ControlProblemElement
{
protected:
    std::shared_ptr<iDynTree::KinDynComputations> m_kinDynPtr; /**< KinDyn pointer object */
    iDynTree::MatrixDynSize m_A; /**< Element Matrix */
    std::string m_name; /**< Name of the element */

public:
    /**
     * Constructor
     * @param kinDyn KinDynComputations object
     */
    ControlProblemElement(std::shared_ptr<iDynTree::KinDynComputations> kinDyn);

    /**
     * Get the size of the element (i.e. the number of rows of the element matrix)
     * @return the size of the element
     */
    size_t getSize() const;

    /**
     * Get the element matrix
     * @return the element matrix
     */
    virtual const iDynTree::MatrixDynSize& getA();

    /**
     * Get the name of the element
     * @return the description of the element
     */
    const std::string& getName() const;
};

/**
 * ControlTask describes a control problem element that will be embedded as cost function or as
 * equality constraint. The element is described by \f$ A \f$ and \f$ b \f$. \f$ A \f$ is the
 * element matrix and \f$ x \f$ and \f$ b \f$ the element vector.
 * In case of cost function \f$ A \f$ and \f$ b \f$ represents:
 * \f$ \|Ax - b\|^2 \f$. In case of equality constraint
 * \f$ Ax = b \f$
 */
class ControlTask : public ControlProblemElement
{
protected:
    iDynTree::VectorDynSize m_b; /**< Element Vector */

public:
    /**
     * Constructor
     * @param kinDyn KinDynComputations object
     */
    ControlTask(std::shared_ptr<iDynTree::KinDynComputations> kinDyn);

    /**
     * Get the element vector
     * @return the element vector
     */
    virtual const iDynTree::VectorDynSize& getB();
};

/**
 * InequalityConstraintElement describes a control problem
 * element that will be embedded as inequality constraint.
 * The element is described by \f$ A \f$, \f$ l \f$ and  \f$ u \f$.
 * \f$ l \le Ax \le u \f$
 */
class InequalityConstraintElement : public ControlProblemElement
{
protected:
    iDynTree::VectorDynSize m_l; /**< Lower bound */
    iDynTree::VectorDynSize m_u; /**< Upper bound */

public:
    /**
     * Constructor
     * @param kinDyn KinDynComputations object
     */
    InequalityConstraintElement(std::shared_ptr<iDynTree::KinDynComputations> kinDyn);

    /**
     * Get the upper bound
     * @return the upper bound
     */
    virtual const iDynTree::VectorDynSize& getUpperBound();

    /**
     * Get the lower bound
     * @return the lower bound
     */
    virtual const iDynTree::VectorDynSize& getLowerBound();
};

} // namespace OptimalControlUtilities
} // namespace BipedalLocomotionControllers

#endif // BIPEDAL_LOCOMOTION_CONTROLLERS_OPTIMAL_CONTROL_UTILITIES_CONTROL_PROBLEM_ELEMENTS_H
