/**
 * @file MomentumBasedControlHelper.cpp
 * @authors Giulio Romualdi <giulio.romualdi@iit.it>
 * @copyright 2020 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the GNU Lesser General Public License v2.1 or any later version.
 * @date 2020
 */

// std library
#include <unordered_map>
#include <vector>

// osqp-eigen solver
#include <OsqpEigen/OsqpEigen.h>

// iDynTree library
#include <iDynTree/Core/EigenHelpers.h>
#include <iDynTree/Core/Wrench.h>
#include <iDynTree/KinDynComputations.h>
#include <iDynTree/Model/Model.h>

// BipedalLocomotionControllers library
#include <BipedalLocomotionControllers/OptimalControlUtilities/CartesianElements.h>
#include <BipedalLocomotionControllers/OptimalControlUtilities/CentroidalMomentumElementsWithCompliantContacts.h>
#include <BipedalLocomotionControllers/OptimalControlUtilities/FeasibilityElements.h>
#include <BipedalLocomotionControllers/OptimalControlUtilities/FloatingBaseMultiBodyDynamicsElements.h>
#include <BipedalLocomotionControllers/OptimalControlUtilities/Frame.h>
#include <BipedalLocomotionControllers/OptimalControlUtilities/RegularizationElements.h>

#include <BipedalLocomotionControllers/OptimalControlUtilities/OptimizationProblemElements.h>
#include <BipedalLocomotionControllers/OptimalControlUtilities/VariableHandler.h>

#include <BipedalLocomotionControllers/WholeBodyControllers/MomentumBasedControlHelper.h>

using namespace BipedalLocomotionControllers::WholeBodyControllers;
using namespace BipedalLocomotionControllers::OptimalControlUtilities;

void MomentumBasedControlHelper::initializeVariableHandler()
{
    // instantiate variable handler and initialize the variables
    m_variableHandler.addVariable("base_acceleration", 6);
    m_variableHandler.addVariable("joint_accelerations", m_kinDyn->model().getNrOfDOFs());

    // add the stance feet in the optimization problem
    for (const auto& stanceFoot : m_stanceFeetIdetrifiers)
        m_variableHandler.addVariable(stanceFoot.identifierInVariableHandler(), 6);


    // initialize the constraints
    m_constraints = std::make_unique<Constraints>(m_variableHandler);

    // initialize the cost function
    m_costFunction = std::make_unique<CostFunction>(m_variableHandler);
}

void MomentumBasedControlHelper::setVerbosity(bool isVerbose) noexcept
{
    m_isVerbose = isVerbose;
}

MomentumBasedControlHelper::MomentumBasedControlHelper(
    std::shared_ptr<iDynTree::KinDynComputations> kinDyn)
    : m_kinDyn(kinDyn)
{
}

void MomentumBasedControlHelper::printElements() const
{
    // print useful information
    std::cout << m_description << std::endl;
    std::cout << "Cost Functions" << std::endl;
    std::cout << "---------------------" << std::endl;
    for (const auto& cost : m_costFunction->getCostFunctions())
        std::cout << cost.second.element->getName() << std::endl;
    std::cout << "---------------------" << std::endl;

    std::cout << "Equality Constraints" << std::endl;
    std::cout << "---------------------" << std::endl;
    for (const auto& constraint : m_constraints->getEqualityConstraints())
        std::cout << constraint.element->getName() << std::endl;
    std::cout << "---------------------" << std::endl;

    std::cout << "Inequality Constraints" << std::endl;
    std::cout << "----------------------" << std::endl;
    for (const auto& constraint : m_constraints->getInequalityConstraints())
        std::cout << constraint.element->getName() << std::endl;
    std::cout << "---------------------" << std::endl;
}

void MomentumBasedControlHelper::initialzeSolver()
{
    // initialize the optimization problem
    m_solver = std::make_unique<OsqpEigen::Solver>();
    m_solver->data()->setNumberOfVariables(m_variableHandler.getNumberOfVariables());
    m_solver->data()->setNumberOfConstraints(m_constraints->getNumberOfConstraints());

    m_solver->settings()->setVerbosity(false);
    m_solver->settings()->setLinearSystemSolver(0);
    m_solver->settings()->setMaxIteraction(100000);
    m_solver->settings()->setPolish(false);
};

bool MomentumBasedControlHelper::solve()
{
    auto costElements = m_costFunction->getElements();
    auto bounds = m_constraints->getBounds();

    Eigen::SparseMatrix<double> hessianSparse
        = iDynTree::toEigen(costElements.hessian()).sparseView();

    Eigen::SparseMatrix<double> constraintSparse
        = iDynTree::toEigen(m_constraints->getConstraintMatrix()).sparseView();

    iDynTree::VectorDynSize& gradient = costElements.gradient();
    if (m_solver->isInitialized())
    {
        // update matrices hessian matrix
        // TODO do it in a smart way

        if (!m_solver->updateHessianMatrix(hessianSparse))
        {
            std::cerr << "[MomentumBasedControlHelper::solve] Unable to update the hessian"
                      << std::endl;
            return false;
        }

        if (!m_solver->updateGradient(iDynTree::toEigen(gradient)))
        {
            std::cerr << "[MomentumBasedControlHelper::solve] Unable to update the gradient"
                      << std::endl;
            return false;
        }

        if (!m_solver->updateLinearConstraintsMatrix(constraintSparse))
        {
            std::cerr << "[MomentumBasedControlHelper::solve] Unable to update the linear "
                         "constraint matrix"
                      << std::endl;
            return false;
        }

        if (!m_solver->updateBounds(iDynTree::toEigen(bounds.lowerBound()),
                                    iDynTree::toEigen(bounds.upperBound())))
        {
            std::cerr << "[MomentumBasedControlHelper::solve] Unable to update the linear "
                         "constraint matrix"
                      << std::endl;
            return false;
        }
    } else
    {
        if (!m_solver->data()->setHessianMatrix(hessianSparse))
        {
            std::cerr << "[MomentumBasedControlHelper::solve] Unable to set the hessian the first "
                         "time"
                      << std::endl;
            return false;
        }

        if (!m_solver->data()->setGradient(iDynTree::toEigen(gradient)))
        {

            std::cerr << "[MomentumBasedControlHelper::solve] Unable to set the gradient the first "
                         "time"
                      << std::endl;
            return false;
        }

        if (!m_solver->data()->setLinearConstraintsMatrix(constraintSparse))
        {
            std::cerr << "[MomentumBasedControlHelper::solve] Unable to set the linear constraint "
                         "matrix the first time"
                      << std::endl;
            return false;
        }

        if (!m_solver->data()->setLowerBound(iDynTree::toEigen(bounds.lowerBound())))
        {
            std::cerr << "[MomentumBasedControlHelper::solve] Unable to set the lower bound "
                         "vector the first time"
                      << std::endl;
            return false;
        }

        if (!m_solver->data()->setUpperBound(iDynTree::toEigen(bounds.upperBound())))
        {
            std::cerr << "[MomentumBasedControlHelper::solve] Unable to set the upper bound "
                         "vector the first time"
                      << std::endl;
            return false;
        }

        if (!m_solver->initSolver())
        {
            std::cerr << "[MomentumBasedControlHelper::solve] Unable to initialize the software"
                      << std::endl;
            return false;
        }
    }

    if (!m_solver->solve())
    {
        std::cerr << "[MomentumBasedControlHelper::solve] Unable to solve the problem" << std::endl;
        return false;
    }

    return true;
}

void MomentumBasedControlHelper::setCentroidalMomentumReference(
    const iDynTree::SpatialForceVector& momentumSecondDerivative,
    const iDynTree::SpatialForceVector& momentumDerivative,
    const iDynTree::SpatialForceVector& momentum,
    const iDynTree::Vector3& centerOfMass)
{
    if (m_centroidalLinearMomentumElement != nullptr)
        m_centroidalLinearMomentumElement->setReference(momentumSecondDerivative.getLinearVec3(),
                                                        momentumDerivative.getLinearVec3(),
                                                        momentum.getLinearVec3(),
                                                        centerOfMass);


    if (m_centroidalAngularMomentumElement != nullptr)
        m_centroidalAngularMomentumElement->setReference(momentumSecondDerivative.getAngularVec3(),
                                                         momentumDerivative.getAngularVec3(),
                                                         momentum.getAngularVec3());
}

bool MomentumBasedControlHelper::setMeasuredContactWrench(const std::unordered_map<std::string, iDynTree::Wrench>& contactWrenches)
{
    std::vector<iDynTree::LinearForceVector3> contactForces;

    iDynTree::VectorDynSize dummy(6);
    dummy.zero();

    for (const auto& contactWrench : contactWrenches)
    {
        auto contactWrenchElement = m_contactWrenchFeasibilityElements.find(contactWrench.first);
        if(contactWrenchElement != m_contactWrenchFeasibilityElements.end())
            contactWrenchElement->second->setContactWrench(contactWrench.second);

        iDynTree::VectorDynSize wrench(6);
        iDynTree::toEigen(wrench) = iDynTree::toEigen(contactWrench.second.asVector());
        auto contactWrenchRegularizationElement = m_regularizationWithControlElements.find(contactWrench.first);
        if (contactWrenchRegularizationElement != m_regularizationWithControlElements.end())
            contactWrenchRegularizationElement->second->setState(dummy, wrench);
    }

    if (m_jointDynamics != nullptr)
        if (!m_jointDynamics->setMeasuredContactWrenches(contactWrenches))
        {
            std::cerr << "[MomentumBasedControlHelper::setMeasuredContactWrench] Unable to set the "
                         "measured contact wrench in the joint dynamics"
                      << std::endl;
            return false;
        }

    if (m_floatingBaseDynamics != nullptr)
        if (!m_floatingBaseDynamics->setMeasuredContactWrenches(contactWrenches))
        {
            std::cerr << "[MomentumBasedControlHelper::setMeasuredContactWrench] Unable to set the "
                         "measured contact wrench in the floating base dynamics"
                      << std::endl;
            return false;
        }

    if (m_centroidalLinearMomentumElement != nullptr)
        if (!m_centroidalLinearMomentumElement->setMeasuredContactWrenches(contactWrenches))
        {
            std::cerr << "[MomentumBasedControlHelper::setMeasuredContactWrench] Unable to set the "
                         "measured contact wrench in the linear momentum element"
                      << std::endl;
            return false;
        }

    if (m_centroidalAngularMomentumElement != nullptr)
        if (!m_centroidalAngularMomentumElement->setMeasuredContactWrenches(contactWrenches))
        {
            std::cerr << "[MomentumBasedControlHelper::setMeasuredContactWrench] Unable to set the "
                         "measured contact wrench in the angular momentum element"
                      << std::endl;
            return false;
        }

    if (m_centroidalAngularMomentumBound != nullptr)
        if (!m_centroidalAngularMomentumBound->setMeasuredContactWrenches(contactWrenches))
        {
            std::cerr << "[MomentumBasedControlHelper::setMeasuredContactWrench] Unable to set the "
                         "measured contact wrench in the angular momentum bounds"
                      << std::endl;
            return false;
        }

    return true;
}

void MomentumBasedControlHelper::setContactState(const std::string& name,
                                                 bool isInContact,
                                                 const iDynTree::Transform& desiredFootPose)
{
    auto contactModel = m_contactModelElements.find(name);
    if (contactModel != m_contactModelElements.end())
        contactModel->second->setContactState(isInContact, desiredFootPose);
}

void MomentumBasedControlHelper::setFootUpperBoundNormalForce(const std::string& name, const double& force)
{
    auto element = m_contactWrenchFeasibilityElements.find(name);
    if (element != m_contactWrenchFeasibilityElements.end())
        element->second->setUpperBoundNormalForce(force);
}

void MomentumBasedControlHelper::setRotationReference(const iDynTree::Vector3& acceleration,
                                                      const iDynTree::Vector3& velocity,
                                                      const iDynTree::Rotation& rotation,
                                                      const std::string& name)
{
    auto element = m_orientationElements.find(name);
    if (element != m_orientationElements.end())
        element->second->setReference(acceleration, velocity, rotation);
}

void MomentumBasedControlHelper::setTransformationReference(const iDynTree::SpatialAcc& acceleration,
                                                            const iDynTree::Twist& twist,
                                                            const iDynTree::Transform& transform,
                                                            const std::string& name)
{
    auto element = m_cartesianElements.find(name);
    if (element != m_cartesianElements.end())
        element->second->setReference(acceleration, twist, transform);
}

void MomentumBasedControlHelper::setRegularizationReference(const iDynTree::VectorDynSize& acceleration,
                                                            const iDynTree::VectorDynSize& velocity,
                                                            const iDynTree::VectorDynSize& position,
                                                            const std::string& name)
{
    const auto& element = m_regularizationWithControlElements.find(name);
    if (element != m_regularizationWithControlElements.end())
        element->second->setReference(acceleration, velocity, position);
}

void MomentumBasedControlHelper::setJointState(
    const iDynTree::VectorDynSize& velocity, const iDynTree::VectorDynSize& position)
{
    const auto& element = m_regularizationWithControlElements.find("joint_accelerations");

    if (element != m_regularizationWithControlElements.end())
        element->second->setState(velocity, position);
}

iDynTree::VectorDynSize MomentumBasedControlHelper::getDesiredAcceleration()
{
    const size_t numberOfJoints = m_variableHandler.getVariable("joint_accelerations").size;
    iDynTree::VectorDynSize robotAcceleration(numberOfJoints + 6);

    iDynTree::toEigen(robotAcceleration) = m_solver->getSolution().head(numberOfJoints + 6);

    return robotAcceleration;
}

iDynTree::VectorDynSize MomentumBasedControlHelper::getDesiredTorques()
{
    const size_t numberOfJoints = m_variableHandler.getVariable("joint_accelerations").size;
    iDynTree::VectorDynSize jointTorques(numberOfJoints);

    iDynTree::toEigen(jointTorques) = iDynTree::toEigen(m_jointDynamics->getB())
          - iDynTree::toEigen(m_jointDynamics->getA()).leftCols(numberOfJoints + 6)
                * m_solver->getSolution().head(numberOfJoints + 6);

    return jointTorques;
}

iDynTree::Vector6 MomentumBasedControlHelper::getLeftFootWrenchRateOfChange() const
{
    auto index = m_variableHandler.getVariable("left_foot");
    iDynTree::Vector6 forceRateOfChange;
    iDynTree::toEigen(forceRateOfChange) = m_solver->getSolution().segment(index.offset, index.size);

    return forceRateOfChange;
}

iDynTree::Vector6 MomentumBasedControlHelper::getRightFootWrenchRateOfChange() const
{
    auto index = m_variableHandler.getVariable("right_foot");
    iDynTree::Vector6 forceRateOfChange;
    iDynTree::toEigen(forceRateOfChange) = m_solver->getSolution().segment(index.offset, index.size);

    return forceRateOfChange;
}
