/**
 * @file MomentumBasedTorqueControl.cpp
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

#include <BipedalLocomotionControllers/WholeBodyControllers/MomentumBasedTorqueControlWithCompliantContacts.h>

using namespace BipedalLocomotionControllers::WholeBodyControllers;
using namespace BipedalLocomotionControllers::OptimalControlUtilities;

class MomentumBasedTorqueControl::Impl
{
    friend MomentumBasedTorqueControl;

    template <typename T> using dictionary = std::unordered_map<std::string, T>;

    const std::shared_ptr<iDynTree::KinDynComputations> kinDyn; /**< KinDyn pointer object */
    std::unique_ptr<OsqpEigen::Solver> solver; /**< Quadratic programming solver */
    VariableHandler variableHandler; /**< Variable handler */
    std::unique_ptr<Constraints> constraints; /**< Collection of all the constraints */
    std::unique_ptr<CostFunction> costFunction; /**< The cost function */

    iDynTree::VectorDynSize jointTorques; /**< Desired joint torques */

    bool isVerbose{true}; /**< If true the controller will be verbose */

    /** Dynamics of the floating base */
    std::unique_ptr<FloatingBaseDynamicsElement> floatingBaseDynamics;

    /** Dynamics of the joint base */
    std::unique_ptr<JointSpaceDynamicsElement> jointDynamics;

    /** Centroidal linear momentum in case of elastic contacts */
    std::unique_ptr<CentroidalLinearMomentumElementWithCompliantContact> centroidalLinearMomentumElement;

    /** Dictionary containing regularization elements */
    dictionary<std::unique_ptr<RegularizationElement>> regularizationElements;

    /** Dictionary containing regularization with control elements */
    dictionary<std::unique_ptr<RegularizationWithControlElement>> regularizationWithControlElements;

    /** Dictionary containing Cartesian elements */
    dictionary<std::unique_ptr<CartesianElement<CartesianElementType::ORIENTATION>>> cartesianElements;

    /** Joint values element */
    std::unique_ptr<JointValuesFeasibilityElement> jointValuesFeasibilityElement;

    Impl(const std::shared_ptr<iDynTree::KinDynComputations> kinDyn)
        : kinDyn(kinDyn)
    {
        // instantiate variable handler and initialize the variables
        variableHandler.addVariable("base_acceleration", 6);
        variableHandler.addVariable("joint_accelerations", this->kinDyn->model().getNrOfDOFs());

        // initialize the constraints
        constraints = std::make_unique<Constraints>(variableHandler);

        // initialize the cost function
        costFunction = std::make_unique<CostFunction>(variableHandler);

        // resize the joint torques
        jointTorques.resize(this->kinDyn->model().getNrOfDOFs());
    };

    void printElements() const
    {
        // print useful information
        std::cout << "Cost Functions" << std::endl;
        std::cout << "---------------------" << std::endl;
        for (const auto& cost : costFunction->getCostFunctions())
            std::cout << cost.second.element->getName() << std::endl;
        std::cout << "---------------------" << std::endl;

        std::cout << "Equality Constraints" << std::endl;
        std::cout << "---------------------" << std::endl;
        for (const auto& constraint : constraints->getEqualityConstraints())
            std::cout << constraint.element->getName() << std::endl;
        std::cout << "---------------------" << std::endl;

        std::cout << "Inequality Constraints" << std::endl;
        std::cout << "---------------------" << std::endl;
        for (const auto& constraint : constraints->getInequalityConstraints())
            std::cout << constraint.element->getName() << std::endl;
        std::cout << "---------------------" << std::endl;
    }

    void initialze()
    {
        // initialize the optimization problem
        solver = std::make_unique<OsqpEigen::Solver>();
        solver->data()->setNumberOfVariables(variableHandler.getNumberOfVariables());
        solver->data()->setNumberOfConstraints(constraints->getNumberOfConstraints());

        solver->settings()->setVerbosity(false);
        solver->settings()->setLinearSystemSolver(0);
        solver->settings()->setMaxIteraction(100000);
        solver->settings()->setPolish(false);

        printElements();
    };

    void solve()
    {
        auto costElements = costFunction->getElements();
        auto bounds = constraints->getBounds();

        Eigen::SparseMatrix<double> hessianSparse
            = iDynTree::toEigen(costElements.hessian()).sparseView();

        Eigen::SparseMatrix<double> constraintSparse
            = iDynTree::toEigen(constraints->getConstraintMatrix()).sparseView();

        iDynTree::VectorDynSize& gradient = costElements.gradient();

        if (solver->isInitialized())
        {
            // update matrices hessian matrix
            // TODO do it in a smart way

            if (!solver->updateHessianMatrix(hessianSparse))
                throw std::runtime_error("[MomentumBasedTorqueControl::Impl::"
                                         "solve] Unable to update the hessian");

            if (!solver->updateGradient(iDynTree::toEigen(gradient)))
                throw std::runtime_error("[MomentumBasedTorqueControl::Impl::"
                                         "solve] Unable to update the gradient");

            if (!solver->updateLinearConstraintsMatrix(constraintSparse))
                throw std::runtime_error("[MomentumBasedTorqueControl::Impl::"
                                         "solve] Unable to update the linear constraint matrix");

            if (!solver->updateBounds(iDynTree::toEigen(bounds.lowerBound()),
                                      iDynTree::toEigen(bounds.upperBound())))
                throw std::runtime_error("[MomentumBasedTorqueControl::Impl::"
                                         "solve] Unable to update the bounds");
        } else
        {
            if (!solver->data()->setHessianMatrix(hessianSparse))
                throw std::runtime_error("[MomentumBasedTorqueControl::Impl::"
                                         "solve] Unable to set the hessian the first time");

            if (!solver->data()->setGradient(iDynTree::toEigen(gradient)))
                throw std::runtime_error("[MomentumBasedTorqueControl::Impl::"
                                         "solve] Unable to set the gradient the first time");

            if (!solver->data()->setLinearConstraintsMatrix(constraintSparse))
                throw std::runtime_error("[MomentumBasedTorqueControl::Impl::"
                                         "solve] Unable to set the linear constraint matrix the "
                                         "first time");

            if (!solver->data()->setLowerBound(iDynTree::toEigen(bounds.lowerBound())))
                throw std::runtime_error("[MomentumBasedTorqueControl::Impl::"
                                         "solve] Unable to set the lower bounds the first time");

            if (!solver->data()->setUpperBound(iDynTree::toEigen(bounds.upperBound())))
                throw std::runtime_error("[MomentumBasedTorqueControl::Impl::"
                                         "solve] Unable to set the upper bounds the first time");

            if (!solver->initSolver())
                throw std::runtime_error("[MomentumBasedTorqueControl::Impl::"
                                         "solve] Unable to initialize the problem");
        }

        if (!solver->solve())
            throw std::runtime_error("[MomentumBasedTorqueControl::Impl::"
                                     "solve] Unable to solve the problem");
    }
};

// MomentumBasedTorqueControl
MomentumBasedTorqueControl::MomentumBasedTorqueControl(
    std::shared_ptr<iDynTree::KinDynComputations> kinDyn)
    : m_pimpl(new Impl(kinDyn))
{
}

MomentumBasedTorqueControl::~MomentumBasedTorqueControl()
{
}

void MomentumBasedTorqueControl::setVerbosity(bool isVerbose) noexcept
{
    m_pimpl->isVerbose = isVerbose;
}

// TODO
void MomentumBasedTorqueControl::addCentroidalLinearMomentumElement(
    const std::vector<FrameInContactWithContactModel<std::string, std::string>>& framesInContact,
    std::unique_ptr<LinearPD<iDynTree::Vector3>> pdController,
    bool isConstraint,
    const Weight<iDynTree::VectorDynSize>& weight /**= Weight<iDynTree::VectorDynSize>::Zero(3) */)
{
    m_pimpl->centroidalLinearMomentumElement = std::make_unique<
        CentroidalLinearMomentumElementWithCompliantContact>(m_pimpl->kinDyn,
                                                             std::move(pdController),
                                                             m_pimpl->variableHandler,
                                                             framesInContact);

    // add to the constraint or to the cost function
    if (isConstraint)
        m_pimpl->constraints->addConstraint(m_pimpl->centroidalLinearMomentumElement.get());
    else
        m_pimpl->costFunction->addCostFunction(m_pimpl->centroidalLinearMomentumElement.get(),
                                               weight,
                                               "centroidal_linear_momentum");
};

void MomentumBasedTorqueControl::addOrientationElement(const Frame<std::string, std::string>& frameName,
                                                       std::unique_ptr<OrientationPD> pdController,
                                                       bool isConstraint,
                                                       const Weight<iDynTree::VectorDynSize>& weight)

{
    const auto& label = frameName.identifierInVariableHandler();
    const auto& frameInTheModel = frameName.identifierInModel();

    if (m_pimpl->cartesianElements.find(label) != m_pimpl->cartesianElements.end())
        throw std::runtime_error("[TaskBasedTorqueControl::addCartesianElement] The element named "
                                 + label + " has been already added.");

    m_pimpl->cartesianElements.emplace(label,
                                       std::make_unique<CartesianElement<CartesianElementType::ORIENTATION>>(m_pimpl->kinDyn,
                                                                                                             std::move(pdController),
                                                                                                             m_pimpl->variableHandler,
                                                                                                             frameInTheModel));

    if (isConstraint)
        m_pimpl->constraints->addConstraint(m_pimpl->cartesianElements.find(label)->second.get());
    else
        m_pimpl->costFunction->addCostFunction(m_pimpl->cartesianElements.find(label)->second.get(),
                                               weight,
                                               label + "_cartesian_element");
};

void MomentumBasedTorqueControl::addFloatingBaseDynamicsElement(
    const std::vector<FrameInContact<std::string, std::string>>& framesInContact,
    bool isConstraint,
    const Weight<iDynTree::VectorDynSize>& weight)
{
    m_pimpl->floatingBaseDynamics
        = std::make_unique<FloatingBaseDynamicsElement>(m_pimpl->kinDyn,
                                                        m_pimpl->variableHandler,
                                                        framesInContact);

    // add to the constraint or to the cost function
    if (isConstraint)
        m_pimpl->constraints->addConstraint(m_pimpl->floatingBaseDynamics.get());
    else
        m_pimpl->costFunction->addCostFunction(m_pimpl->floatingBaseDynamics.get(),
                                               weight,
                                               "system_dynamics");
};

void MomentumBasedTorqueControl::addJointDynamicsElement(const std::vector<FrameInContact<std::string, std::string>>& framesInContact)
{
    VariableHandler tempVariableHandler(m_pimpl->variableHandler);
    size_t jointAccelerationSize = m_pimpl->variableHandler.getVariable("joint_accelerations").size;
    tempVariableHandler.addVariable("joint_torques", jointAccelerationSize);

    m_pimpl->jointDynamics = std::make_unique<JointSpaceDynamicsElement>(m_pimpl->kinDyn,
                                                                         tempVariableHandler,
                                                                         framesInContact);
};

void MomentumBasedTorqueControl::addRegularizationWithControlElement(const std::string& label,
                                                                     std::unique_ptr<LinearPD<iDynTree::VectorDynSize>> pdController,
                                                                     bool isConstraint,
                                                                     const Weight<iDynTree::VectorDynSize>& weight)
{
    if (m_pimpl->regularizationWithControlElements.find(label)
        != m_pimpl->regularizationWithControlElements.end())
        throw std::runtime_error("[TaskBasedTorqueControl::addRegularizationWithControlElement] "
                                 "This element has been already added.");

    m_pimpl->regularizationWithControlElements.emplace(label,
                                                       std::make_unique<RegularizationWithControlElement>(m_pimpl->kinDyn,
                                                                                                          std::move(pdController),
                                                                                                          m_pimpl->variableHandler,
                                                                                                          label));

    if (isConstraint)
        m_pimpl->constraints->addConstraint(
            m_pimpl->regularizationWithControlElements.find(label)->second.get());
    else
        m_pimpl->costFunction
            ->addCostFunction(m_pimpl->regularizationWithControlElements.find(label)->second.get(),
                              weight,
                              label);
}

void MomentumBasedTorqueControl::addJointValuesFeasibilityElement(
    const std::string& variableName,
    const iDynTree::VectorDynSize& maxJointPositionsLimit,
    const iDynTree::VectorDynSize& minJointPositionsLimit,
    const double& samplingTime)
{
    m_pimpl->jointValuesFeasibilityElement
        = std::make_unique<JointValuesFeasibilityElement>(m_pimpl->kinDyn,
                                                          m_pimpl->variableHandler,
                                                          variableName,
                                                          maxJointPositionsLimit,
                                                          minJointPositionsLimit,
                                                          samplingTime);

    m_pimpl->constraints->addConstraint(m_pimpl->jointValuesFeasibilityElement.get());
}

void MomentumBasedTorqueControl::initialize()
{
    m_pimpl->initialze();
}

void MomentumBasedTorqueControl::solve()
{
    m_pimpl->solve();
}

size_t MomentumBasedTorqueControl::variableSize(const std::string& name) const
{
    size_t variableSize = 0;
    const auto variable = m_pimpl->variableHandler.getVariable(name);
    if (variable.isValid())
        variableSize = variable.size;
    else if(m_pimpl->isVerbose)
        std::cerr << "[MomentumBasedTorqueControl::variableSize] Variable named "
                  << name << " not found." << std::endl;

    return variableSize;
}

iDynTree::VectorDynSize MomentumBasedTorqueControl::getDesiredTorques()
{
    iDynTree::VectorDynSize jointTorques(
        m_pimpl->variableHandler.getVariable("joint_accelerations").size);

    const size_t numberOfVariables = m_pimpl->variableHandler.getNumberOfVariables();

    iDynTree::toEigen(jointTorques)
        = iDynTree::toEigen(m_pimpl->jointDynamics->getB())
          - iDynTree::toEigen(m_pimpl->jointDynamics->getA()).topRows(numberOfVariables)
                * m_pimpl->solver->getSolution();

    return jointTorques;
}

// iDynTree::Wrench TaskBasedTorqueControl::getDesiredWrench(const std::string& name)
// {
//     auto wrenchIndex = m_pimpl->variableHandler->getVariable(name);

//     if (!wrenchIndex.isValid())
//         return iDynTree::Wrench::Zero();

//     iDynTree::Wrench wrench;
//     iDynTree::toEigen(wrench.getLinearVec3())
//         = m_pimpl->solver->getSolution().segment(wrenchIndex.offset, 3);
//     iDynTree::toEigen(wrench.getAngularVec3())
//         = m_pimpl->solver->getSolution().segment(wrenchIndex.offset + 3, 3);

//     return wrench;
// }

void MomentumBasedTorqueControl::setDesiredRegularizationTrajectory(
    const iDynTree::VectorDynSize& acceleration,
    const iDynTree::VectorDynSize& velocity,
    const iDynTree::VectorDynSize& position,
    const std::string& name)
{
    const auto& element = m_pimpl->regularizationWithControlElements.find(name);
    if (element == m_pimpl->regularizationWithControlElements.end())
    {
        if (m_pimpl->isVerbose)
            std::cerr << "[TaskBasedTorqueControl::setDesiredRegularizationTrajectory] The "
                         "regularization element with control called "
                      << name << " is not defined" << std::endl;
        return;
    }

    element->second->setDesiredTrajectory(acceleration, velocity, position);
}

void MomentumBasedTorqueControl::setJointState(
    const iDynTree::VectorDynSize& velocity, const iDynTree::VectorDynSize& position)
{
    const auto& element = m_pimpl->regularizationWithControlElements.find("joint_accelerations");

    if (element == m_pimpl->regularizationWithControlElements.end())
    {
        if (m_pimpl->isVerbose)
            std::cerr << "[TaskBasedTorqueControl::setJointState] The regularization element with "
                         "control called"
                      << " joint_accelerations is not defined" << std::endl;
        return;
    }

    element->second->setState(velocity, position);
}

void MomentumBasedTorqueControl::setDesiredLinearMomentumValue(
    const iDynTree::Vector3& centroidalLinearMomentumSecondDerivative,
    const iDynTree::Vector3& centroidalLinearMomentumDerivative,
    const iDynTree::Vector3& centroidalLinearMomentum)
{
    if (m_pimpl->centroidalLinearMomentumElement == nullptr)
    {
        if (m_pimpl->isVerbose)
            std::cerr << "[MomentumBasedTorqueControl::"
                         "setDesiredLinearMomentumValue] The centroidal linear momentum element "
                         "angular momentum element does not exist."
                      << std::endl;
        return;
    }

    m_pimpl->centroidalLinearMomentumElement
        ->setDesiredCentroidalLinearMomentum(centroidalLinearMomentumSecondDerivative,
                                             centroidalLinearMomentumDerivative,
                                             centroidalLinearMomentum);
}

void MomentumBasedTorqueControl::setMeasuredContactWrench(const std::unordered_map<std::string, iDynTree::Wrench>& contactWrenches)
{

    std::vector<iDynTree::LinearForceVector3> contactForces;

    for (const auto& contactWrench : contactWrenches)
    {
        m_pimpl->jointDynamics->setExternalWrench(contactWrench.first, contactWrench.second);
        m_pimpl->floatingBaseDynamics->setExternalWrench(contactWrench.first, contactWrench.second);

        // stack all the contact forces
        contactForces.push_back(contactWrench.second.getLinearVec3());

    }


    m_pimpl->centroidalLinearMomentumElement->setMeasuredContactForces(contactForces);
}

void MomentumBasedTorqueControl::setContactState(const std::string& name,
                                                 bool isInContact,
                                                 const iDynTree::Transform& desiredFootPose)
{
    m_pimpl->centroidalLinearMomentumElement->setContactState(name, isInContact, desiredFootPose);
}

void MomentumBasedTorqueControl::setDesiredRotationReference(const iDynTree::Vector3& acceleration,
                                                             const iDynTree::Vector3& velocity,
                                                             const iDynTree::Rotation& rotation,
                                                             const std::string& name)
{

    auto element = m_pimpl->cartesianElements.find(name);
    if (element == m_pimpl->cartesianElements.end())
    {
        if(m_pimpl->isVerbose)
            std::cerr << "[MomentumBasedTorqueControl::setDesiredCartesianTrajectory] The "
                         "cartesian element called "
                      << name << " is not defined" << std::endl;
        return;
    }

    element->second->setReference(acceleration, velocity, rotation);
}

// void TaskBasedTorqueControl::setDesiredAngularMomentum(
//     const iDynTree::Vector3& centroidalAngularMomentumVelocity,
//     const iDynTree::Vector3& centroidalAngularMomentum)
// {
//     auto element = centroidalAngularMomentumElement();
//     if (element == nullptr)
//     {
//         if (m_pimpl->isVerbose)
//             std::cerr << "[TaskBasedTorqueControl::setDesiredAngularMomentum] The centroidal "
//                          "angular momentum element does not exist"
//                       << std::endl;
//         return;
//     }

//     element->setDesiredCentroidalAngularMomentum(centroidalAngularMomentumVelocity,
//                                                  centroidalAngularMomentum);
// }

// void TaskBasedTorqueControl::setWeight(const iDynTree::VectorDynSize& weight,
//                                        const std::string& name)
// {
//     if (!m_pimpl->costFunction->setWeight(weight, name) && m_pimpl->isVerbose)
//     {
//         std::cerr << "[TaskBasedTorqueControl::setWeight] The cost function element named " <<
//         name
//                   << " does not exist" << std::endl;
//     }
// }

// void TaskBasedTorqueControl::setWeight(const double& weight, const std::string& name)
// {
//     if (!m_pimpl->costFunction->setWeight(weight, name) && m_pimpl->isVerbose)
//     {
//         std::cerr << "[TaskBasedTorqueControlnnnnnnnnnnnnnnnnnnnnnn::setWeight] The cost function
//         element named " << name
//                   << " does not exist" << std::endl;
//     }
// }
