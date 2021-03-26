/**
 * @file FloatingBaseDynamicsWithCompliantContacts.cpp
 * @authors Giulio Romualdi
 * @copyright 2021 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the GNU Lesser General Public License v2.1 or any later version.
 */

#include <iDynTree/Core/EigenHelpers.h>
#include <iDynTree/Core/Twist.h>
#include <iDynTree/Model/Model.h>

#include <BipedalLocomotion/ContinuousDynamicalSystem/FloatingBaseDynamicsWithCompliantContacts.h>
#include <BipedalLocomotion/Conversions/CommonConversions.h>
#include <BipedalLocomotion/Math/Constants.h>
#include <BipedalLocomotion/TextLogging/Logger.h>

using namespace BipedalLocomotion;
using namespace BipedalLocomotion::ContinuousDynamicalSystem;
using namespace BipedalLocomotion::ParametersHandler;

bool FloatingBaseDynamicsWithCompliantContacts::initialize(std::weak_ptr<IParametersHandler> handler)
{
    constexpr auto logPrefix = "[FloatingBaseDynamicsWithCompliantContacts::initialize]";

    auto ptr = handler.lock();
    if (ptr == nullptr)
    {
        log()->error("{} The parameter handler is expired. Please call the function passing a "
                     "pointer pointing an already allocated memory.",
                     logPrefix);
        return false;
    }

    if (!ptr->getParameter("rho", m_rho))
    {
        log()->info("{} The Baumgarte stabilization parameter not found. The default one will be "
                    "used {}.",
                    logPrefix,
                    m_rho);
    }

    if (!ptr->getParameter("gravity", m_gravity))
    {
        log()->info("{} The gravity vector  not found. The default one will be "
                    "used {}.",
                    logPrefix,
                    m_gravity.transpose());
    }

    return true;
}

bool FloatingBaseDynamicsWithCompliantContacts::setKinDyn(
    std::shared_ptr<iDynTree::KinDynComputations> kinDyn)
{
    constexpr auto logPrefix = "[FloatingBaseDynamicsWithCompliantContacts::setKinDyn]";

    if (kinDyn == nullptr)
    {
        log()->error("{} The kinDyn computation object is not valid.", logPrefix);
        return false;
    }

    m_kinDyn = kinDyn;
    m_actuatedDoFs = m_kinDyn->model().getNrOfDOFs();

    // resize matrices
    m_massMatrix.resize(m_actuatedDoFs + m_baseDoFs, m_actuatedDoFs + m_baseDoFs);
    m_jacobianMatrix.resize(m_baseDoFs, m_actuatedDoFs + m_baseDoFs);
    m_generalizedRobotAcceleration.resize(m_actuatedDoFs + m_baseDoFs);
    m_knownCoefficent.resize(m_actuatedDoFs + m_baseDoFs);

    return true;
}

bool FloatingBaseDynamicsWithCompliantContacts::setMassMatrixRegularization(
    const Eigen::Ref<const Eigen::MatrixXd>& matrix)
{
    constexpr auto logPrefix = "[FloatingBaseDynamicsWithCompliantContacts::"
                               "setMassMatrixRegularization]";

    if (m_kinDyn == nullptr)
    {
        log()->error("{} Please call setKinDyn() before.", logPrefix);
        return false;
    }

    if ((m_actuatedDoFs + m_baseDoFs != matrix.rows()) || (matrix.cols() != matrix.rows()))
    {
        const auto rightSize = m_actuatedDoFs + m_baseDoFs;

        log()->error("{} The size of the regularization matrix is not correct. The correct size "
                     "is:  {} x {}. While the input of the function is a {} x {} matrix.",
                     logPrefix,
                     rightSize,
                     rightSize,
                     matrix.rows(),
                     matrix.cols());
        return false;
    }

    m_massMatrixReglarizationTerm = matrix;
    m_useMassMatrixRegularizationTerm = true;

    return true;
}

bool FloatingBaseDynamicsWithCompliantContacts::dynamics(const double& time,
                                                         StateDerivative& stateDerivative)
{
    constexpr auto logPrefix = "[FloatingBaseDynamicsWithCompliantContacts::dynamics]";

    if (m_kinDyn == nullptr)
    {
        log()->error("{} Please call setKinDyn() before.", logPrefix);
        return false;
    }

    // get the state
    const auto& [baseVelocity, jointVelocity, basePosition, baseOrientation, jointPositions]
        = m_state;

    auto& [baseAcceleration,
           jointAcceleration,
           baseLinearVelocity,
           baseRotationRate,
           jointVelocityOutput]
        = stateDerivative;

    const auto& [jointTorques, contactWrenches] = m_controlInput;

    // check the size of the vectors
    if (jointVelocity.size() != m_actuatedDoFs || jointPositions.size() != m_actuatedDoFs
        || jointTorques.size() != m_actuatedDoFs)
    {
        log()->error("{} Wrong size of the vectors.", logPrefix);
        return false;
    }

    // compute the base linear velocity
    baseLinearVelocity = baseVelocity.head<3>();

    // here we assume that the velocity is expressed using the mixed representation
    baseRotationRate = -baseOrientation.colwise().cross(baseVelocity.tail<3>())
                       + m_rho / 2.0 * (baseOrientation.transpose().inverse() - baseOrientation);

    jointVelocityOutput = jointVelocity;

    // update kindyncomputations object
    const Eigen::Matrix4d baseTransform = Conversions::toEigenPose(baseOrientation, basePosition);
    if (!m_kinDyn
             ->setRobotState(baseTransform, jointPositions, baseVelocity, jointVelocity, m_gravity))
    {
        log()->error("{} Unable to update the kinDyn object.", logPrefix);
        return false;
    }

    // compute the mass matrix
    if (!m_kinDyn->getFreeFloatingMassMatrix(m_massMatrix))
    {
        log()->error("{} Unable to get the mass matrix.", logPrefix);
        return false;
    }

    // compute the generalized bias forces
    // here we want to compute the robot acceleration as
    // robotAcceleration = M^-1 (-h + J' F + B tau) = M^-1 * m_knownCoefficent

    if (!m_kinDyn->generalizedBiasForces(m_knownCoefficent))
    {
        log()->error("{} Unable to get the bias forces.", logPrefix);
        return false;
    }

    m_knownCoefficent *= -1;

    // add the contact wrench to the knownCoefficent
    for (const auto& contactWrench : contactWrenches)
    {
        // compute the contact jacobian
        if (!m_kinDyn->getFrameFreeFloatingJacobian(contactWrench.index(), m_jacobianMatrix))
        {
            log()->error("{} Unable to get the jacobian of the frame named: {}.",
                         logPrefix,
                         m_kinDyn->model().getFrameLink(contactWrench.index()));
            return false;
        }

        // update the state of the contact model
        auto contactPtr = contactWrench.contactModel().lock();
        if (contactPtr == nullptr)
        {
            log()->error("{} The contact model associated to the frame named {} is expired",
                         logPrefix,
                         m_kinDyn->model().getFrameLink(contactWrench.index()));
            return false;
        }

        contactPtr->setState(m_kinDyn->getFrameVel(contactWrench.index()),
                             m_kinDyn->getWorldTransform(contactWrench.index()));

        m_knownCoefficent.noalias()
            += m_jacobianMatrix.transpose() * iDynTree::toEigen(contactPtr->getContactWrench());
    }

    // add the joint torques to the known coefficient
    m_knownCoefficent.tail(m_actuatedDoFs) += jointTorques;

    // resize the joint acceleration
    jointAcceleration.resize(m_actuatedDoFs);

    // compute the generalized robot acceleration solving the linear system. Here we assume that the
    // mass matrix is positive definite (check here for further informations:
    // https://eigen.tuxfamily.org/dox/group__TutorialLinearAlgebra.html)
    if (m_useMassMatrixRegularizationTerm)
    {
        m_generalizedRobotAcceleration
            = (m_massMatrix + m_massMatrixReglarizationTerm).llt().solve(m_knownCoefficent);
    } else
    {
        m_generalizedRobotAcceleration = m_massMatrix.llt().solve(m_knownCoefficent);
    }

    // split the acceleration in base and joint acceleration
    baseAcceleration = m_generalizedRobotAcceleration.head<m_baseDoFs>();
    jointAcceleration = m_generalizedRobotAcceleration.tail(m_actuatedDoFs);

    return true;
}

bool FloatingBaseDynamicsWithCompliantContacts::setState(const State& state)
{
    m_state = state;
    return true;
}

const FloatingBaseDynamicsWithCompliantContacts::State&
FloatingBaseDynamicsWithCompliantContacts::getState() const
{
    return m_state;
}

bool FloatingBaseDynamicsWithCompliantContacts::setControlInput(const Input& controlInput)
{
    m_controlInput = controlInput;
    return true;
}
