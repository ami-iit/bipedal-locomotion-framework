/**
 * @file FloatingBaseSystemDynamics.cpp
 * @authors Giulio Romualdi
 * @copyright 2020 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the GNU Lesser General Public License v2.1 or any later version.
 */

#include <iDynTree/Model/Model.h>
#include <iDynTree/Core/Twist.h>
#include <iDynTree/Core/EigenHelpers.h>

#include <BipedalLocomotion/Conversions/CommonConversions.h>
#include <BipedalLocomotion/System/FloatingBaseSystemDynamics.h>

using namespace BipedalLocomotion;
using namespace BipedalLocomotion::System;
using namespace BipedalLocomotion::ParametersHandler;

bool FloatingBaseDynamicalSystem::initalize(std::weak_ptr<IParametersHandler> handler)
{
    auto ptr = handler.lock();
    if (ptr == nullptr)
    {
        std::cerr << "[FloatingBaseDynamicalSystem::initalize] The parameter handler is expired. "
                     "Please call the function passing a pointer pointing an already allocated "
                     "memory."
                  << std::endl;
        return false;
    }

    if (!ptr->getParameter("rho", m_rho))
    {
        std::cerr << "[FloatingBaseDynamicalSystem::initalize] Unable to load the Baumgarte "
                     "stabilization parameter."
                  << std::endl;
        return false;
    }

    return true;
}


FloatingBaseDynamicalSystem::FloatingBaseDynamicalSystem()
{
    // set the gravity vector
    m_gravity.setZero();
    m_gravity(2) = -9.81;
}

void FloatingBaseDynamicalSystem::setGravityVector(const Eigen::Ref<const Eigen::Vector3d>& gravity)
{
    m_gravity = gravity;
}

bool FloatingBaseDynamicalSystem::setKinDyn(std::shared_ptr<iDynTree::KinDynComputations> kinDyn)
{
    if (kinDyn == nullptr)
    {
        std::cerr << "[FloatingBaseDynamicalSystem::setKinDynComputation] Corrupted KinDyn "
                     "computation object."
                  << std::endl;
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

bool FloatingBaseDynamicalSystem::setMassMatrixRegularization(const Eigen::Ref<const Eigen::MatrixXd>& matrix)
{

    if (m_kinDyn == nullptr)
    {
        std::cerr << "[FloatingBaseDynamicalSystem::setMassMatrixRegularization] Please call "
                     "'setKinDyn()' before."
                  << std::endl;
        return false;
    }

    if ((m_actuatedDoFs + m_baseDoFs != matrix.rows()) || (matrix.cols() != matrix.rows()))
    {
        const auto rightSize = m_actuatedDoFs + m_baseDoFs;
        std::cerr << "[FloatingBaseDynamicalSystem::setMassMatrixRegularization] The size of the "
                     "regularization matrix is not correct. The correct size is: "
                  << rightSize << " x " << rightSize << ". While the input of the function is a "
                  << matrix.rows() << " x " << matrix.cols() << " matrix." << std::endl;
        return false;
    }

    m_massMatrixReglarizationTerm = matrix;
    m_useMassMatrixRegularizationTerm = true;
    return true;
}

bool FloatingBaseDynamicalSystem::dynamics(const double& time,
                                           StateDerivativeType& stateDerivative)
{

    if (m_kinDyn == nullptr)
    {
        std::cerr << "[FloatingBaseDynamicalSystem::dynamics] Please call 'setKinDyn()' before."
                  << std::endl;
        return false;
    }

    // get the state
    const auto& [baseVelocity, jointVelocity, basePosition, baseOrientation, jointPositions]
        = m_state;

    auto
        & [baseAcceleration, jointAcceleration, baseLinearVelocity, baseRotationRate,
           jointVelocityOutput]
        = stateDerivative;

    const Eigen::VectorXd & jointTorques = std::get<0>(m_controlInput);
    const std::vector<ContactWrench>& contactWrenches = std::get<1>(m_controlInput);

    // check the size of the vectors
    if (jointVelocity.size() != m_actuatedDoFs || jointPositions.size() != m_actuatedDoFs
        || jointTorques.size() != m_actuatedDoFs)
    {
        std::cerr << "[FloatingBaseDynamicalSystem::dynamics] Wrong size of the vectors."
                  << std::endl;
        return false;
    }

    // compute the base linear velocity
    baseLinearVelocity = baseVelocity.head<3>();
    baseRotationRate = -baseOrientation.colwise().cross(baseVelocity.tail<3>())
                       + m_rho / 2.0
                             * ((baseOrientation * baseOrientation.transpose()).inverse()
                                - Eigen::Matrix3d::Identity())
                             * baseOrientation;

    jointVelocityOutput = jointVelocity;

    // update kindyncomputations object
    const Eigen::Matrix4d baseTransform = Conversions::toEigenPose(baseOrientation, basePosition);
    if (!m_kinDyn->setRobotState(baseTransform, jointPositions,
                                 baseVelocity, jointVelocity, m_gravity))
    {
        std::cerr << "[FloatingBaseDynamicalSystem::dynamics] Unable to update the kindyn object."
                  << std::endl;
        return false;
    }

    // compute the mass matrix
    if (!m_kinDyn->getFreeFloatingMassMatrix(m_massMatrix))
    {
        std::cerr << "[FloatingBaseDynamicalSystem::dynamics] Unable to get the mass matrix."
                  << std::endl;
        return false;
    }

    // compute the generalized bias forces
    // here we want to compute the robot acceleration as
    // robotAcceleration = M^-1 (-h + J' F + B tau) = M^-1 * m_knownCoefficent

    if (!m_kinDyn->generalizedBiasForces(m_knownCoefficent))
    {
        std::cerr << "[FloatingBaseDynamicalSystem::dynamics] Unable to get the bias forces."
                  << std::endl;
        return false;
    }

    m_knownCoefficent *= -1;

    // add the contact wrench to the knownCoefficent
    for (const auto& contactWrench : contactWrenches)
    {
        // compute the contact jacobian
        if (!m_kinDyn->getFrameFreeFloatingJacobian(contactWrench.index(), m_jacobianMatrix))
        {
            std::cerr << "[FloatingBaseDynamicalSystem::dynamics] Unable to get the Jacobian for "
                         "the frame named: "
                      << m_kinDyn->model().getFrameLink(contactWrench.index()) << "." << std::endl;
            return false;
        }

        // update the state of the contact model
        auto contactPtr = contactWrench.contactModel().lock();
        if (contactPtr == nullptr)
        {
            std::cerr << "[FloatingBaseDynamicalSystem::dynamics] The contact model associated to "
                         "the frame named: "
                      << m_kinDyn->model().getFrameLink(contactWrench.index())
                      << " has been expired." << std::endl;
            return false;
        }

        contactPtr->setState(m_kinDyn->getFrameVel(contactWrench.index()),
                             m_kinDyn->getWorldTransform(contactWrench.index()));

        m_knownCoefficent += m_jacobianMatrix.transpose()
            * iDynTree::toEigen(contactPtr->getContactWrench());
    }

    // add the joint torques to the known coefficent
    m_knownCoefficent.tail(m_actuatedDoFs) += jointTorques;

    // resize the joint acceleration
    jointAcceleration.resize(m_actuatedDoFs);

    // compute the generalized robot acceleration solving the linear system. Here we assume that the
    // mass matrix is positive definite (check here for further informations:
    // https://eigen.tuxfamily.org/dox/group__TutorialLinearAlgebra.html)
    if (m_useMassMatrixRegularizationTerm)
        m_generalizedRobotAcceleration
            = (m_massMatrix + m_massMatrixReglarizationTerm).llt().solve(m_knownCoefficent);
    else
        m_generalizedRobotAcceleration = m_massMatrix.llt().solve(m_knownCoefficent);

    // split the acceleration in base and joint acceleration
    baseAcceleration = m_generalizedRobotAcceleration.head<m_baseDoFs>();
    jointAcceleration = m_generalizedRobotAcceleration.tail(m_actuatedDoFs);

    return true;
}
