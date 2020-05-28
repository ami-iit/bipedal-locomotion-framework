/**
 * @file FloatingBaseDynamicalSystem.cpp
 * @authors Giulio Romualdi
 * @copyright 2020 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the GNU Lesser General Public License v2.1 or any later version.
 */

#include <iDynTree/Model/Model.h>
#include <iDynTree/Core/Twist.h>
#include <iDynTree/Core/EigenHelpers.h>

#include <BipedalLocomotion/System/FloatingBaseDynamicalSystem.h>

using namespace BipedalLocomotion::System;

FloatingBaseDynamicalSystem::FloatingBaseDynamicalSystem()
{
    // set the gravity vector
    m_gravity.zero();
    m_gravity(2) = -9.81;
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
    m_generalizedBiasForces.resize(m_kinDyn->model());
    m_generalizedRobotAcceleration.resize(m_actuatedDoFs + m_baseDoFs);
    m_knownCoefficent.resize(m_actuatedDoFs + m_baseDoFs);

    return true;
}

bool FloatingBaseDynamicalSystem::dynamics(const StateType& state,
                                           const double& time,
                                           StateDerivativeType& stateDerivative)
{

    if (m_kinDyn == nullptr)
    {
        std::cerr << "[FloatingBaseDynamicalSystem::dynamics] Please call 'setKinDyn()' before."
                  << std::endl;
        return false;
    }

    // get the state
    const iDynTree::Vector6& baseVelocity = std::get<0>(state);
    const iDynTree::VectorDynSize& jointVelocity = std::get<1>(state);
    const iDynTree::Position& basePosition = std::get<2>(state);
    const iDynTree::Rotation& baseRotation = std::get<3>(state);
    const iDynTree::VectorDynSize& jointPositions = std::get<4>(state);

    // get the state derivative
    iDynTree::Vector6& baseAcceleration = std::get<0>(stateDerivative);
    iDynTree::VectorDynSize& jointAcceleration = std::get<1>(stateDerivative);
    iDynTree::Vector3& baseLinearVelocity = std::get<2>(stateDerivative);
    iDynTree::Matrix3x3& baseRotationRate = std::get<3>(stateDerivative);
    iDynTree::VectorDynSize& jointVelocityOutput = std::get<4>(stateDerivative);

    const iDynTree::VectorDynSize& jointTorques = std::get<0>(m_controlInput);
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
    iDynTree::toEigen(baseLinearVelocity) = iDynTree::toEigen(baseVelocity).head<3>();

    // here we assume that the velocity is expressed using the mixed representation
    iDynTree::toEigen(baseRotationRate) = iDynTree::skew(iDynTree::toEigen(baseVelocity).tail<3>())
                                          * iDynTree::toEigen(baseRotation);

    jointVelocityOutput = jointVelocity;

    // update kindyncomputations object
    iDynTree::Twist baseTwist;
    iDynTree::toEigen(baseTwist.getLinearVec3()) = iDynTree::toEigen(baseVelocity).head<3>();
    iDynTree::toEigen(baseTwist.getAngularVec3()) = iDynTree::toEigen(baseVelocity).tail<3>();
    if (!m_kinDyn->setRobotState(iDynTree::Transform(baseRotation, basePosition),
                                 jointPositions,
                                 baseTwist,
                                 jointVelocity,
                                 m_gravity))
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
    if (!m_kinDyn->generalizedBiasForces(m_generalizedBiasForces))
    {
        std::cerr << "[FloatingBaseDynamicalSystem::dynamics] Unable to get the bias forces."
                  << std::endl;
        return false;
    }

    // here we want to compute the robot acceleration as
    // robotAcceleration = M^-1 (-h + J' F + B tau) = M^-1 * m_knownCoefficent

    // add the generalized bias forces to the known coefficent
    iDynTree::toEigen(m_knownCoefficent).head<m_baseDoFs>()
        = -iDynTree::toEigen(m_generalizedBiasForces.baseWrench());
    iDynTree::toEigen(m_knownCoefficent).tail(m_actuatedDoFs)
        = -iDynTree::toEigen(m_generalizedBiasForces.jointTorques());

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
        contactWrench.contactModel()->setState(m_kinDyn->getFrameVel(contactWrench.index()),
                                               m_kinDyn->getWorldTransform(contactWrench.index()));

        iDynTree::toEigen(m_knownCoefficent) += iDynTree::toEigen(m_jacobianMatrix).transpose()
            * iDynTree::toEigen(contactWrench.contactModel()->getContactWrench());
    }

    // add the joint torques to the known coefficent
    iDynTree::toEigen(m_knownCoefficent).tail(m_actuatedDoFs) += iDynTree::toEigen(jointTorques);

    // resize the joint acceleration
    jointAcceleration.resize(m_actuatedDoFs);

    // compute the generalized robot acceleration solving the linear system. Here we assume that the
    // mass matrix is positive definite (check here for further informations:
    // https://eigen.tuxfamily.org/dox/group__TutorialLinearAlgebra.html)
    iDynTree::toEigen(m_generalizedRobotAcceleration)
        = iDynTree::toEigen(m_massMatrix).llt().solve(iDynTree::toEigen(m_knownCoefficent));

    // split the acceleration in base and joint acceleration
    iDynTree::toEigen(baseAcceleration)
        = iDynTree::toEigen(m_generalizedRobotAcceleration).head<m_baseDoFs>();
    iDynTree::toEigen(jointAcceleration)
        = iDynTree::toEigen(m_generalizedRobotAcceleration).tail(m_actuatedDoFs);

    return true;
}
