/**
 * @file FloatingBaseSystemKinematics.cpp
 * @authors Giulio Romualdi
 * @copyright 2020 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the GNU Lesser General Public License v2.1 or any later version.
 */

#include <BipedalLocomotion/System/FloatingBaseSystemKinematics.h>

using namespace BipedalLocomotion::System;
using namespace BipedalLocomotion::ParametersHandler;

bool FloatingBaseSystemKinematics::initalize(std::weak_ptr<IParametersHandler> handler)
{
    auto ptr = handler.lock();
    if (ptr == nullptr)
    {
        std::cerr << "[FloatingBaseSystemKinematics::initalize] The parameter handler is expired. "
                     "Please call the function passing a pointer pointing an already allocated "
                     "memory."
                  << std::endl;
        return false;
    }

    if (!ptr->getParameter("rho", m_rho))
    {
        std::cerr << "[FloatingBaseSystemKinematics::initalize] Unable to load the Baumgarte "
                     "stabilization parameter."
                  << std::endl;
        return false;
    }

    return true;
}

bool FloatingBaseSystemKinematics::dynamics(const double& time,
                                            StateDerivativeType& stateDerivative)
{
    // get the state
    const Eigen::Vector3d& basePosition = std::get<0>(m_state);
    const Eigen::Matrix3d& baseRotation = std::get<1>(m_state);
    const Eigen::VectorXd& jointPositions = std::get<2>(m_state);

    // get the state derivative
    Eigen::Vector3d& baseLinearVelocity = std::get<0>(stateDerivative);
    Eigen::Matrix3d& baseRotationRate = std::get<1>(stateDerivative);
    Eigen::VectorXd& jointVelocityOutput = std::get<2>(stateDerivative);

    const Eigen::Matrix<double, 6, 1>& baseTwist = std::get<0>(m_controlInput);
    const Eigen::VectorXd& jointVelocity = std::get<1>(m_controlInput);

    // check the size of the vectors
    if (jointVelocity.size() != jointPositions.size())
    {
        std::cerr << "[FloatingBaseSystemKinematics::dynamics] Wrong size of the vectors."
                  << std::endl;
        return false;
    }

    // compute the base linear velocity
    baseLinearVelocity = baseTwist.head<3>();

    // here we assume that the velocity is expressed using the mixed representation
    baseRotationRate = -baseRotation.colwise().cross(baseTwist.tail<3>())
                       + m_rho / 2.0
                             * ((baseRotation * baseRotation.transpose()).inverse()
                                - Eigen::Matrix3d::Identity())
                             * baseRotation;

    jointVelocityOutput = jointVelocity;

    return true;
}
