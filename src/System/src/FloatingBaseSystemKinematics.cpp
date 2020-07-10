/**
 * @file FloatingBaseSystemKinematics.cpp
 * @authors Giulio Romualdi
 * @copyright 2020 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the GNU Lesser General Public License v2.1 or any later version.
 */

#include <BipedalLocomotion/System/FloatingBaseSystemKinematics.h>

using namespace BipedalLocomotion::System;

bool FloatingBaseSystemKinematics::setState(const StateType& state)
{
    std::get<0>(m_state) = std::get<0>(state);
    std::get<2>(m_state) = std::get<2>(state);

    // project the base orientation matrix in SO3
    // here we assume that the velocity is expressed using the mixed representation
    const Eigen::Matrix3d& baseOrientation = std::get<1>(state);
    Eigen::JacobiSVD<Eigen::Matrix3d> svd(baseOrientation,
                                          Eigen::ComputeFullU | Eigen::ComputeFullV);
    std::get<1>(m_state) = svd.matrixU() * svd.matrixV().transpose();

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
    baseRotationRate = -baseRotation.colwise().cross(baseTwist.tail<3>());

    jointVelocityOutput = jointVelocity;

    return true;
}
