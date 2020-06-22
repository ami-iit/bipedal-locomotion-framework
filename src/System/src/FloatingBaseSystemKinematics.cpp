/**
 * @file FloatingBaseSystemKinematics.cpp
 * @authors Giulio Romualdi
 * @copyright 2020 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the GNU Lesser General Public License v2.1 or any later version.
 */

#include <iDynTree/Model/Model.h>
#include <iDynTree/Core/Twist.h>
#include <iDynTree/Core/EigenHelpers.h>

#include <BipedalLocomotion/System/FloatingBaseSystemKinematics.h>

using namespace BipedalLocomotion::System;

bool FloatingBaseSystemKinematics::dynamics(const StateType& state,
                                            const double& time,
                                            StateDerivativeType& stateDerivative)
{
    // get the state
    const iDynTree::Position& basePosition = std::get<0>(state);
    const iDynTree::Rotation& baseRotation = std::get<1>(state);
    const iDynTree::VectorDynSize& jointPositions = std::get<2>(state);

    // get the state derivative
    iDynTree::Vector3& baseLinearVelocity = std::get<0>(stateDerivative);
    iDynTree::Matrix3x3& baseRotationRate = std::get<1>(stateDerivative);
    iDynTree::VectorDynSize& jointVelocityOutput = std::get<2>(stateDerivative);

    const iDynTree::Twist& baseTwist = std::get<0>(m_controlInput);
    const iDynTree::VectorDynSize& jointVelocity = std::get<1>(m_controlInput);

    // check the size of the vectors
    if (jointVelocity.size() != jointPositions.size())
    {
        std::cerr << "[FloatingBaseSystemKinematics::dynamics] Wrong size of the vectors."
                  << std::endl;
        return false;
    }

    // compute the base linear velocity
    iDynTree::toEigen(baseLinearVelocity) = iDynTree::toEigen(baseTwist).head<3>();

    // here we assume that the velocity is expressed using the mixed representation
    iDynTree::toEigen(baseRotationRate)
        = iDynTree::skew(iDynTree::toEigen(baseTwist).tail<3>()) * iDynTree::toEigen(baseRotation);

    jointVelocityOutput = jointVelocity;

    return true;
}
