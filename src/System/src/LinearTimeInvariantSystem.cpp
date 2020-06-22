/**
 * @file LinearTimeInvariantSystem.cpp
 * @authors Giulio Romualdi
 * @copyright 2020 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the GNU Lesser General Public License v2.1 or any later version.
 */

#include <iostream>

#include <iDynTree/Core/EigenHelpers.h>

#include <BipedalLocomotion/System/LinearTimeInvariantSystem.h>

using namespace BipedalLocomotion::System;

bool LinearTimeInvariantSystem::setSystemMatrices(const iDynTree::MatrixDynSize& A,
                                                  const iDynTree::MatrixDynSize& B)
{

    if (A.rows() != B.rows())
    {
        std::cerr << "[LinearTimeInvariantSystem::setSystemMatrices] A and B must have the same "
                     "number of rows."
                  << std::endl;
        return false;
    }

    if (A.rows() != A.cols())
    {
        std::cerr << "[LinearTimeInvariantSystem::setSystemMatrices] The A matrix has to be a "
                     "square matrix."
                  << std::endl;
        return false;
    }

    m_A = A;
    m_B = B;
    m_isInitialized = true;

    return true;
}

bool LinearTimeInvariantSystem::dynamics(const StateType& state,
                                         const double& time,
                                         StateDerivativeType& stateDerivative)
{

    if (!m_isInitialized)
    {
        std::cerr << "[LinearTimeInvariantSystem::dynamics] Please initialize the matrices."
                  << std::endl;
        return false;
    }

    const auto& x = std::get<0>(state);
    auto& dx = std::get<0>(stateDerivative);
    const auto& u = std::get<0>(m_controlInput);

    if (x.size() != m_A.rows())
    {
        std::cerr << "[LinearTimeInvariantSystem::dynamics] The size of the vector 'state' is not "
                     "coherent with the system matrices."
                  << std::endl;
        return false;
    }

    if (u.size() != m_B.cols())
    {
        std::cerr << "[LinearTimeInvariantSystem::dynamics] The size of the vector 'control input' "
                     "is not coherent with the system matrices."
                  << std::endl;
        return false;
    }

    dx.resize(x.size());
    iDynTree::toEigen(dx) = iDynTree::toEigen(m_A) * iDynTree::toEigen(x)
                            + iDynTree::toEigen(m_B) * iDynTree::toEigen(u);

    return true;
}
