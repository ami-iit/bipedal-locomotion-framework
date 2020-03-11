/**
 * @file RecursiveLeastSquare.tpp
 * @authors Giulio Romualdi
 * @copyright 2020 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the GNU Lesser General Public License v2.1 or any later version.
 */

#ifndef BIPEDAL_LOCOMOTION_CONTROLLERS_ESTIMATORS_RLS_TPP
#define BIPEDAL_LOCOMOTION_CONTROLLERS_ESTIMATORS_RLS_TPP

#include <memory>
#include <iostream>

#include <BipedalLocomotionControllers/Estimators/RecursiveLeastSquare.h>
#include <iDynTree/Core/EigenHelpers.h>

namespace BipedalLocomotionControllers
{
namespace Estimators
{

template <class Derived>
bool RecursiveLeastSquare::initialize(std::weak_ptr<ParametersHandler::IParametersHandler<Derived>> handlerWeak)
{
    if (m_estimatorState != State::NotInitialized)
    {
        std::cerr << "[RecursiveLeastSquare::initialize] The estimator has been already "
                     "initialized."
                  << std::endl;
        return false;
    }

    if (handlerWeak.expired())
    {
        std::cerr << "[RecursiveLeastSquare::initialize] The parameter handler is expired. Please "
                     "check its scope."
                  << std::endl;
        return false;
    }

    auto handler = handlerWeak.lock();

    // we assume that the measuraments are uncorrelated (furthermore since the model of the noise is
    // Gaussian the random variable are also independent)
    // Since the variable are independent we are interested only on the element in the diagonal
    // of the matrix
    iDynTree::VectorDynSize measurementCovariance;
    if (!handler->getParameter("measurement_covariance", measurementCovariance))
    {
        std::cerr << "[RecursiveLeastSquare::initialize] Unable to find the covariance matrix of "
                     "the measuraments."
                  << std::endl;
        return false;
    }

    m_measurementCovarianceMatrix.resize(measurementCovariance.size(), measurementCovariance.size());
    iDynTree::toEigen(m_measurementCovarianceMatrix) = iDynTree::toEigen(measurementCovariance).asDiagonal();

    // get the lambda
    if (!handler->getParameter("lambda", m_lambda))
    {
        std::cerr << "[RecursiveLeastSquare::initialize] Unable to find lambda." << std::endl;
        return false;
    }

    // check if the presence of the initial initial guess of the state and of the covariance matrix
    if (!handler->getParameter("state", m_state))
    {
        std::cerr << "[RecursiveLeastSquare::initialize] Unable to get the initial guess."
                  << std::endl;
        return false;
    }

    iDynTree::VectorDynSize stateCovariance;
    if(!handler->getParameter("state_covariance", stateCovariance))
    {
        std::cerr << "[RecursiveLeastSquare::initialize] Unable to get the initial state covariance."
                  << std::endl;
        return false;
    }

    m_stateCovarianceMatrix.resize(m_state.size(), m_state.size());
    iDynTree::toEigen(m_stateCovarianceMatrix) = iDynTree::toEigen(stateCovariance).asDiagonal();

    // resize the vector containing the measuraments
    m_measuraments.resize(measurementCovariance.size());
    m_measuraments.zero();

    // resize the matrix containing the kalman gain
    m_kalmanGain.resize(m_state.size(), measurementCovariance.size());
    m_kalmanGain.zero();

    m_estimatorState = State::Initialized;

    return true;
}

} // namespace Estimators
} // namespace BipedalLocomotionControllers

#endif // BIPEDAL_LOCOMOTION_CONTROLLERS_ESTIMATORS_RLS_TPP
