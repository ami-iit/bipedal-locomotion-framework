/**
 * @file RecursiveLeastSquare.cpp
 * @authors Giulio Romualdi
 * @copyright 2020 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the GNU Lesser General Public License v2.1 or any later version.
 */

#include <iDynTree/Core/EigenHelpers.h>

#include <BipedalLocomotion/Estimators/RecursiveLeastSquare.h>
#include <BipedalLocomotion/GenericContainer/Vector.h>

using namespace BipedalLocomotion::Estimators;
using namespace BipedalLocomotion::ParametersHandler;
using namespace BipedalLocomotion::GenericContainer;

bool RecursiveLeastSquare::initialize(std::weak_ptr<IParametersHandler> handlerWeak)
{
    if (m_estimatorState != State::NotInitialized)
    {
        std::cerr << "[RecursiveLeastSquare::initialize] The estimator has been already "
                     "initialized."
                  << std::endl;
        return false;
    }

    auto handler = handlerWeak.lock();
    if (handler == nullptr)
    {
        std::cerr << "[RecursiveLeastSquare::initialize] The parameter handler is expired. Please "
                     "check its scope."
                  << std::endl;
        return false;
    }

    // we assume that the measuraments are uncorrelated (furthermore since the model of the noise is
    // Gaussian the random variable are also independent)
    // Since the variable are independent we are interested only on the element in the diagonal
    // of the matrix
    iDynTree::VectorDynSize measurementCovariance;
    if (!handler->getParameter("measurement_covariance", measurementCovariance, VectorResizeMode::Resizable))
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
    if (!handler->getParameter("state", m_state, VectorResizeMode::Resizable))
    {
        std::cerr << "[RecursiveLeastSquare::initialize] Unable to get the initial guess."
                  << std::endl;
        return false;
    }

    iDynTree::VectorDynSize stateCovariance;
    if(!handler->getParameter("state_covariance", stateCovariance, VectorResizeMode::Resizable))
    {
        std::cerr << "[RecursiveLeastSquare::initialize] Unable to get the initial state covariance."
                  << std::endl;
        return false;
    }

    m_stateCovarianceMatrix.resize(m_state.size(), m_state.size());
    iDynTree::toEigen(m_stateCovarianceMatrix) = iDynTree::toEigen(stateCovariance).asDiagonal();

    // resize the vector containing the measuraments
    m_measurements.resize(measurementCovariance.size());
    m_measurements.zero();

    // resize the matrix containing the kalman gain
    m_kalmanGain.resize(m_state.size(), measurementCovariance.size());
    m_kalmanGain.zero();

    m_estimatorState = State::Initialized;

    return true;
}

void RecursiveLeastSquare::setRegressorFunction(std::function<iDynTree::MatrixDynSize(void)> regressor)
{
    m_regressor = regressor;
}

bool RecursiveLeastSquare::advance()
{
    using iDynTree::toEigen;

    if (m_regressor == nullptr)
    {
        std::cerr << "[RecursiveLeastSquare::advance] Please call the setRegressorFunction() "
                     "before calling advance"
                  << std::endl;
        return false;
    }

    if (m_estimatorState != State::Initialized && m_estimatorState != State::Running)
    {
        std::cerr << "[RecursiveLeastSquare::advance] Please initialize the estimator before "
                     "calling advance."
                  << std::endl;
        return false;
    }

    if (m_estimatorState == State::Initialized)
        m_estimatorState = State::Running;

    iDynTree::MatrixDynSize regressor = m_regressor();
    toEigen(m_kalmanGain) = toEigen(m_stateCovarianceMatrix) * toEigen(regressor).transpose()
                            * (m_lambda * toEigen(m_measurementCovarianceMatrix)
                               + (toEigen(regressor) * toEigen(m_stateCovarianceMatrix)
                                  * toEigen(regressor).transpose())).inverse();

    toEigen(m_state) = toEigen(m_state) + toEigen(m_kalmanGain)
                * (toEigen(m_measurements) - toEigen(regressor) * toEigen(m_state));

    toEigen(m_stateCovarianceMatrix) = (toEigen(m_stateCovarianceMatrix)
                                        - toEigen(m_kalmanGain) * toEigen(regressor)
                                        * toEigen(m_stateCovarianceMatrix)) / m_lambda;

    return true;
}

void RecursiveLeastSquare::setMeasurements(const iDynTree::VectorDynSize& measurements)
{
    assert(m_measurements.size() == measurements.size());
    m_measurements = measurements;
}

const iDynTree::VectorDynSize& RecursiveLeastSquare::parametersExpectedValue() const
{
    return m_state;
}

const iDynTree::MatrixDynSize& RecursiveLeastSquare::parametersCovarianceMatrix() const
{
    return m_stateCovarianceMatrix;
}
