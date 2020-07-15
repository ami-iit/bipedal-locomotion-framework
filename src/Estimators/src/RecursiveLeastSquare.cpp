/**
 * @file RecursiveLeastSquare.cpp
 * @authors Giulio Romualdi
 * @copyright 2020 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the GNU Lesser General Public License v2.1 or any later version.
 */

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
    Eigen::VectorXd measurementCovariance;
    if (!handler->getParameter("measurement_covariance",
                               measurementCovariance,
                               VectorResizeMode::Resizable))
    {
        std::cerr << "[RecursiveLeastSquare::initialize] Unable to find the covariance matrix of "
                     "the measuraments."
                  << std::endl;
        return false;
    }

    m_measurementCovarianceMatrix.resize(measurementCovariance.size(), measurementCovariance.size());
    m_measurementCovarianceMatrix = measurementCovariance.asDiagonal();

    // get the lambda
    if (!handler->getParameter("lambda", m_lambda))
    {
        std::cerr << "[RecursiveLeastSquare::initialize] Unable to find lambda." << std::endl;
        return false;
    }

    // check if the presence of the initial initial guess of the state and of the covariance matrix
    if (!handler->getParameter("state", m_state.expectedValue, VectorResizeMode::Resizable))
    {
        std::cerr << "[RecursiveLeastSquare::initialize] Unable to get the initial guess."
                  << std::endl;
        return false;
    }

    Eigen::VectorXd stateCovariance;
    if(!handler->getParameter("state_covariance", stateCovariance, VectorResizeMode::Resizable))
    {
        std::cerr << "[RecursiveLeastSquare::initialize] Unable to get the initial state covariance."
                  << std::endl;
        return false;
    }

    m_state.covariance.resize(m_state.expectedValue.size(), m_state.expectedValue.size());
    m_state.covariance= stateCovariance.asDiagonal();

    // resize the vector containing the measuraments
    m_measurements.resize(measurementCovariance.size());
    m_measurements.setZero();

    // resize the matrix containing the kalman gain
    m_kalmanGain.resize(m_state.expectedValue.size(), measurementCovariance.size());
    m_kalmanGain.setZero();

    m_estimatorState = State::Initialized;

    return true;
}

void RecursiveLeastSquare::setRegressorFunction(std::function<Eigen::MatrixXd(void)> regressor)
{
    m_regressorFunction = regressor;
}

void RecursiveLeastSquare::setRegressor(const Eigen::Ref<const Eigen::MatrixXd>& regressor)
{
    m_regressor = regressor;
}

bool RecursiveLeastSquare::advance()
{
    if (m_regressorFunction == nullptr && (m_regressor.rows() == 0 && m_regressor.cols() == 0))
    {
        std::cerr << "[RecursiveLeastSquare::advance] Please call the setRegressorFunction() or "
                     "setRegressor() before calling advance()."
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

    auto& expectedValue = m_state.expectedValue;
    auto& covariance = m_state.covariance;

    if(m_regressorFunction)
    {
        m_regressor = m_regressorFunction();
    }

    m_kalmanGain = covariance * m_regressor.transpose()
                   * (m_lambda * m_measurementCovarianceMatrix
                      + (m_regressor * covariance * m_regressor.transpose())).inverse();

    expectedValue += m_kalmanGain * (m_measurements - m_regressor * expectedValue);
    covariance = (covariance - m_kalmanGain * m_regressor * covariance) / m_lambda;

    return true;
}

void RecursiveLeastSquare::setMeasurements(const Eigen::Ref<const Eigen::VectorXd>& measurements)
{
    assert(m_measurements.size() == measurements.size());
    m_measurements = measurements;
}

const RecursiveLeastSquareState& RecursiveLeastSquare::get() const
{
    return m_state;
}

bool RecursiveLeastSquare::isValid() const
{
    return m_estimatorState == State::Running;
}
