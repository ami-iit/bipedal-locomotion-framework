/**
 * @file RecursiveLeastSquare.cpp
 * @authors Giulio Romualdi
 * @copyright 2020 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the GNU Lesser General Public License v2.1 or any later version.
 */

#include <iDynTree/Core/EigenHelpers.h>

#include <BipedalLocomotionControllers/Estimators/RecursiveLeastSquare.h>

using namespace BipedalLocomotionControllers::Estimators;

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
