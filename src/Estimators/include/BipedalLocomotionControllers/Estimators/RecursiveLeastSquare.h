/**
 * @file RecursiveLeastSquare.h
 * @authors Giulio Romualdi
 * @copyright 2020 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the GNU Lesser General Public License v2.1 or any later version.
 */

#ifndef BIPEDAL_LOCOMOTION_CONTROLLERS_ESTIMATORS_RLS_H
#define BIPEDAL_LOCOMOTION_CONTROLLERS_ESTIMATORS_RLS_H

#include <functional>
#include <memory>

#include <iDynTree/Core/MatrixDynSize.h>
#include <iDynTree/Core/VectorDynSize.h>

#include <BipedalLocomotionControllers/ParametersHandler/IParametersHandler.h>

namespace BipedalLocomotionControllers
{
namespace Estimators
{
/**
 * RecursiveLeastSquare contains the implementation of the Recursive least square filter described
 * in Lennart Ljung - System Identification Theory for the User (1999, Prentice Hall). Chapter 11
 * Section 2.
 */
class RecursiveLeastSquare
{
    iDynTree::VectorDynSize m_state; /**< Vector containing the expected value of the estimated
                                        state */
    iDynTree::VectorDynSize m_measurements; /**< Vector containing the measurements */

    iDynTree::MatrixDynSize m_stateCovarianceMatrix; /**< Covariance matrix of the state */

    /** Covariance matrix of the measurements we assume that the measurements are uncorrelated
     (furthermore since the model of the noise is Gaussian the random variable are also independent)
     Since the variable are independent we are interested only on the element in the diagonal
     of the matrix */
    iDynTree::MatrixDynSize m_measurementCovarianceMatrix;

    iDynTree::MatrixDynSize m_kalmanGain; /**< Gain of the Kalman filter */

    double m_lambda; /**< Filter gain. The recursive least square filter is equivalent to a kalman
                        filter if lambda is equal to 1 */

    std::function<iDynTree::MatrixDynSize(void)> m_regressor; /**< Function containing the regressor
                                                                 of the system */

    /**
     * Enumerator useful to described the current status of the filter
     */
    enum class State
    {
        NotInitialized, /**< The filter is not initialized yet call RecursiveLeastSquare::initialze
                           method to initialize it*/
        Initialized, /**< The filter is initialized and ready to be used */
        Running /**< The filter is running */
    };

    State m_estimatorState{State::NotInitialized}; /**< State of the filter */

public:
    /**
     * Initialize the filter
     * @note The following parameter are required by the filter:
     * - "measurement_covariance" vector containing the covariance of the measurements. We assume
     that the measurements are uncorrelated (furthermore since the model of the noise is Gaussian
     the random variable are also independent) Since the variable are independent we are interested
     only on the element in the diagonal of the matrix
     * - "lambda" double containing the filter gain. (The recursive least square filter is
     equivalent to a kalman filter if lambda is equal to 1)
     * - "state" vector containing the initial guess of the state
     * - "state_covariance" vector containing the diagonal matrix of the covariance of the state
     * @param handlerWeak weak pointer to a ParametersHandler::IParametersHandler interface
     * @tparameter Derived particular implementation of the IParameterHandler
     * @return true in case of success, false otherwise
     */
    template <class Derived>
    bool initialize(std::weak_ptr<ParametersHandler::IParametersHandler<Derived>> handlerWeak);

    /**
     * Set the regressor
     * @parameter regressor fucntion that return an iDynTree::MatrixDynSize containing the regressor
     * of the system
     */
    void setRegressorFunction(std::function<iDynTree::MatrixDynSize(void)> regressor);

    /**
     * Set the measurements
     * @parameter measurements vector containing all the measurements
     */
    void setMeasurements(const iDynTree::VectorDynSize& measurements);

    /**
     * Compute one step of the filter
     * @return true in case of success, false otherwise
     */
    bool advance();

    /**
     * Get the expected value of the estimated parameters
     * @return vector containing the expected value of the estimated parameters
     */
    const iDynTree::VectorDynSize& parametersExpectedValue() const;

    /**
     * Get the Covariance matrix of the estimated parameters
     * @return covariance matrix
     */
    const iDynTree::MatrixDynSize& parametersCovarianceMatrix() const;
};
} // namespace Estimators
} // namespace BipedalLocomotionControllers

#include "RecursiveLeastSquare.tpp"

#endif // BIPEDAL_LOCOMOTION_CONTROLLERS_ESTIMATORS_RLS_H
