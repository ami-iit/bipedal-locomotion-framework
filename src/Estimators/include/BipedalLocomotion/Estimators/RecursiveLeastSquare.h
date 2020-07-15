/**
 * @file RecursiveLeastSquare.h
 * @authors Giulio Romualdi
 * @copyright 2020 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the GNU Lesser General Public License v2.1 or any later version.
 */

#ifndef BIPEDAL_LOCOMOTION_ESTIMATORS_RLS_H
#define BIPEDAL_LOCOMOTION_ESTIMATORS_RLS_H

#include <functional>
#include <memory>

#include <Eigen/Dense>

#include <BipedalLocomotion/ParametersHandler/IParametersHandler.h>
#include <BipedalLocomotion/System/Advanceable.h>

namespace BipedalLocomotion
{
namespace Estimators
{
/**
 * RecursiveLeastSquare contains the implementation of the Recursive least square filter described
 * in Lennart Ljung - System Identification Theory for the User (1999, Prentice Hall). Chapter 11
 * Section 2.
 */

struct RecursiveLeastSquareState
{
    Eigen::VectorXd expectedValue; /**< Vector containing the expected value of the estimated
                                        state */
    Eigen::MatrixXd covariance; /**< Covariance matrix of the state */
};

class RecursiveLeastSquare : public System::Advanceable<RecursiveLeastSquareState>
{

    RecursiveLeastSquareState m_state; /**< State of the RLS algorithm */

    Eigen::VectorXd m_measurements; /**< Vector containing the measurements */

    /** Covariance matrix of the measurements we assume that the measurements are uncorrelated
     (furthermore since the model of the noise is Gaussian the random variable are also independent)
     Since the variable are independent we are interested only on the element in the diagonal
     of the matrix */

    Eigen::MatrixXd m_measurementCovarianceMatrix;

    Eigen::MatrixXd m_kalmanGain; /**< Gain of the Kalman filter */

    double m_lambda{1}; /**< Filter gain. The recursive least square filter is equivalent to a
                        kalman filter if lambda is equal to 1 */


    std::function<Eigen::MatrixXd(void)> m_regressorFunction; /**< Function containing the regressor
                                                                         of the system */

    Eigen::MatrixXd m_regressor; /**< Regressor matrix */

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
     * @return True in case of success, false otherwise.
     */
    bool initialize(std::weak_ptr<ParametersHandler::IParametersHandler> handlerWeak);

    /**
     * Set the regressor function.
     * @parameter regressor function that return an Eigen::MatrixXd containing the regressor
     * of the system
     * @note The user can decide to set the regressor function or set the regressor matrix by
     * calling setReressor(). If setRegressorFunction() is called, the RLS algorithm will compute
     * the regressor every time the advance() function is called.
     */
    void setRegressorFunction(std::function<Eigen::MatrixXd(void)> regressor);

    /**
     * Set the regressor function.
     * @parameter regressor is the regressor matrix.
     * @note The user can decide to set the regressor function or set the regressor matrix by
     * calling setReressor(). If setRegressorFunction() is called, the RLS algorithm will compute
     * the regressor every time the advance() function is called.
     */
    void setRegressor(const Eigen::Ref<const Eigen::MatrixXd>& regressor);

    /**
     * Set the measurements
     * @parameter measurements vector containing all the measurements
     */
    void setMeasurements(const Eigen::Ref<const Eigen::VectorXd>& measurements);

    /**
     * Compute one step of the filter
     * @return True in case of success, false otherwise
     */
    bool advance() final;

    /**
     * Get the expected value and the covariance matrix of the estimated parameters.
     * @return A struct containing the expected value and the covariance of the estimated
     * parameters.
     */
    const RecursiveLeastSquareState& get() const final;

    /**
     * Determines the validity of the object retrieved with get()
     * @return True if the object is valid, false otherwise.
     */
    bool isValid() const final;
};
} // namespace Estimators
} // namespace BipedalLocomotion

#endif // BIPEDAL_LOCOMOTION_ESTIMATORS_RLS_H
