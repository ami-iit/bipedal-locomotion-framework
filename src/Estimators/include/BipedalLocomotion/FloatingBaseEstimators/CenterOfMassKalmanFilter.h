/**
 * @file CenterOfMassKalmanFilter.h
 * @authors Prashanth Ramadoss
 * @copyright 2021 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the GNU Lesser General Public License v2.1 or any later version.
 */

#ifndef BIPEDAL_LOCOMOTION_ESTIMATORS_CENTER_OF_MASS_KALMAN_FILTER_H
#define BIPEDAL_LOCOMOTION_ESTIMATORS_CENTER_OF_MASS_KALMAN_FILTER_H

#include <BipedalLocomotion/System/Source.h>
#include <iDynTree/KinDynComputations.h>
#include <Eigen/Dense>

#include <memory>

namespace BipedalLocomotion
{
namespace Estimators
{

/**
 * Internal state of a constant height CoM Kalman Filter
 */
struct CoMKFState
{
    Eigen::Vector2d comPosition;
    Eigen::Vector2d comVelocity;
    Eigen::Vector2d comOffset;
};

struct CoMKFInput
{
    Eigen::Vector2d globalCoP;
    Eigen::Vector3d acc, gyro;
};

/**
 * Implements the constant height LIPM model based Kalman filter
 * described in the paper,
 * "Center of Mass Estimator for Humanoids and its Application in
 *  Modelling Error Compensation, Fall Detection and Prevention"
 * to estimate the horizontal center of mass position and velocity
 *
 * assumptions,
 * - constant height CoM LIPM model
 * - the IMU is rigidly attached to the base link
 * - CoM is always close to the same rigid body as the IMU
 * - constant angular velocity is assumed, making angular acceleration is zero
 */
class CenterOfMassKalmanFilter : public BipedalLocomotion::System::Advanceable<CoMKFInput, CoMKFState>
{
public:
    CenterOfMassKalmanFilter();
    ~CenterOfMassKalmanFilter();
    /**
     * Initialize the estimator.
     * @param paramHandler pointer to the parameters handler.
     * @note the following parameters are required by the class
     * |               Parameter Name              |   Type   |                                       Description                                      | Mandatory |
     * |:-----------------------------------------:|:--------:|:--------------------------------------------------------------------------------------:|:---------:|
     * |              `sampling_time`              | `double` |                                    Sampling period in seconds                          |    Yes    |
     * |              `base_link_imu`              | `string` |                               name of the IMU attached to the base link                |    Yes    |
     * |   `com_position_measurement_noise_var`    | `double` |                       covariance of center of mass position measurement noise          |    Yes    |
     * | `com_acceleration_measurement_noise_var`  | `double` |                     covariance of center of mass acceleration measurement noise        |    Yes    |
     * |    `com_position_prediction_noise_var`    | `double` |                       covariance of center of mass position prediction noise           |    Yes    |
     * |    `com_velocity_prediction_noise_var`    | `double` |                       covariance of center of mass velocity prediction noise           |    Yes    |
     * |    `com_offset_prediction_noise_var`      | `double` |                       covariance of center of mass offset prediction noise             |    Yes    |
     */
    bool initialize(std::weak_ptr<const ParametersHandler::IParametersHandler> handler) override;

    /**
     * @note: This estimator will not modify the state of the KinDyn.
     * Assumes that the kindyn state is set externally before every advance() call of this method
     */
    bool setKinDynObject(std::shared_ptr<iDynTree::KinDynComputations> kinDyn);
    bool setInitialState(const CoMKFState& initialState);
    bool setInput(const CoMKFInput& input) override;

    bool setGlobalCenterOfPressure(const double& copX, const double& copY);
    bool setBaseLinkIMUMeasurement(Eigen::Ref<Eigen::Vector3d> acc,
                                   Eigen::Ref<Eigen::Vector3d> gyro);
    bool advance() override;

    const CoMKFState& getOutput() const override;
    bool isOutputValid() const override;
private:
    class Impl;
    std::unique_ptr<Impl> m_pimpl;
};

} // namespace Estimators
} // namespace BipedalLocomotion


#endif // BIPEDAL_LOCOMOTION_ESTIMATORS_CENTER_OF_MASS_KALMAN_FILTER_H
